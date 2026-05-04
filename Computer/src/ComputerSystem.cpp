#include "ComputerSystem.h"
#include "ATCTimer.h"
#include <ctime>        // For std::time_t, std::localtime
#include <iomanip>      // For std::put_time
#include <cmath>
#include <sys/dispatch.h>
#include <sys/neutrino.h>
#include <cstring>      // For memcpy

// Define the channel names for the Computer System
#define display_channel_name "dipi_display"
#define SHM_NAME "/radar_shared_mem"
#define SHM_SEM_NAME "/radar_shm_sem"
#define COMPUTER_CHANNEL_NAME "dipi_computer"
#define COMMS_CHANNEL_NAME "dipi_comms"


// Constructor: Initialize the shared memory variables and the operator channel
ComputerSystem::ComputerSystem() : shm_fd(-1), shared_mem(nullptr), shm_sem(SEM_FAILED), running(false), operator_chid(-1) {}


// Destructor: Join the threads and cleanup the shared memory to avoid leaks
ComputerSystem::~ComputerSystem() {
    joinThread();
    cleanupSharedMemory();
}

// Method to initialize the shared memory
bool ComputerSystem::initializeSharedMemory() {
	// Open the shared memory object
	while (true) {
        // Attempt to open the shared memory object
        // In case of error, retry until successful
        shm_fd = shm_open(SHM_NAME, O_RDONLY, 0666);
        if (shm_fd == -1) {
            perror("Computer system: Error occurred while opening shared memory. Retrying...");
            std::this_thread::sleep_for(std::chrono::seconds(1));
            continue;
        }

		// Map the shared memory object into the process's address space to avoid leaks
        // In case of error, retry until successful
        shared_mem = (SharedMemory*) mmap(NULL, sizeof(SharedMemory), PROT_READ, MAP_SHARED, shm_fd, 0);

        if (shared_mem == MAP_FAILED) {
            perror("Computer system: Error occurred while mapping shared memory. Retrying...");
            close(shm_fd);
            shm_fd = -1;
            std::this_thread::sleep_for(std::chrono::seconds(1));
            continue;
        }

        // Open the semaphore for the shared memory
        shm_sem = sem_open(SHM_SEM_NAME, 0);
        if (shm_sem == SEM_FAILED) {
            perror("ComputerSystem: sem_open failed (will run without sync)");
        }
        return true;
    }
    return false;
}


// Method to cleanup the shared memory to avoid leaks
void ComputerSystem::cleanupSharedMemory() {
    // Close the semaphore for the shared memory
    if (shm_sem != SEM_FAILED) {
        sem_close(shm_sem);
        shm_sem = SEM_FAILED;
    }
    // Unmap the shared memory
    if (shared_mem && shared_mem != MAP_FAILED) {
        munmap(shared_mem, sizeof(SharedMemory));
        shared_mem = nullptr;
    }
    // Close the shared memory file descriptor
    if (shm_fd != -1) {
        close(shm_fd);
        shm_fd = -1;
    }
}


// Method to start monitoring the airspace
bool ComputerSystem::startMonitoring() {
    // Initialize the shared memory
    if (initializeSharedMemory()) {
        running = true; 
        operator_chid = ChannelCreate(0); // Create the channel for the operator input
        if (operator_chid == -1) {
            std::cerr << "ComputerSystem: Failed to create operator input channel\n";
        }
        // Start the threads for monitoring the airspace and processing the operator input
        monitorThread = std::thread(&ComputerSystem::monitorAirspace, this);
        monitorOperatorInput = std::thread(&ComputerSystem::processMessage, this);
        return true;
    } else {
        // If the shared memory is not initialized, print an error message and return false
        std::cerr << "Failed to initialize shared memory. Monitoring not started.\n";
        return false;
    }
}


// Method to join the threads to avoid leaks
void ComputerSystem::joinThread() {
    if (monitorThread.joinable()) {
        monitorThread.join();
    }
    listen = false;
    if (monitorOperatorInput.joinable()) {
        monitorOperatorInput.join();
    }
}


// Method to monitor the airspace
void ComputerSystem::monitorAirspace() {
	// Create a timer for the airspace monitoring
	ATCTimer timer(3,0); // Timer ticks every 3 seconds
	std::vector<msg_plane_info> plane_data_vector; // Vector to store plane data for the current iteration
	uint64_t timestamp; // Timestamp of the last write
    // Keep monitoring indefinitely until the shared memory is empty
	while (shared_mem->is_empty.load()) {
		timer.waitTimer();
	}

	while (running) {
		// Acquire semaphore before reading shared memory
		if (shm_sem != SEM_FAILED) sem_wait(shm_sem);

		bool isEmpty = shared_mem->is_empty.load();
		if (!isEmpty) {
			plane_data_vector.clear();
			timestamp = shared_mem->timestamp;
			int count = shared_mem->count;
			for (int i = 0; i < count; ++i) {
				plane_data_vector.push_back(shared_mem->plane_data[i]);
			}
		}

		// Release semaphore after reading
		if (shm_sem != SEM_FAILED) sem_post(shm_sem);

		if (isEmpty) {
			running = false;
	        break;
        }
		checkCollision(timestamp, plane_data_vector);
        // Sleep for a short interval before the next poll
       timer.waitTimer();
    }
	// Monitoring loop ended
}


// Method to check for collisions between planes in the airspace within the time constraint
void ComputerSystem::checkCollision(uint64_t currentTime, std::vector<msg_plane_info> planes) {
    // Iterate through each pair of planes and in case of collision, store the pair of plane IDs that are predicted to collide
    std::vector<std::pair<int, int>> collisionPairs; // Vector to store pairs of colliding planes
    for (size_t i = 0; i < planes.size(); ++i) {
        for (size_t j = i + 1; j < planes.size(); ++j) {
            if (checkAxes(planes[i], planes[j])) {
                collisionPairs.emplace_back(planes[i].id, planes[j].id);
            }
        }
    }

    // In case of collision, send message to Display system

    Message_inter_process msg_to_send{};
    msg_to_send.header = true;
    msg_to_send.planeID = -1;
    msg_to_send.type = MessageType::COLLISION_DETECTED;

    // If there are collision pairs, set the data size and copy the data to the message
    if (!collisionPairs.empty()) {
        size_t dataSize = collisionPairs.size() * sizeof(std::pair<int, int>);
        msg_to_send.dataSize = dataSize;
        std::memcpy(msg_to_send.data.data(), collisionPairs.data(), dataSize);
    } else {
        msg_to_send.dataSize = 0;
    }
    // Send the message to the Display system
    sendCollisionToDisplay(msg_to_send);
}



// Method to check if two planes will collide within the time constraint
bool ComputerSystem::checkAxes(msg_plane_info plane1, msg_plane_info plane2) {

    // If velocities are identical, relative positions never change, so we only check at time t=0
    if (sameSpeed(plane1.VelocityX, plane2.VelocityX) && sameSpeed(plane1.VelocityY, plane2.VelocityY) && sameSpeed(plane1.VelocityZ, plane2.VelocityZ)) {
        // If the planes will collide within the time constraint, return true
        if (std::fabs(plane1.PositionX - plane2.PositionX) <= CONSTRAINT_X && std::fabs(plane1.PositionY - plane2.PositionY) <= CONSTRAINT_Y && std::fabs(plane1.PositionZ - plane2.PositionZ) <= CONSTRAINT_Z) {
            return true;
        }
        else {
            return false;
        }
    }
    // Check if the planes will collide within the time constraint for unequal velocities
    for (int t = 0; t <= timeConstraintCollisionFreq; t++) {
        double futureX1 = plane1.PositionX + plane1.VelocityX * t;
        double futureY1 = plane1.PositionY + plane1.VelocityY * t;
        double futureZ1 = plane1.PositionZ + plane1.VelocityZ * t;

        double futureX2 = plane2.PositionX + plane2.VelocityX * t;
        double futureY2 = plane2.PositionY + plane2.VelocityY * t;
        double futureZ2 = plane2.PositionZ + plane2.VelocityZ * t;

        // If the planes will collide within the time constraint, return true
        if (std::fabs(futureX1 - futureX2) <= CONSTRAINT_X && std::fabs(futureY1 - futureY2) <= CONSTRAINT_Y && std::fabs(futureZ1 - futureZ2) <= CONSTRAINT_Z) {
            return true;
        }
    }
    return false;
}


// Method to send the collision message to the Display system
void ComputerSystem::sendCollisionToDisplay(const Message_inter_process& msg){
    // Open the channel for the Display system
	int display_channel = name_open(display_channel_name, 0);
	if (display_channel == -1) {
        return;
	}
    // Send the message to the Display system
	int reply;

	int status = MsgSend(display_channel, &msg, sizeof(msg), &reply, sizeof(reply));
	if (status == -1) {
		perror("Computer system: Error occurred while sending message to display channel");
	}
    name_close(display_channel);
}


// Method to check if two planes have the same speed
bool ComputerSystem::sameSpeed(double speed1, double speed2) {
    const double NEGLIGIBLE_DIFFERENCE = 1e-6;
    return std::fabs(speed1 - speed2) < NEGLIGIBLE_DIFFERENCE;
}


// Method to process messages from the operator
void ComputerSystem::processMessage() {
    if (operator_chid == -1) {
        std::cerr << "ComputerSystem: No operator channel available\n";
        return;
    }
    // Listen for messages from the operator until the listen flag is false
    while (listen) {
        Message_inter_process msg{};
        // Receive message from the operator
        int rcvid = MsgReceive(operator_chid, &msg, sizeof(msg), NULL);

        // If the message is not received, print an error message and break the loop
        if (rcvid == -1) {
            if (!listen) break;
            perror("ComputerSystem: MsgReceive failed");
            break;
        }
        if (rcvid == 0) continue;

        // Send a reply to the operator
        int reply = 0;
        MsgReply(rcvid, 0, &reply, sizeof(reply));

        // Handle different message types from the operator
        // Handles only two message types: CHANGE_TIME_CONSTRAINT_COLLISIONS and EXIT
        switch (msg.type) {
            case MessageType::CHANGE_TIME_CONSTRAINT_COLLISIONS: {
                if (msg.dataSize >= sizeof(int)) {
                    int newConstraint;
                    std::memcpy(&newConstraint, msg.data.data(), sizeof(int));
                    timeConstraintCollisionFreq = newConstraint;
                }
                break;
            }
            case MessageType::EXIT: {
                listen = false;
                break;
            }
            default:
                break;
        }
    }
}


// Method to send messages to the Communications System
void ComputerSystem::sendMessagesToComms(const Message& msg) {
    // Open the channel for the Communications System
    int coid = name_open(COMMS_CHANNEL_NAME, 0);
    if (coid == -1) {
        std::cerr << "ComputerSystem: Failed to connect to CommunicationsSystem\n";
        return;
    }

    // Create a message to send to the Communications System
    Message_inter_process ipc_msg{};
    ipc_msg.header = true;
    ipc_msg.type = msg.type;
    ipc_msg.planeID = msg.planeID;
    ipc_msg.dataSize = msg.dataSize;
    if (msg.data && msg.dataSize > 0 && msg.dataSize <= ipc_msg.data.size()) {
        std::memcpy(ipc_msg.data.data(), msg.data, msg.dataSize);
    }

    // Send the message to the Communications System
    int reply;
    int status = MsgSend(coid, &ipc_msg, sizeof(ipc_msg), &reply, sizeof(reply));
    if (status == -1) {
        perror("ComputerSystem: Failed to send message to CommunicationsSystem");
    }
    name_close(coid);
}


// Method to handle time constraint change
void ComputerSystem::handleTimeConstraintChange(const Message& msg) {
    if (msg.data && msg.dataSize >= sizeof(int)) {
        int newConstraint;
        std::memcpy(&newConstraint, msg.data, sizeof(int));
        timeConstraintCollisionFreq = newConstraint;
        std::cout << "ComputerSystem: Collision time constraint updated to "
                  << newConstraint << " seconds\n";
    }
}
