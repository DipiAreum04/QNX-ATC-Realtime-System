#include "Radar.h"
#include <sys/dispatch.h>

// Constructor
Radar::Radar(uint64_t& tick_counter) : tick_counter_ref(tick_counter), activeBufferIndex(0), timer(1,0), stopThreads(false) {

    // Open and map shared memory once for the lifetime of the Radar instance

    // Create or open shared memory object in the constructor 
	shm_fd = shm_open(SHM_NAME, O_CREAT | O_RDWR, 0666);
	if (shm_fd == -1) {
		perror("Radar: shm_open failed");
	}
    // Set the size of shared memory
    else if (ftruncate(shm_fd, SHARED_MEMORY_SIZE) == -1) {
		perror("Radar: ftruncate failed");
		close(shm_fd); // Close the shared memory file descriptor to avoid memory leaks
		shm_fd = -1;
	}
    // Map shared memory into process address space
    else {
		sharedMemPtr = (SharedMemory*) mmap(NULL, SHARED_MEMORY_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd, 0);
		if (sharedMemPtr == MAP_FAILED) {
			perror("Radar: mmap failed");
			sharedMemPtr = nullptr;
			close(shm_fd);
			shm_fd = -1;
		}
	}

	// Create a named semaphore for shared memory synchronization to avoid race conditions
	sem_unlink(SHM_SEM_NAME);
	shm_sem = sem_open(SHM_SEM_NAME, O_CREAT | O_EXCL, 0666, 1);
	if (shm_sem == SEM_FAILED) {
		perror("Radar: sem_open failed");
	}

	Radar_channel = NULL; // Initialize the radar channel to NULL

	clearSharedMemory(); // Initialize shared memory contents

	// Start threads for listening to airspace events
    Arrival_Departure = std::thread(&Radar::ListenAirspaceArrivalAndDeparture, this);
    UpdatePosition = std::thread(&Radar::ListenUpdatePosition, this);
}


// Destructor
Radar::~Radar() {
    // Join threads to ensure proper cleanup
    shutdown();
    clearSharedMemory();

    // Unmap and close shared memory once at end of lifetime
    if (sharedMemPtr != nullptr) {
        munmap(sharedMemPtr, SHARED_MEMORY_SIZE);
        sharedMemPtr = nullptr;
    }
    if (shm_fd != -1) {
        close(shm_fd);
        shm_fd = -1;
    }
    shm_unlink(SHM_NAME); // remove shared memory object

    if (shm_sem != SEM_FAILED) {
        sem_close(shm_sem);
        sem_unlink(SHM_SEM_NAME);
    }
}


// Method to shutdown the Radar instance
void Radar::shutdown() {
    // Set stop flag and wait for threads to complete
    stopThreads.store(true);

    // If the channel exists, close it properly
    if (Radar_channel) {
        name_detach(Radar_channel, 0);
        Radar_channel = NULL;
    }

    if (Arrival_Departure.joinable()) {
        Arrival_Departure.join();
    }
    if (UpdatePosition.joinable()) {
        UpdatePosition.join();
    }
}


// Method to get the current active buffer
std::vector<msg_plane_info>& Radar::getActiveBuffer() {
    return planesInAirspaceData[activeBufferIndex];
}


// Method to listen for aircraft arrivals and departures from the airspace
void Radar::ListenAirspaceArrivalAndDeparture() {
	Radar_channel = name_attach(NULL, "dipi_radar", 0);
	if (Radar_channel == NULL) {
		std::cerr << "Failed to create channel for Radar" << std::endl;
		exit(EXIT_FAILURE);
	}
	// Listening for aircraft arrivals and departures from the Airspace
    while (!stopThreads.load()) {
        Message msg;
        int rcvid = MsgReceive(Radar_channel->chid, &msg, sizeof(msg), nullptr); // Receive message from the channel
        if (rcvid == -1) {
            if (stopThreads.load()) {
                break;
            }
            std::cerr << "Error receiving airspace message: " << strerror(errno) << std::endl;
            continue;
        }

        if (rcvid == 0) {
            // QNX Pulse received - not a real message
            continue;
        }

        // Reply back to the client
        int msg_ret = msg.planeID;
        MsgReply(rcvid, 0, &msg_ret, sizeof(msg_ret)); // Send plane's ID back to airplane

        switch (msg.type) {
        case MessageType::ENTER_AIRSPACE:
            addPlaneToAirspace(msg);
            break;
        case MessageType::EXIT_AIRSPACE:
            removePlaneFromAirspace(msg.planeID);
            break;
        default:
        	// All other messages generate an error message
            std::cerr << "Unknown airspace message type" << std::endl;
        	break;
        }

    }
}


// Method to listen for update position requests from the aircraft
void Radar::ListenUpdatePosition() {

    while (!stopThreads.load()) {
    	timer.waitTimer(); // Wait for the next timer interval before polling again
    	// Only poll airspace if there are planes
        if (!planesInAirspace.empty()) {
            pollAirspace();  // Call pollAirspace() to gather position data
            writeToSharedMemory();  // Write active buffer to shared memory //For future Use
            wasAirspaceEmpty = false;
        } else if (!wasAirspaceEmpty){
        	// Only write empty buffer once after transition to empty
        	writeToSharedMemory();  // Write to shared mem when all planes have left the airspace //For future Use
        	wasAirspaceEmpty = true;  // Set flag to indicate airspace is empty
        }

    }
}


// Method to poll the airspace for aircraft positions
void Radar::pollAirspace() {

	airspaceMutex.lock();
	// Make a copy of the current planes in airspace to avoid modification during iteration
	std::unordered_set<int> planesToPoll = planesInAirspace;
	airspaceMutex.unlock();

	int inactiveBufferIndex = (activeBufferIndex + 1) % 2;
	std::vector<msg_plane_info>& inactiveBuffer = planesInAirspaceData[inactiveBufferIndex];
	inactiveBuffer.clear();

	// make channel to aircraft to get their position data
	for (int planeID: planesToPoll) {

		airspaceMutex.lock();
		bool isPlaneInAirspace = planesInAirspace.find(planeID) != planesInAirspace.end();
		airspaceMutex.unlock();
		if (isPlaneInAirspace){
			try {
				// Confirm that the plane is still in airspace
				msg_plane_info plane_info = getAircraftData(planeID);
				inactiveBuffer.emplace_back(plane_info);
			} catch (const std::exception& e) {
				// If the plane is no longer reachable, remove it from the airspace
				std::cerr << "Radar: plane " << planeID << " no longer reachable (" << e.what() << ")\n";
				std::lock_guard<std::mutex> lk(airspaceMutex);
				planesInAirspace.erase(planeID);
			}
		}
	}
	std::lock_guard<std::mutex> lock(bufferSwitchMutex);
	activeBufferIndex = inactiveBufferIndex;
}


// Method to get the aircraft data from the aircraft
msg_plane_info Radar::getAircraftData(int id) {
    std::string id_str = "dipi" + std::to_string(id);
    int plane_channel = name_open(id_str.c_str(), 0);

    if (plane_channel == -1) {
        throw std::runtime_error("Radar: Error occurred while attaching to channel");
    }

    // Create a request message to get the aircraft data
    Message requestMsg{};
    requestMsg.header = 0;
    requestMsg.type = MessageType::REQUEST_POSITION;
    requestMsg.planeID = id;
    requestMsg.data = nullptr;
    requestMsg.dataSize = 0;

    // Send the request message to the aircraft and receive the aircraft data
    msg_plane_info received_info{};
    if (MsgSend(plane_channel, &requestMsg, sizeof(requestMsg), &received_info, sizeof(received_info)) == -1) {
        name_close(plane_channel);
        throw std::runtime_error("Radar: Error occurred while sending request message to aircraft");
    }
    name_close(plane_channel);
    return received_info;
}


// Method to add a plane to the airspace
void Radar::addPlaneToAirspace(Message msg) {
	std::lock_guard<std::mutex> lock(airspaceMutex);
	int plane_data = msg.planeID;
    planesInAirspace.insert(plane_data);
    std::cout << "Plane " << msg.planeID << " added to airspace" << std::endl;
}


// Method to remove a plane from the airspace
void Radar::removePlaneFromAirspace(int planeID) {
	std::lock_guard<std::mutex> lock(airspaceMutex);
	planesInAirspace.erase(planeID);  // Directly remove the integer from the list
	std::cout << "Plane " << planeID << " removed from airspace" << std::endl;
}


// Method to write the active buffer to shared memory
void Radar::writeToSharedMemory() {
	
    // Check if shared memory is not initialized
    if (sharedMemPtr == nullptr) {
        return;
    }

    // Lock the buffer switch mutex to protect the buffer switching process
    std::lock_guard<std::mutex> lock(bufferSwitchMutex);

    // Get the active buffer based on the current active index
    std::vector<msg_plane_info>& activeBuffer = getActiveBuffer();

    // Acquire the semaphore before writing to shared memory
    if (shm_sem != SEM_FAILED) sem_wait(shm_sem);

    // Set the timestamp to the current tick counter
    sharedMemPtr->timestamp = tick_counter_ref;

    // Check if the active buffer is empty
    if (activeBuffer.empty()) {
        sharedMemPtr->is_empty.store(true);
        sharedMemPtr->count = 0;
    }
	else {
        // If the active buffer is not empty, set the is_empty flag to false and copy the data to the shared memory
        sharedMemPtr->is_empty.store(false);
        sharedMemPtr->count = activeBuffer.size();
		std::memcpy(sharedMemPtr->plane_data, activeBuffer.data(), activeBuffer.size() * sizeof(msg_plane_info));
		activeBuffer.clear();
    }

    // Release the semaphore after writing
    if (shm_sem != SEM_FAILED) sem_post(shm_sem);
}


// Method to clear the shared memory
void Radar::clearSharedMemory() {
	
    // Check if shared memory is not initialized
	if (sharedMemPtr == nullptr) {
		return;
	}
    // Clear the shared memory by setting all values to 0
	std::memset(sharedMemPtr->plane_data, 0, sizeof(sharedMemPtr->plane_data));
	sharedMemPtr->count = 0;
	sharedMemPtr->is_empty.store(true);
	sharedMemPtr->start = false;
	sharedMemPtr->timestamp = 0;
}
