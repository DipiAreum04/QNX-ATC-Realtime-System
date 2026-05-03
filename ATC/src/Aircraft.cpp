#include <iostream>
#include <iomanip>
#include <memory>
#include <cstring>
#include <cstdlib>
#include <pthread.h>
#include "Aircraft.h"
#include "ATCTimer.h"

#define Radar "dipi_radar" // Attach point for AirTrafficControl
#define Display_ID "dipi_display" // Attach point for Display

// Thread function to update the position of the aircraft
void* updatePositionThread(void* arg) {
    Aircraft* aircraft = static_cast<Aircraft*>(arg);
    return reinterpret_cast<void*>(aircraft->updatePosition());
}

// Constructor definition
Aircraft::Aircraft(int id, double x, double y, double z, double sx, double sy, double sz, int t)
    : id(id), posX(x), posY(y), posZ(z), speedX(sx), speedY(sy), speedZ(sz), arrivalTime(t), inAirspace(true) {
	message_id = -1;
	Radar_id = -1;
	airspace = {0, 100000, 0, 100000, 15000, 40000}; // Default airspace boundaries

    // Create a thread to update the position of the aircraft
    if (pthread_create(&thread_id, NULL, updatePositionThread, (void*)this) != 0) {
        std::cerr << "Error: pthread_create failed for Aircraft " << id << std::endl;
    }
    std::cout << "Aircraft " << id << " thread created successfully" << std::endl;
}

// Destructor definition
Aircraft::~Aircraft(){};

// Print current Aircraft data
void Aircraft::printInitialAircraftData() const {
    std::cout << std::left 	<< std::setw(5) << id
    						<< std::setw(5) << arrivalTime
							<< std::setw(5) << posX
							<< std::setw(5) << posY
							<< std::setw(5) << posZ
							<< std::setw(5) << speedX
							<< std::setw(5) << speedY
							<< std::setw(5) << speedZ
							<< "\n";
}

// Change the heading of the aircraft by changing the speed in the xyz direction
void Aircraft::changeHeading(double Vx, double Vy, double Vz) {
	if (Vx != 0) speedX = Vx;
	if (Vy != 0) speedY = Vy;
	if (Vz != 0) speedZ = Vz;
}


// Update the position of the aircraft
int Aircraft::updatePosition() {
    ATCTimer timer(1, 0);
    int currentTime = 0;  // Variable to track current time

    // Wait until the arrival time has passed
    while (currentTime < arrivalTime) {
        timer.waitTimer();  // Wait for 1 second
        ++currentTime;
    }

    //********SEND ENTER AIRSPACE TO RADAR*************

    // Open the channel with the Radar module
    if ((Radar_id = name_open(Radar, 0)) == -1) {
		perror("Error occurred while creating the channel with Radar");
		return EXIT_FAILURE;
	}

    // Once the arrival time is reached, send the ENTER_AIRSPACE message
    Message enterAirspaceMessage = createEnterAirspaceMessage(id);

    // Send message to the Radar using the MsgSend function
    if (MsgSend(Radar_id, &enterAirspaceMessage, sizeof(enterAirspaceMessage), 0, 0) == -1) {
        std::cout << "Failed to send enter message to Radar!\n";
        return EXIT_FAILURE;
	}

    //********SEND UPDATE POSITION TO RADAR**************

    // Create a channel to be reachable by the Radar that wants to poll the Airplane
    std::string id_str = "dipi"+std::to_string(id);  // Convert integer id to string
    const char* ID = id_str.c_str();         // Convert string to const char*
    name_attach_t* Plane_channel = name_attach(NULL, ID, 0); // Create Channel using name_attach

    // Error handling if channel creation fails
    if (Plane_channel == NULL) {
        std::cerr << "Could not attach plane ID: " << ID << " to channel\n";
        return EXIT_FAILURE;
    }

    // Start the position update loop
    while (true) {
        // Update position based on velocity since timer updates every second
        posX += speedX;
        posY += speedY;
        posZ += speedZ;

        // Debug: Print the new position
        std::cout << "Updated Position: (" << posX << ", " << posY << ", " << posZ << ")\n";

        // Check if the plane is still within the airspace boundaries
        if (posX < airspace.lower_x_boundary || posX > airspace.upper_x_boundary ||
            posY < airspace.lower_y_boundary || posY > airspace.upper_y_boundary ||
            posZ < airspace.lower_z_boundary || posZ > airspace.upper_z_boundary) {
            
            // Send exit airspace message to the Radar and exit loop if out of bounds
            Message exitAirspaceMessage = createExitAirspaceMessage(id);
            if (MsgSend(Radar_id, &exitAirspaceMessage, sizeof(exitAirspaceMessage), 0, 0) == -1) {
                std::cout << "Failed to send exit message to Radar!\n";
                return EXIT_FAILURE;
            }
            break; // Exit the loop if out of bounds
        }

        // Check for incoming position update requests from Radar
        char buffer[sizeof(Message_inter_process)];  // Buffer to handle largest message size
        int rcvid = MsgReceive(Plane_channel->chid, buffer, sizeof(buffer), NULL); // Receive message from the channel

        // Error handling if message reception fails (e.g. channel not found, etc.)
        if (rcvid == -1) {
            std::cerr << "Error receiving message from channel: " << strerror(errno) << std::endl;
            return EXIT_FAILURE;
        }

        if (rcvid > 0) { // >0 = real message; 0 = pulse; -1 = error

        	bool isInterProcess = buffer[0] & 0x01; // Check if the message is inter-process (1)

            if (isInterProcess){  // sporadic messages (inter-process)
            	Message_inter_process* receivedMsg = reinterpret_cast<Message_inter_process*>(buffer);

            	// Handle different message types from the Communication System using switch statements
                switch (receivedMsg->type) {

                    // Handle requests to change the heading of the aircraft
                    case MessageType::REQUEST_CHANGE_OF_HEADING: {
                        msg_change_heading headingCmd{};
                        // Copy the data from the received message to the headingCmd struct
                        if (receivedMsg->dataSize >= sizeof(headingCmd)) {
                            std::memcpy(&headingCmd, receivedMsg->data.data(), sizeof(headingCmd));
                            if (headingCmd.ID == id) {
                                changeHeading(headingCmd.VelocityX, headingCmd.VelocityY, headingCmd.VelocityZ);
                            }
                        }
                        MsgReply(rcvid, 0, NULL, 0);
                        break;
                    }

                    // Handle requests to change the position of the aircraft
                    case MessageType::REQUEST_CHANGE_POSITION: {
                        msg_change_position positionCmd{};
                        if (receivedMsg->dataSize >= sizeof(positionCmd)) {
                            std::memcpy(&positionCmd, receivedMsg->data.data(), sizeof(positionCmd));
                            // Calculate the new speed to reach the target position in ~3 seconds
                            const int transitionTime = 3; // reach target in ~3 seconds
                            speedX = (positionCmd.x - posX) / transitionTime;
                            speedY = (positionCmd.y - posY) / transitionTime;
                            speedZ = (positionCmd.z - posZ) / transitionTime;
                        }
                        MsgReply(rcvid, 0, NULL, 0);
                        break;
                    }

                    // Handle requests to change the altitude of the aircraft
                    case MessageType::REQUEST_CHANGE_ALTITUDE: {
                        msg_change_heading altitudeCmd{};
                        if (receivedMsg->dataSize >= sizeof(altitudeCmd)) {
                            std::memcpy(&altitudeCmd, receivedMsg->data.data(), sizeof(altitudeCmd));
                            if (altitudeCmd.ID == id) {
                                posZ = altitudeCmd.altitude; // Set the new altitude to the z-coordinate
                            }
                        }
                        MsgReply(rcvid, 0, NULL, 0);
                        break;
                    }

                    // Handle requests for augmented information
                    case MessageType::REQUEST_AUGMENTED_INFO: {
                        msg_plane_info info = {id, posX, posY, posZ, speedX, speedY, speedZ};
                        Message_inter_process reply_msg{};
                        reply_msg.header = true;
                        reply_msg.type = MessageType::REQUEST_AUGMENTED_INFO;
                        reply_msg.planeID = id;
                        reply_msg.dataSize = sizeof(info);
                        std::memcpy(reply_msg.data.data(), &info, sizeof(info));
                        MsgReply(rcvid, 0, &reply_msg, sizeof(reply_msg));
                        break;
                    }

                    // Handle requests to change the time constraint for collisions
                    case MessageType::CHANGE_TIME_CONSTRAINT_COLLISIONS: {
                        // This request is implemented in the ComputerSystem class
                        // So, here just print received message to console and reply with a NULL message
                        std::cout << "Aircraft " << id << ": Received time constraint change notification\n";
                        MsgReply(rcvid, 0, NULL, 0);
                        break;
                    }

                    // Handle requests to exit the aircraft
                    case MessageType::EXIT: {
                        int exitCmd = EXIT_FAILURE;
                        if (receivedMsg->dataSize >= sizeof(exitCmd)) {
                            std::memcpy(&exitCmd, receivedMsg->data.data(), sizeof(exitCmd));
                        }
                        std::cout << "Aircraft " << id << " shutting down.\n";
                        MsgReply(rcvid, 0, NULL, 0);
                        name_detach(Plane_channel, 0);
                        name_close(Radar_id);
                        pthread_exit(NULL);
                        break;
                    }

                    // Handle requests for collision detection
                    case MessageType::COLLISION_DETECTED: {
                        // This request is implemented in the ComputerSystem class
                        // So, here just print received message to console and reply with a NULL message
                        std::cout << "Aircraft " << id << ": Collision detected\n";
                        MsgReply(rcvid, 0, NULL, 0);
                        break;
                    }

                    // Handle unknown message types
                    default: {
                        std::cout << "Unknown inter-process message received\n";
                        MsgReply(rcvid, 0, NULL, 0);
                        break;
                    }
                }
            } else {  // periodic messages (intra-process): rcvid = 0

            	Message* receivedMsg = reinterpret_cast<Message*>(buffer);
                // Handle requests for position data
            	if (receivedMsg->type == MessageType::REQUEST_POSITION) {
            		msg_plane_info positionData = {id, posX, posY, posZ, speedX, speedY, speedZ};
            	    MsgReply(rcvid, 0, &positionData, sizeof(positionData));
            	} else { // Handle unknown message types
                    MsgReply(rcvid, 0, NULL, 0);
                }
            }
        }

        // Wait for the next time step
        timer.waitTimer();
    }

    name_detach(Plane_channel, 0);
    pthread_exit(NULL);

    return 0;
}


int Aircraft::getArrivalTime() {
	return arrivalTime;
}

int Aircraft::getID(){
	return id;
}

// Create the ENTER_AIRSPACE message
Message Aircraft::createEnterAirspaceMessage(int planeID) {
	Message msg;
    msg.header = 0; // intra-process
    msg.type = MessageType::ENTER_AIRSPACE;
    msg.planeID = planeID;
    msg.data = NULL;  // Allocate dynamically and copy info data
    msg.dataSize = 0;
    return msg;
}

// Create the EXIT_AIRSPACE message
Message Aircraft::createExitAirspaceMessage(int planeID) {

	Message msg;
    msg.header = 0; // intra-process
    msg.type = MessageType::EXIT_AIRSPACE;
    msg.planeID = planeID;
    msg.data = NULL;  // Allocate dynamically and copy info data
    msg.dataSize = 0;
    return msg;
}