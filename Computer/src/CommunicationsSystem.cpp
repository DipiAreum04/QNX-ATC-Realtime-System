#include "CommunicationsSystem.h"
#include <cstring>
#include <sys/neutrino.h>

// Constructor: Create the channel for the Communications System and start the thread
CommunicationsSystem::CommunicationsSystem() : chid(-1) {
    chid = ChannelCreate(0);
    if (chid == -1) {
        std::cerr << "CommunicationsSystem: Failed to create channel\n";
        return;
    }
    // Start the thread and attach it to the HandleCommunications method
    Communications_System = std::thread(&CommunicationsSystem::HandleCommunications, this);
}


// Destructor: Join the thread and destroy the channel to avoid leaks
CommunicationsSystem::~CommunicationsSystem() {
    if (Communications_System.joinable()) {
        Communications_System.join();
    }
    if (chid != -1) {
        ChannelDestroy(chid);
    }
}


// Method to handle communications with the aircraft
void CommunicationsSystem::HandleCommunications() {
    // Listen for messages from the Computer System
    while (true) {
        // Receive message from the channel
        Message_inter_process msg{};
        int rcvid = MsgReceive(chid, &msg, sizeof(msg), NULL);

        // If the message is not received, print an error message and break the loop
        if (rcvid == -1) {
            perror("CommunicationsSystem: MsgReceive failed");
            break;
        }

        // If rcvid is 0, it means QNX Pulse received - not a real message
        if (rcvid == 0) {
            continue;
        }

        if (msg.type == MessageType::EXIT) {
            int reply = 0;
            MsgReply(rcvid, 0, &reply, sizeof(reply));
            break;
        }

        // If the message type is REQUEST_AUGMENTED_INFO, send a message to the aircraft and wait for the reply
        if (msg.type == MessageType::REQUEST_AUGMENTED_INFO) {
            Message toAircraft{};
            toAircraft.header = true;
            toAircraft.type = msg.type;
            toAircraft.planeID = msg.planeID;
            toAircraft.data = nullptr;
            toAircraft.dataSize = 0;
            Message_inter_process aircraftReply{};
            if (messageAircraftAugmented(toAircraft, aircraftReply)) {
                MsgReply(rcvid, 0, &aircraftReply, sizeof(aircraftReply));
            } else {
                aircraftReply.header = true;
                aircraftReply.type = MessageType::REQUEST_AUGMENTED_INFO;
                aircraftReply.planeID = msg.planeID;
                aircraftReply.dataSize = 0;
                MsgReply(rcvid, -1, &aircraftReply, sizeof(aircraftReply));
            }
            continue;
        }

        int reply = 0;
        MsgReply(rcvid, 0, &reply, sizeof(reply));

        Message toAircraft{};
        toAircraft.header = true;
        toAircraft.type = msg.type;
        toAircraft.planeID = msg.planeID;
        toAircraft.data = msg.data.data();
        toAircraft.dataSize = msg.dataSize;
        messageAircraft(toAircraft);
    }
}


// Method to send a message to the aircraft
void CommunicationsSystem::messageAircraft(const Message& msg) {
    std::string channel = "dipi" + std::to_string(msg.planeID); // Create the channel name for the aircraft
    int coid = name_open(channel.c_str(), 0); // Open the channel for the aircraft

    // If the channel is not opened, print an error message and return
    if (coid == -1) {
        std::cerr << "CommunicationsSystem: Failed to connect to aircraft " << msg.planeID << "\n";
        return;
    }

    // Create a message to send to the aircraft
    Message_inter_process ipc_msg{};
    ipc_msg.header = true;
    ipc_msg.type = msg.type;
    ipc_msg.planeID = msg.planeID;
    ipc_msg.dataSize = msg.dataSize;
    if (msg.data && msg.dataSize > 0 && msg.dataSize <= ipc_msg.data.size()) {
        std::memcpy(ipc_msg.data.data(), msg.data, msg.dataSize);
    }

    // Send the message to the aircraft
    int reply;
    int status = MsgSend(coid, &ipc_msg, sizeof(ipc_msg), &reply, sizeof(reply));
    if (status == -1) {
        perror("CommunicationsSystem: Failed to send message to aircraft");
    }
    
    // Close the channel to avoid leaks
    name_close(coid);
}


// Method to send a message to the aircraft and check if the aircraft replied with a full msg_plane_info payload
bool CommunicationsSystem::messageAircraftAugmented(const Message& msg, Message_inter_process& outReply) {
    std::string channel = "dipi" + std::to_string(msg.planeID);
    int coid = name_open(channel.c_str(), 0);
    if (coid == -1) {
        std::cerr << "CommunicationsSystem: Failed to connect to aircraft " << msg.planeID << "\n";
        return false;
    }

    // Create a message to send to the aircraft
    Message_inter_process ipc_msg{};
    ipc_msg.header = true;
    ipc_msg.type = msg.type;
    ipc_msg.planeID = msg.planeID;
    ipc_msg.dataSize = msg.dataSize;
    if (msg.data && msg.dataSize > 0 && msg.dataSize <= ipc_msg.data.size()) {
        std::memcpy(ipc_msg.data.data(), msg.data, msg.dataSize);
    }

    // Send the message to the aircraft
    outReply = Message_inter_process{};
    int status = MsgSend(coid, &ipc_msg, sizeof(ipc_msg), &outReply, sizeof(outReply));
    name_close(coid);

    // If the message is not sent, print an error message and return false
    if (status == -1) {
        perror("CommunicationsSystem: Failed to send augmented-info request to aircraft");
        return false;
    }
    return outReply.dataSize >= sizeof(msg_plane_info);
}
