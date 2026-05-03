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

    Communications_System = std::thread(&CommunicationsSystem::HandleCommunications, this);
}

// Destructor: Join the thread and destroy the channel
CommunicationsSystem::~CommunicationsSystem() {
    if (Communications_System.joinable()) {
        Communications_System.join();
    }
    if (chid != -1) {
        ChannelDestroy(chid);
    }
}

// Handle communications with the Aircraft
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

        // Send a reply to the Computer System
        int reply = 0;
        MsgReply(rcvid, 0, &reply, sizeof(reply));

        if (msg.type == MessageType::EXIT) {
            break;
        }

        Message toAircraft{};
        toAircraft.header = true;
        toAircraft.type = msg.type;
        toAircraft.planeID = msg.planeID;
        toAircraft.data = msg.data.data();
        toAircraft.dataSize = msg.dataSize;
        messageAircraft(toAircraft);
    }
}

// Send a message to the Aircraft
void CommunicationsSystem::messageAircraft(const Message& msg) {
    std::string channel = "dipi" + std::to_string(msg.planeID); // Create the channel name for the Aircraft
    int coid = name_open(channel.c_str(), 0); // Open the channel for the Aircraft
    if (coid == -1) {
        std::cerr << "CommunicationsSystem: Failed to connect to aircraft " << msg.planeID << "\n";
        return;
    }

    // Create a message to send to the Aircraft
    Message_inter_process ipc_msg{};
    ipc_msg.header = true;
    ipc_msg.type = msg.type;
    ipc_msg.planeID = msg.planeID;
    ipc_msg.dataSize = msg.dataSize;
    if (msg.data && msg.dataSize > 0 && msg.dataSize <= ipc_msg.data.size()) {
        std::memcpy(ipc_msg.data.data(), msg.data, msg.dataSize);
    }

    int reply;
    int status = MsgSend(coid, &ipc_msg, sizeof(ipc_msg), &reply, sizeof(reply));
    if (status == -1) {
        perror("CommunicationsSystem: Failed to send message to aircraft");
    }

    name_close(coid);
}
