#include "OperatorConsole.h"
#include <cstring>
#include <sstream>
#include <sys/neutrino.h>

// Constructor: Initialize the channels and shared memory
OperatorConsole::OperatorConsole(int comms_chid, int computer_chid, SharedMemory* shm)
    : comms_chid(comms_chid), computer_chid(computer_chid), shared_mem(shm) {
    Operator_Console = std::thread(&OperatorConsole::HandleConsoleInputs, this);
}

// Destructor: Join the thread and destroy the channel to avoid leaks
OperatorConsole::~OperatorConsole() {
    if (Operator_Console.joinable()) {
        Operator_Console.join();
    }
}

// Method to handle console inputs
void OperatorConsole::HandleConsoleInputs() {
    std::cout << "\n===== Operator Console =====" << std::endl;
    std::cout << "  1 <planeID> <Vx> <Vy> <Vz>   - Change heading" << std::endl;
    std::cout << "  2 <planeID> <x> <y> <z>      - Change position" << std::endl;
    std::cout << "  3 <planeID> <altitude>       - Change altitude" << std::endl;
    std::cout << "  4 <planeID>                  - Request augmented info" << std::endl;
    std::cout << "  5 <seconds>                  - Change time constraint" << std::endl;
    std::cout << "  0                            - Exit" << std::endl;
    std::cout << "============================\n" << std::endl;

    // Listen for inputs until the exit flag is set
    while (!exit) {
        std::cout << "> " << std::flush;
        std::string line;
        // If the input is not read, break the loop
        if (!std::getline(std::cin, line)) {
            break;
        }
        if (line.empty()) continue;

        std::istringstream iss(line);
        int cmd;
        if (!(iss >> cmd)) {
            std::cout << "Invalid input. Enter a number (0-5).\n";
            continue;
        }

        if (cmd == 0) {
            std::cout << "Exiting operator console.\n";
            exit = true;
            break;
        }

        Message_inter_process msg{};
        msg.header = true;

        // if cmd is 1, change the heading of the plane
        if (cmd == 1) {
            int planeID;
            double vx, vy, vz;
            if (!(iss >> planeID >> vx >> vy >> vz)) {
                std::cout << "Bad format. Use: 1 <planeID> <Vx> <Vy> <Vz>\n";
                continue;
            }
            msg_change_heading data{};
            data.ID = planeID;
            data.VelocityX = vx;
            data.VelocityY = vy;
            data.VelocityZ = vz;
            msg.type = MessageType::REQUEST_CHANGE_OF_HEADING;
            msg.planeID = planeID;
            msg.dataSize = sizeof(data);
            std::memcpy(msg.data.data(), &data, sizeof(data));
        }

        // if cmd is 2, change the position of the plane
        else if (cmd == 2) {
            int planeID;
            double x, y, z;
            if (!(iss >> planeID >> x >> y >> z)) {
                std::cout << "Bad format. Use: 2 <planeID> <x> <y> <z>\n";
                continue;
            }
            msg_change_position data{};
            data.x = x;
            data.y = y;
            data.z = z;
            msg.type = MessageType::REQUEST_CHANGE_POSITION;
            msg.planeID = planeID;
            msg.dataSize = sizeof(data);
            std::memcpy(msg.data.data(), &data, sizeof(data));
        }

        // if cmd is 3, change the altitude of the plane
        else if (cmd == 3) {
            int planeID;
            double altitude;
            if (!(iss >> planeID >> altitude)) {
                std::cout << "Bad format. Use: 3 <planeID> <altitude>\n";
                continue;
            }
            msg_change_heading data{};
            data.ID = planeID;
            data.altitude = altitude;
            msg.type = MessageType::REQUEST_CHANGE_ALTITUDE;
            msg.planeID = planeID;
            msg.dataSize = sizeof(data);
            std::memcpy(msg.data.data(), &data, sizeof(data));
        }  

        // if cmd is 4, fetch the augmented information of the plane
        else if (cmd == 4) {
            int planeID;
            if (!(iss >> planeID)) {
                std::cout << "Bad format. Use: 4 <planeID>\n";
                continue;
            }
            msg.type = MessageType::REQUEST_AUGMENTED_INFO;
            msg.planeID = planeID;
        }

        // if cmd is 5, change the time constraint for collisions
        else if (cmd == 5) {
            int seconds;
            if (!(iss >> seconds)) {
                std::cout << "Bad format. Use: 5 <seconds>\n";
                continue;
            }
            msg.type = MessageType::CHANGE_TIME_CONSTRAINT_COLLISIONS;
            msg.planeID = -1;
            msg.dataSize = sizeof(seconds);
            std::memcpy(msg.data.data(), &seconds, sizeof(seconds));

            int coid = ConnectAttach(0, 0, computer_chid, _NTO_SIDE_CHANNEL, 0);
            if (coid == -1) {
                std::cout << "Failed to send. Retry.\n";
                continue;
            }
            int reply;
            if (MsgSend(coid, &msg, sizeof(msg), &reply, sizeof(reply)) == -1) {
                std::cout << "Failed to send. Retry.\n";
            } else {
                std::cout << "[t=" << shared_mem->timestamp << "] OK: Time constraint updated to " << seconds << "s\n";
            }
            ConnectDetach(coid);
            continue;
        }
        
        // if cmd is unknown, print an error message
        else {
            std::cout << "Unknown command '" << cmd << "'. Use 0-5.\n";
            continue;
        }

        // Connect to the Communications System and send the message
        int coid = ConnectAttach(0, 0, comms_chid, _NTO_SIDE_CHANNEL, 0);
        if (coid == -1) {
            std::cout << "Failed to send. Retry.\n";
            continue;
        }
        // Send the message to the Communications System
        int reply;
        if (MsgSend(coid, &msg, sizeof(msg), &reply, sizeof(reply)) == -1) {
            std::cout << "Failed to send. Retry.\n";
        } else {
            std::cout << "[t=" << shared_mem->timestamp << "] OK: Command sent to Plane " << msg.planeID << "\n";
        }
        ConnectDetach(coid); // Detach from the Communications System
    }
}
