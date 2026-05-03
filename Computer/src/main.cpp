#include "ComputerSystem.h"
#include "OperatorConsole.h"
#include "CommunicationsSystem.h"

int main() {

    // Create the Communications System and Computer System
    CommunicationsSystem comms;
    ComputerSystem computerSystem;

    // Start monitoring the airspace
    if (computerSystem.startMonitoring()) {
        // Create the Operator Console
        OperatorConsole console(comms.chid, computerSystem.operator_chid, computerSystem.getSharedMemory());
        // Join the threads
        computerSystem.joinThread();
    } else {
        std::cerr << "Failed to start monitoring." << std::endl;
    }

    std::cout << "Monitoring stopped. Exiting main." << std::endl;

    return 0;
}
