#ifndef OPERATORCONSOLE_H_
#define OPERATORCONSOLE_H_

#include <iostream>
#include <sys/dispatch.h>
#include <thread>
#include "Msg_structs.h"

struct SharedMemory;

class OperatorConsole {
public:
	OperatorConsole(int comms_chid, int computer_chid, SharedMemory* shm); // Constructor
    ~OperatorConsole(); // Destructor

private:
    void HandleConsoleInputs();
    std::thread Operator_Console; 
    bool exit = false;
    int comms_chid; // Channel ID for the Communications System
    int computer_chid; // Channel ID for the Computer System
    SharedMemory* shared_mem; // Shared memory pointer
};

#endif /* OPERATORCONSOLE_H_ */
