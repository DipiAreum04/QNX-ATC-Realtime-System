#ifndef OPERATORCONSOLE_H_
#define OPERATORCONSOLE_H_

#include <iostream>
#include <sys/dispatch.h>
#include <thread>
#include "Msg_structs.h"

struct SharedMemory;

class OperatorConsole {
public:
	OperatorConsole(int comms_chid, int computer_chid, SharedMemory* shm);
    ~OperatorConsole();

private:
    void HandleConsoleInputs();
    void logCommand(const std::string& command);
    std::thread Operator_Console;
    bool exit = false;
    int comms_chid;
    int computer_chid;
    SharedMemory* shared_mem;
};



#endif /* OPERATORCONSOLE_H_ */
