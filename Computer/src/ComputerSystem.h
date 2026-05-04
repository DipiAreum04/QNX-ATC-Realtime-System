#ifndef COMPUTER_SYSTEM_H
#define COMPUTER_SYSTEM_H

#include <iostream>
#include <thread>
#include <atomic>
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>
#include <chrono>
#include <vector>
#include <semaphore.h>

// Define the constraints (in meters) for the airspace
const double CONSTRAINT_X = 3000;
const double CONSTRAINT_Y = 3000;
const double CONSTRAINT_Z = 1000;

#include "Msg_structs.h"  // Include the structure definition for the message types

// ComputerSystem class
class ComputerSystem {
public:
    ComputerSystem(); // Constructor
    ~ComputerSystem(); // Destructor

    bool startMonitoring(); // Method to start monitoring the airspace
    void joinThread(); // Method to join the threads

    int operator_chid; // Channel ID for the operator input
    SharedMemory* getSharedMemory() const { return shared_mem; } // Method to get the shared memory

private:
    void monitorAirspace(); // Method to monitor the airspace
    bool initializeSharedMemory(); // Method to initialize the shared memory
    void cleanupSharedMemory(); // Method to cleanup the shared memory

    // Collision detection
    void checkCollision(uint64_t currentTime, std::vector<msg_plane_info> planes);
    bool checkAxes(msg_plane_info plane1, msg_plane_info plane2);
    bool sameSpeed(double speed1, double speed2);

    // Handle messages from operator
    void processMessage();
    void sendMessagesToComms(const Message& msg);
    void handleTimeConstraintChange(const Message& msg);
    void sendCollisionToDisplay(const Message_inter_process& msg);

    int timeConstraintCollisionFreq = 180;

    // Shared memory variables
    int shm_fd;
    SharedMemory* shared_mem;
    sem_t* shm_sem;
    std::thread monitorThread; // Thread to monitor the airspace
    std::thread monitorOperatorInput; // Thread to monitor the operator input
    std::atomic<bool> running; // Flag to indicate if the system is running

    // Flag to listen for operator input
    bool listen = true;
};

#endif // COMPUTER_SYSTEM_H
