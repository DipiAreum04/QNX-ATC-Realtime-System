#ifndef DISPLAY_H_
#define DISPLAY_H_

#include <iostream>
#include <thread>
#include <atomic>
#include <mutex>
#include <vector>
#include <string>
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/dispatch.h>
#include <unistd.h>
#include <semaphore.h>
#include "Msg_structs.h"

// Display class
class Display {
public:
    Display(); // Constructor
    ~Display(); // Destructor

    bool start();
    void join(); 

private:
    // Shared memory methods
    bool initializeSharedMemory();
    void cleanupSharedMemory();

    // Display methods
    void displayLoop();
    void render(const std::vector<msg_plane_info>& planes, int count, uint64_t timestamp);

    // Method to listen for collision warnings
    void collisionListener();

    // Shared memory variables
    int shm_fd;
    SharedMemory* shared_mem;
    sem_t* shm_sem; // Shared memory semaphore

    std::thread display_thread; // Thread to display the planes
    std::thread collision_thread; // Thread to listen for collision warnings
    std::atomic<bool> running; // Flag to indicate if the display is running

    std::mutex collision_mutex; // Mutex to synchronize access to the collision warnings
    std::vector<std::string> collision_warnings; // Vector to store the collision warnings
};

#endif /* DISPLAY_H_ */
