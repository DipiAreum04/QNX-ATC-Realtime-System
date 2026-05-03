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

class Display {
public:
    Display();
    ~Display();

    bool start();
    void join();

private:
    bool initializeSharedMemory();
    void cleanupSharedMemory();

    void displayLoop();
    void collisionListener();
    void render(const std::vector<msg_plane_info>& planes, int count, uint64_t timestamp);

    int shm_fd;
    SharedMemory* shared_mem;
    sem_t* shm_sem;

    std::thread display_thread;
    std::thread collision_thread;
    std::atomic<bool> running;

    std::mutex collision_mutex;
    std::vector<std::string> collision_warnings;
};

#endif /* DISPLAY_H_ */
