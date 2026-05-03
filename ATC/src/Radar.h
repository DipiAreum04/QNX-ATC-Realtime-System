#ifndef RADAR_H
#define RADAR_H

#include <atomic>       // for atomic flag usage
#include <iostream>
#include <unordered_set>
#include <memory>
#include <mutex>
#include <thread>
#include <string>
#include <vector>
#include <sys/mman.h>   // for shared memory
#include <fcntl.h>      // for O_CREAT, O_RDWR
#include <sys/stat.h>   // for mode constants
#include <unistd.h>     // for ftruncate, mmap
#include <cstring>      // for memset
#include <semaphore.h>  // for POSIX named semaphores
#include "Aircraft.h"
#include "Msg_structs.h"
#include "ATCTimer.h"

// Shared memory name
#define SHM_NAME "/radar_shared_mem"
#define SHM_SEM_NAME "/radar_shm_sem"

// Shared memory size
#define SHARED_MEMORY_SIZE sizeof(SharedMemory)  // Update this based on the size of the shared memory buffer

// Radar class
class Radar {
public:
	Radar(uint64_t& tick_counter); // Constructor
    ~Radar(); // Destructor

    void ListenAirspaceArrivalAndDeparture();
    void ListenUpdatePosition();

    // Shared memory write method
    void writeToSharedMemory();
    void clearSharedMemory();


private:

    uint64_t& tick_counter_ref;  // Store tick_counter as a reference
    std::unordered_set<int> planesInAirspace ; // Set to store the IDs of the planes in the airspace

    std::thread Arrival_Departure;
    std::thread UpdatePosition;

    void addPlaneToAirspace(Message msg);
    void removePlaneFromAirspace(int ID);
    void pollAirspace();
    msg_plane_info getAircraftData(int id);

    name_attach_t* Radar_channel;

    // Mutexes to protect the airspace and buffer switching
    std::mutex airspaceMutex;
    std::mutex bufferSwitchMutex;

    std::vector<msg_plane_info>& getActiveBuffer();

    // Vector to store the aircraft data in the airspace for two buffers
    std::vector<msg_plane_info> planesInAirspaceData[2];

    // Atomic integer to track the active buffer index
    std::atomic<int> activeBufferIndex; // Index of the active buffer

    ATCTimer timer;

    // Shared memory pointer
    SharedMemory* sharedMemPtr = nullptr;
    sem_t* shm_sem = SEM_FAILED; // Initialize the semaphore to SEM_FAILED
    bool wasAirspaceEmpty = true;  // Track if airspace was empty last time
    int shm_fd = -1;
    std::atomic<bool> stopThreads;

    // Method to shutdown the Radar instance
    void shutdown();


};

#endif /* RADAR_H_ */
