#include "Display.h"
#include "ATCTimer.h"
#include <iomanip>
#include <sstream>
#include <string>
#include <cstring>
#include <chrono>

namespace {

// Inner width between Unicode box sides ║ … ║ (must match top/rule/bottom lines).
// Inner text must be plain ASCII so byte length equals terminal cell width.
constexpr int kBoxInnerWidth = 82;

std::string padInner(std::string s) {
    if (s.size() > static_cast<size_t>(kBoxInnerWidth)) {
        s.resize(static_cast<size_t>(kBoxInnerWidth));
    } else {
        s.append(static_cast<size_t>(kBoxInnerWidth) - s.size(), ' ');
    }
    return s;
}

void printBoxLine(std::ostream& os, const std::string& inner) {
    os << u8"║" << padInner(inner) << u8"║" << '\n';
}

} // namespace


#define SHM_NAME "/radar_shared_mem"
#define SHM_SEM_NAME "/radar_shm_sem"
#define DISPLAY_CHANNEL_NAME "dipi_display"


// Constructor: Initialize the shared memory variables
Display::Display() : shm_fd(-1), shared_mem(nullptr), shm_sem(SEM_FAILED), running(false) {}


// Destructor: Join the threads and cleanup the shared memory to avoid leaks
Display::~Display() {
    running = false;
    if (display_thread.joinable()) {
        display_thread.join();
    }
    if (collision_thread.joinable()) {
        collision_thread.join();
    }
    cleanupSharedMemory();
}


// Method to initialize the shared memory
bool Display::initializeSharedMemory() {
    // In case of error, retry until successful
    while (true) {
        // Open the shared memory object in read only mode
        shm_fd = shm_open(SHM_NAME, O_RDONLY, 0666); 
        if (shm_fd == -1) {
            std::cerr << "Display: Waiting for shared memory...\n";
            std::this_thread::sleep_for(std::chrono::seconds(1));
            continue;
        }

        // Map the shared memory object into the process's address space to avoid leaks
        shared_mem = (SharedMemory*)mmap(NULL, sizeof(SharedMemory), PROT_READ, MAP_SHARED, shm_fd, 0);
        if (shared_mem == MAP_FAILED) {
            std::cerr << "Display: mmap failed, retrying...\n";
            close(shm_fd);
            shm_fd = -1;
            std::this_thread::sleep_for(std::chrono::seconds(1));
            continue;
        }

        std::cout << "Display: Shared memory initialized.\n";

        // Open the semaphore for the shared memory
        shm_sem = sem_open(SHM_SEM_NAME, 0);
        if (shm_sem == SEM_FAILED) {
            perror("Display: sem_open failed (will run without sync)");
        }
        return true;
    }
    return false;
}


// Method to cleanup the shared memory to avoid leaks
void Display::cleanupSharedMemory() {
    // Close the semaphore for the shared memory
    if (shm_sem != SEM_FAILED) {
        sem_close(shm_sem);
        shm_sem = SEM_FAILED;
    }
    // Unmap the shared memory
    if (shared_mem && shared_mem != MAP_FAILED) {
        munmap(shared_mem, sizeof(SharedMemory));
        shared_mem = nullptr;
    }
    // Close the shared memory file descriptor
    if (shm_fd != -1) {
        close(shm_fd);
        shm_fd = -1;
    }
}


// Method to start the display
bool Display::start() {
    // Initialize the shared memory
    if (!initializeSharedMemory()) {
        // If the shared memory is not initialized, return false
        return false;
    }
    running = true;
    // Start the threads for the display and collision listener
    collision_thread = std::thread(&Display::collisionListener, this);
    display_thread = std::thread(&Display::displayLoop, this);
    return true;
}


// Method to join the threads to avoid leaks
void Display::join() {
    if (display_thread.joinable()) {
        display_thread.join();
    }
    if (collision_thread.joinable()) {
        collision_thread.join();
    }
}


// Method to listen for collision warnings
void Display::collisionListener() {
    // Attach to the channel for the collision warnings
    name_attach_t* attach = NULL; 
    for (int attempt = 0; attempt < 5 && attach == NULL; ++attempt) { // Retry up to 5 times if the attach fails
        attach = name_attach(NULL, DISPLAY_CHANNEL_NAME, 0);
        if (attach == NULL) {
            std::cerr << "Display: name_attach failed (attempt " << attempt + 1 << "), retrying...\n";
            std::this_thread::sleep_for(std::chrono::seconds(1)); // Sleep for 1 second before retrying
        }
    }
    // If the attach fails, return
    if (attach == NULL) {
        std::cerr << "Display: Failed to attach channel '" << DISPLAY_CHANNEL_NAME << "'\n";
        return;
    }
    std::cout << "Display: Listening for collision warnings on '" << DISPLAY_CHANNEL_NAME << "'\n";

    // Listen for collision warnings until the threads stop running
    while (running) {
        // Receive the message from the channel
        Message_inter_process msg{};
        int rcvid = MsgReceive(attach->chid, &msg, sizeof(msg), NULL);

        if (rcvid == -1) {
            if (!running) break;
            perror("Display: MsgReceive failed");
            continue;
        }
        if (rcvid == 0) {
            continue;
        }

        // Send a reply to the channel
        int reply = 0;
        MsgReply(rcvid, 0, &reply, sizeof(reply));

        // If the message type is COLLISION_DETECTED, add the collision warnings to the vector
        if (msg.type == MessageType::COLLISION_DETECTED) {
            size_t numPairs = msg.dataSize / sizeof(std::pair<int, int>);
            std::vector<std::pair<int, int>> pairs(numPairs);
            std::memcpy(pairs.data(), msg.data.data(), msg.dataSize);

            std::lock_guard<std::mutex> lock(collision_mutex); // Lock the mutex to synchronize access to the collision warnings
            collision_warnings.clear();
            for (const auto& p : pairs) {
                std::ostringstream oss; // Create a string stream to format the collision warning
                oss << "*** COLLISION WARNING: Plane " << p.first
                    << " <-> Plane " << p.second << " ***";
                collision_warnings.push_back(oss.str()); // Add the collision warning to the vector
            }
        }
    }

    // Detach from the channel
    name_detach(attach, 0);
}


// Method to display the planes
void Display::displayLoop() {
    ATCTimer timer(1, 0); // Create a timer to wait for 1 second

    // Wait for the aircraft data to be available
    while (shared_mem->is_empty.load()) {
        std::cout << "Display: Waiting for aircraft data...\n";
        timer.waitTimer();
    }

    // Display the planes until the threads stop running
    while (running) {
        // Acquire semaphore before reading shared memory
        if (shm_sem != SEM_FAILED) sem_wait(shm_sem);

        bool isEmpty = shared_mem->is_empty.load();
        std::vector<msg_plane_info> planes;
        int count = 0;
        uint64_t timestamp = 0;
        // Add the aircraft data (count, timestamp and planes) to the planes vector
        if (!isEmpty) {
            count = shared_mem->count;
            timestamp = shared_mem->timestamp;
            for (int i = 0; i < count; ++i) {
                planes.push_back(shared_mem->plane_data[i]);
            }
        }

        // Release semaphore after reading
        if (shm_sem != SEM_FAILED) sem_post(shm_sem);

        // If the aircraft data is empty, stop the display
        if (isEmpty) {
            std::cout << "Display: No aircraft in airspace. Stopping.\n";
            running = false;
            break;
        }

        // Render the planes
        render(planes, count, timestamp);
        timer.waitTimer();
    }
}


// Method to render the planes
void Display::render(const std::vector<msg_plane_info>& planes, int count, uint64_t timestamp) {

    std::cout << "\033[2J\033[H"; // Clear the screen

    // Print the header
    std::cout << "╔══════════════════════════════════════════════════════════════════════════════════╗\n";
    printBoxLine(std::cout, "                         AIR TRAFFIC CONTROL - DISPLAY                            ");
    std::cout << "╠══════════════════════════════════════════════════════════════════════════════════╣\n";

    {
        std::ostringstream row;
        row << "  Timestamp: " << std::left << std::setw(12) << timestamp
            << "  Aircraft in airspace: " << std::setw(3) << std::right << count;
        printBoxLine(std::cout, row.str());
    }

    std::cout << "╠══════════════════════════════════════════════════════════════════════════════════╣\n";

    {
        std::ostringstream row;
        // ASCII " | " only (Unicode │ is multi-byte: padInner counts bytes, not columns).
        row << ' ' << std::left << std::setw(4) << "ID"
            << " | " << std::right << std::setw(8) << "Pos X"
            << " | " << std::setw(8) << "Pos Y"
            << " | " << std::setw(8) << "Pos Z"
            << " | " << std::setw(8) << "Vel X"
            << " | " << std::setw(8) << "Vel Y"
            << " | " << std::setw(8) << "Vel Z";
        printBoxLine(std::cout, row.str());
    }

    std::cout << "╠══════════════════════════════════════════════════════════════════════════════════╣\n";

    // Print the plane data rows (8-char numeric columns so row fits kBoxInnerWidth)
    for (const auto& p : planes) {
        std::ostringstream row;
        row << ' ' << std::left << std::setw(4) << p.id
            << " | " << std::right << std::fixed << std::setprecision(1) << std::setw(8) << p.PositionX
            << " | " << std::setw(8) << p.PositionY
            << " | " << std::setw(8) << p.PositionZ
            << " | " << std::setw(8) << p.VelocityX
            << " | " << std::setw(8) << p.VelocityY
            << " | " << std::setw(8) << p.VelocityZ;
        printBoxLine(std::cout, row.str());
    }

    if (planes.empty()) {
        printBoxLine(std::cout, "  (no aircraft data)");
    }

    std::cout << "╠══════════════════════════════════════════════════════════════════════════════════╣\n";

    // Collision warnings section
    {
        std::lock_guard<std::mutex> lock(collision_mutex);
        if (collision_warnings.empty()) {
            printBoxLine(std::cout, "  Status: ALL CLEAR - No collision warnings");
        } else {
            std::cout << "║  \033[1;31m" << std::left << std::setw(80) << "!!! COLLISION ALERTS !!!"
                      << "\033[0m║\n";
            for (const auto& warning : collision_warnings) {
                std::string w = warning;
                if (w.size() > 80) {
                    w.resize(80);
                }
                std::cout << "║  \033[1;31m" << std::left << std::setw(80) << w << "\033[0m║\n";
            }
        }
    }

    std::cout << "╚══════════════════════════════════════════════════════════════════════════════════╝\n";
    std::cout << std::flush;
}
