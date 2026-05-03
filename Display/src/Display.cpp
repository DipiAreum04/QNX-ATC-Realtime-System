#include "Display.h"
#include "ATCTimer.h"
#include <iomanip>
#include <sstream>
#include <cstring>
#include <chrono>

#define SHM_NAME "/radar_shared_mem"
#define SHM_SEM_NAME "/radar_shm_sem"
#define DISPLAY_CHANNEL_NAME "dipi_display"

Display::Display() : shm_fd(-1), shared_mem(nullptr), shm_sem(SEM_FAILED), running(false) {}

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

bool Display::initializeSharedMemory() {
    while (true) {
        shm_fd = shm_open(SHM_NAME, O_RDONLY, 0666);
        if (shm_fd == -1) {
            std::cerr << "Display: Waiting for shared memory...\n";
            std::this_thread::sleep_for(std::chrono::seconds(1));
            continue;
        }

        shared_mem = (SharedMemory*)mmap(NULL, sizeof(SharedMemory), PROT_READ, MAP_SHARED, shm_fd, 0);
        if (shared_mem == MAP_FAILED) {
            std::cerr << "Display: mmap failed, retrying...\n";
            close(shm_fd);
            shm_fd = -1;
            std::this_thread::sleep_for(std::chrono::seconds(1));
            continue;
        }

        std::cout << "Display: Shared memory initialized.\n";

        shm_sem = sem_open(SHM_SEM_NAME, 0);
        if (shm_sem == SEM_FAILED) {
            perror("Display: sem_open failed (will run without sync)");
        }

        return true;
    }
    return false;
}

void Display::cleanupSharedMemory() {
    if (shm_sem != SEM_FAILED) {
        sem_close(shm_sem);
        shm_sem = SEM_FAILED;
    }
    if (shared_mem && shared_mem != MAP_FAILED) {
        munmap(shared_mem, sizeof(SharedMemory));
        shared_mem = nullptr;
    }
    if (shm_fd != -1) {
        close(shm_fd);
        shm_fd = -1;
    }
}

bool Display::start() {
    if (!initializeSharedMemory()) {
        return false;
    }
    running = true;
    collision_thread = std::thread(&Display::collisionListener, this);
    display_thread = std::thread(&Display::displayLoop, this);
    return true;
}

void Display::join() {
    if (display_thread.joinable()) {
        display_thread.join();
    }
    if (collision_thread.joinable()) {
        collision_thread.join();
    }
}

void Display::collisionListener() {
    name_attach_t* attach = NULL;
    for (int attempt = 0; attempt < 5 && attach == NULL; ++attempt) {
        attach = name_attach(NULL, DISPLAY_CHANNEL_NAME, 0);
        if (attach == NULL) {
            std::cerr << "Display: name_attach failed (attempt " << attempt + 1 << "), retrying...\n";
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
    }
    if (attach == NULL) {
        std::cerr << "Display: Failed to attach channel '" << DISPLAY_CHANNEL_NAME << "'\n";
        return;
    }
    std::cout << "Display: Listening for collision warnings on '" << DISPLAY_CHANNEL_NAME << "'\n";

    while (running) {
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

        int reply = 0;
        MsgReply(rcvid, 0, &reply, sizeof(reply));

        if (msg.type == MessageType::COLLISION_DETECTED) {
            size_t numPairs = msg.dataSize / sizeof(std::pair<int, int>);
            std::vector<std::pair<int, int>> pairs(numPairs);
            std::memcpy(pairs.data(), msg.data.data(), msg.dataSize);

            std::lock_guard<std::mutex> lock(collision_mutex);
            collision_warnings.clear();
            for (const auto& p : pairs) {
                std::ostringstream oss;
                oss << "*** COLLISION WARNING: Plane " << p.first
                    << " <-> Plane " << p.second << " ***";
                collision_warnings.push_back(oss.str());
            }
        }
    }

    name_detach(attach, 0);
}

void Display::displayLoop() {
    ATCTimer timer(1, 0);

    while (shared_mem->is_empty.load()) {
        std::cout << "Display: Waiting for aircraft data...\n";
        timer.waitTimer();
    }

    while (running) {
        // Acquire semaphore before reading shared memory
        if (shm_sem != SEM_FAILED) sem_wait(shm_sem);

        bool isEmpty = shared_mem->is_empty.load();
        std::vector<msg_plane_info> planes;
        int count = 0;
        uint64_t timestamp = 0;
        if (!isEmpty) {
            count = shared_mem->count;
            timestamp = shared_mem->timestamp;
            for (int i = 0; i < count; ++i) {
                planes.push_back(shared_mem->plane_data[i]);
            }
        }

        // Release semaphore after reading
        if (shm_sem != SEM_FAILED) sem_post(shm_sem);

        if (isEmpty) {
            std::cout << "Display: No aircraft in airspace. Stopping.\n";
            running = false;
            break;
        }

        render(planes, count, timestamp);
        timer.waitTimer();
    }
}

void Display::render(const std::vector<msg_plane_info>& planes, int count, uint64_t timestamp) {

    std::cout << "\033[2J\033[H";

    // Header
    std::cout << "╔══════════════════════════════════════════════════════════════════════════════════╗\n";
    std::cout << "║                         AIR TRAFFIC CONTROL - DISPLAY                           ║\n";
    std::cout << "╠══════════════════════════════════════════════════════════════════════════════════╣\n";

    std::cout << "║  Timestamp: " << std::setw(15) << std::left << timestamp
              << "    Aircraft in airspace: " << std::setw(3) << count
              << std::setw(20) << "" << "║\n";

    std::cout << "╠══════════════════════════════════════════════════════════════════════════════════╣\n";

    // Column headers
    std::cout << "║ " << std::setw(4) << std::left << "ID"
              << " │ " << std::setw(10) << std::right << "Pos X"
              << " │ " << std::setw(10) << "Pos Y"
              << " │ " << std::setw(10) << "Pos Z"
              << " │ " << std::setw(10) << "Vel X"
              << " │ " << std::setw(10) << "Vel Y"
              << " │ " << std::setw(10) << "Vel Z"
              << " ║\n";

    std::cout << "╠══════════════════════════════════════════════════════════════════════════════════╣\n";

    // Plane data rows
    for (const auto& p : planes) {
        std::cout << "║ " << std::setw(4) << std::left << p.id
                  << " │ " << std::setw(10) << std::right << std::fixed << std::setprecision(1) << p.PositionX
                  << " │ " << std::setw(10) << p.PositionY
                  << " │ " << std::setw(10) << p.PositionZ
                  << " │ " << std::setw(10) << p.VelocityX
                  << " │ " << std::setw(10) << p.VelocityY
                  << " │ " << std::setw(10) << p.VelocityZ
                  << " ║\n";
    }

    if (planes.empty()) {
        std::cout << "║                          (no aircraft data)                                    ║\n";
    }

    std::cout << "╠══════════════════════════════════════════════════════════════════════════════════╣\n";

    // Collision warnings section
    {
        std::lock_guard<std::mutex> lock(collision_mutex);
        if (collision_warnings.empty()) {
            std::cout << "║  Status: ALL CLEAR - No collision warnings                                    ║\n";
        } else {
            std::cout << "║  \033[1;31m!!! COLLISION ALERTS !!!\033[0m                                                    ║\n";
            for (const auto& warning : collision_warnings) {
                std::cout << "║  \033[1;31m" << std::setw(78) << std::left << warning << "\033[0m║\n";
            }
        }
    }

    std::cout << "╚══════════════════════════════════════════════════════════════════════════════════╝\n";
    std::cout << std::flush;
}
