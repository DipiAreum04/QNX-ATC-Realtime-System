#include "AirTrafficControl.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <thread>

// Constructor
AirTrafficControl::AirTrafficControl() {
    // Constructor remains empty since no initialization is needed
}

// Destructor
AirTrafficControl::~AirTrafficControl() {
}

// Read planes from file and create aircraft instances
void AirTrafficControl::readPlanesFromFile(const std::string& fileName) {
    std::ifstream file(fileName); // Open the file
    if (!file.is_open()) { // Check if the file is open
        std::cerr << "Error opening file: " << fileName << std::endl;
        return;
    }

    std::string line;
    while (std::getline(file, line)) {
        std::stringstream ss(line); // Create a stringstream from the line
        PlaneData data;

        ss >> data.arrivaTime >> data.id >> data.posX >> data.posY >> data.posZ
           >> data.speedX >> data.speedY >> data.speedZ; // Extract the data from the stringstream

        if (ss.fail()) {
            std::cerr << "Error parsing line: " << line << std::endl;
            continue;  // Skip invalid lines and continue with the next line
        }

        // Store the plane data in the planeData vector
        planeData.push_back(data);
    }

    file.close(); // Close the file
}

// Start all planes (i.e., create and join their threads)
void AirTrafficControl::startPlanes() {
    // For each plane data, create an Aircraft instance and start its thread
    for (const auto& data : planeData) {
        // Print the values when creating the Aircraft instance for debugging purposes
        std::cout << "Creating Aircraft " << data.id << ": "
                  << "Pos(" << data.posX << ", " << data.posY << ", " << data.posZ << ") "
                  << "Speed(" << data.speedX << ", " << data.speedY << ", " << data.speedZ << ") "
                  << "ArrivalTime(" << data.arrivaTime << ")\n";

        // Dynamically allocate Aircraft instance and store the pointer in planes vector
        Aircraft* plane = new Aircraft(data.id, data.posX, data.posY, data.posZ,
                                       data.speedX, data.speedY, data.speedZ, data.arrivaTime);
        planes.push_back(plane);  // Store the pointer in the vector
    }

    // Join all aircraft threads
    for (Aircraft* Plane : planes) {
        if (pthread_join(Plane->thread_id, nullptr) != 0) { // join with each plane's thread
            std::cerr << "Error: pthread_join failed for Aircraft " << Plane->getID() << std::endl;
            exit(1);
        }
    }
    allPlanesFinished = true;  // Set the flag after all threads are joined
    std::cout << "All aircraft have finished their tasks and are no longer active.\n";
}

// Check if all planes have finished their tasks
bool AirTrafficControl::areAllPlanesFinished() const {
    return allPlanesFinished;
}
