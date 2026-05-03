#ifndef AIRTRAFFICCONTROL_H
#define AIRTRAFFICCONTROL_H

#include "Aircraft.h"
#include <vector>
#include <thread>
#include <string>

// Struct to hold the plane data
struct PlaneData {
    int arrivaTime;
	int id;
    int posX, posY, posZ;
    int speedX, speedY, speedZ;
};

// AirTrafficControl class
class AirTrafficControl {
public:
    AirTrafficControl(); // Constructor
    ~AirTrafficControl(); // Destructor

    // Reads the file and creates aircraft instances
    void readPlanesFromFile(const std::string& fileName);

    // Starts all planes (i.e., creates and joins their threads)
    void startPlanes();

    // Check if all planes have finished their tasks
    bool areAllPlanesFinished() const;

private:
    std::vector<Aircraft*> planes;  // Vector to store all aircraft objects
    std::vector<PlaneData> planeData;  // Vector to store the plane data
    bool allPlanesFinished = false;  // Flag to indicate all plane threads are joined
};

#endif // AIRTRAFFICCONTROL_H
