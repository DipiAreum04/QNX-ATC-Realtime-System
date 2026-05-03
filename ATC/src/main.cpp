#include "AirTrafficControl.h"
#include "Radar.h"
#include "ATCTimer.h"

// Global tick counter
uint64_t tick_counter = 0; // Counter for time ticks
std::atomic<bool> running(true);  // Flag to control the timer thread's execution status

// Function to increment the tick_counter every second
void timer_tick() {
    while (running) {
        std::this_thread::sleep_for(std::chrono::seconds(1));  // Sleep for 1 second to simulate time passing
        tick_counter++;  // Increment the tick counter
        std::cout << "Tick counter: " << tick_counter << std::endl; // print for debugging purposes
    }
}

int main() {

    // Create the AirTrafficControl instance
    AirTrafficControl atc;

    atc.readPlanesFromFile("planes.txt");  // Ensure the file is in the correct directory

    Radar radar(tick_counter); // Create the Radar instance

    // Start a timer thread to increment tick_counter every second
    std::thread timer_thread(timer_tick);

    atc.startPlanes();

    // Check if all planes have finished their tasks
    if (atc.areAllPlanesFinished()) {
    	std::cout << "Main function received signal that all aircraft are inactive.\n";
    	running = false;  // Stop the timer thread
    	timer_thread.join();  // Wait for the timer thread to finish
    }
    return 0;
}
