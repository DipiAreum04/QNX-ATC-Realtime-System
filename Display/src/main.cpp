#include "Display.h"

int main() {
    Display display;

    if (display.start()) {
        display.join();
    } else {
        std::cerr << "Display: Failed to start." << std::endl;
        return 1;
    }

    std::cout << "Display: Exiting." << std::endl;
    return 0;
}
