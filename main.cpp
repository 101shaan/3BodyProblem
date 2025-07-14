#include "Application.h"
#include <iostream>
#include <stdexcept>

/**
 * @brief Entry point for the 3-body simulation application
 * 
 * This function initializes the application, handles any startup errors,
 * and runs the main simulation loop. All exceptions are caught and
 * logged appropriately.
 * 
 * @return int Exit code (0 for success, 1 for error)
 */
int main() {
    try {
        // Create and configure the application
        Application app;
        
        // Initialize all subsystems (graphics, physics, logging)
        if (!app.Initialize()) {
            std::cerr << "Failed to initialize application" << std::endl;
            return 1;
        }
        
        // Display startup information
        std::cout << "=== 3-Body Gravitational Simulation ===" << std::endl;
        std::cout << "Controls:" << std::endl;
        std::cout << "  SPACE    - Pause/Resume simulation" << std::endl;
        std::cout << "  R        - Reset to initial conditions" << std::endl;
        std::cout << "  +/-      - Increase/Decrease simulation speed" << std::endl;
        std::cout << "  1-5      - Load preset configurations" << std::endl;
        std::cout << "  T        - Toggle trail rendering" << std::endl;
        std::cout << "  E        - Toggle energy display" << std::endl;
        std::cout << "  ESC      - Exit simulation" << std::endl;
        std::cout << "==========================================" << std::endl;
        
        // Run the main application loop
        app.Run();
        
        // Clean shutdown
        app.Shutdown();
        
        std::cout << "Simulation completed successfully." << std::endl;
        std::cout << "Check logs/ directory for detailed simulation data." << std::endl;
        
    } catch (const std::exception& e) {
        std::cerr << "Critical error: " << e.what() << std::endl;
        return 1;
    } catch (...) {
        std::cerr << "Unknown critical error occurred" << std::endl;
        return 1;
    }
    
    return 0;
}