#pragma once

#include <glm/glm.hpp>
#include <cstdint>

/**
 * @brief Global constants for the 3-body simulation
 * 
 * This file contains all physical constants, simulation parameters,
 * and configuration values used throughout the application.
 */
namespace Constants {
    // Physical constants
    constexpr double G = 6.67430e-11;           // Gravitational constant (m³/kg·s²)
    constexpr double EARTH_MASS = 5.972e24;     // Earth mass (kg)
    constexpr double EARTH_RADIUS = 6.371e6;    // Earth radius (m)
    constexpr double AU = 1.496e11;             // Astronomical unit (m)
    
    // Simulation parameters
    constexpr double DEFAULT_TIME_STEP = 0.01;  // Default RK4 time step (s)
    constexpr double MIN_TIME_STEP = 1e-6;      // Minimum allowed time step
    constexpr double MAX_TIME_STEP = 1.0;       // Maximum allowed time step
    constexpr double COLLISION_FACTOR = 2.0;    // Collision detection multiplier
    constexpr double ENERGY_TOLERANCE = 1e-6;   // Energy conservation tolerance
    
    // Graphics constants
    constexpr int WINDOW_WIDTH = 1200;          // Default window width
    constexpr int WINDOW_HEIGHT = 800;          // Default window height
    constexpr float FOV = 45.0f;                // Field of view (degrees)
    constexpr float NEAR_PLANE = 0.1f;          // Near clipping plane
    constexpr float FAR_PLANE = 1000.0f;        // Far clipping plane
    
    // Rendering parameters
    constexpr int MAX_TRAIL_POINTS = 2000;      // Maximum trail points per body
    constexpr float TRAIL_FADE_RATE = 0.995f;   // Trail alpha decay rate
    constexpr float MIN_BODY_SIZE = 2.0f;       // Minimum rendered body size
    constexpr float MAX_BODY_SIZE = 50.0f;      // Maximum rendered body size
    
    // Colors (RGB)
    constexpr glm::vec3 BODY_COLOR_1{1.0f, 0.3f, 0.3f};  // Red
    constexpr glm::vec3 BODY_COLOR_2{0.3f, 1.0f, 0.3f};  // Green
    constexpr glm::vec3 BODY_COLOR_3{0.3f, 0.3f, 1.0f};  // Blue
    constexpr glm::vec3 BACKGROUND_COLOR{0.02f, 0.02f, 0.05f};  // Dark blue
    constexpr glm::vec3 TRAIL_COLOR{0.7f, 0.7f, 0.7f};   // Light gray
    constexpr glm::vec3 TEXT_COLOR{1.0f, 1.0f, 1.0f};    // White
    
    // Physics integration
    constexpr double SOFTENING_PARAMETER = 1e-10;  // Gravitational softening
    constexpr int RK4_SUBSTEPS = 4;                 // Runge-Kutta substeps
    
    // Logging and output
    constexpr int LOG_FREQUENCY = 100;              // Log every N simulation steps
    constexpr int ENERGY_CHECK_FREQUENCY = 10;     // Check energy every N steps
    constexpr size_t MAX_LOG_ENTRIES = 1000000;    // Maximum log entries
    
    // Input parameters
    constexpr double SPEED_MULTIPLIER = 1.5;       // Speed change factor
    constexpr double MIN_SPEED = 0.01;             // Minimum simulation speed
    constexpr double MAX_SPEED = 100.0;            // Maximum simulation speed
    
    // File paths
    constexpr const char* CONFIG_FILE = "config/initial_conditions.json";
    constexpr const char* LOG_FILE = "logs/simulation.log";
    constexpr const char* ENERGY_FILE = "logs/energy_log.csv";
    constexpr const char* POSITION_FILE = "logs/positions.csv";
    
    // Preset configurations
    namespace Presets {
        // Figure-8 orbit (Chenciner-Montgomery solution)
        constexpr double FIGURE8_MASS = 1.0;
        constexpr glm::dvec3 FIGURE8_POS_1{-0.97000436, 0.24308753, 0.0};
        constexpr glm::dvec3 FIGURE8_POS_2{-FIGURE8_POS_1.x, -FIGURE8_POS_1.y, 0.0};
        constexpr glm::dvec3 FIGURE8_POS_3{0.0, 0.0, 0.0};
        constexpr glm::dvec3 FIGURE8_VEL_1{0.466203685, 0.43236573, 0.0};
        constexpr glm::dvec3 FIGURE8_VEL_2{FIGURE8_VEL_1.x, FIGURE8_VEL_1.y, 0.0};
        constexpr glm::dvec3 FIGURE8_VEL_3{-2.0 * FIGURE8_VEL_1.x, -2.0 * FIGURE8_VEL_1.y, 0.0};
        
        // Stable triangular orbit (Lagrange L4/L5 inspired)
        constexpr double TRIANGLE_MASS = 1.0;
        constexpr double TRIANGLE_RADIUS = 2.0;
        constexpr double TRIANGLE_VELOCITY = 0.5;
        
        // Chaotic system
        constexpr double CHAOS_MASS_1 = 1.0;
        constexpr double CHAOS_MASS_2 = 1.5;
        constexpr double CHAOS_MASS_3 = 0.8;
    }
}