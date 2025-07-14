#pragma once

#include <vector>
#include <memory>
#include "Body.h"
#include "PhysicsEngine.h"
#include "Renderer.h"
#include "Logger.h"
#include "ConfigManager.h"

/**
 * @brief High-level orchestrator tying together physics, rendering, input and
 * logging to deliver a complete interactive simulation.
 */
class Application {
public:
    Application();
    ~Application();

    bool Initialize();
    void Run();
    void Shutdown();

private:
    // Simulation -----------------------------------------------------------
    std::vector<Body> bodies_;
    PhysicsEngine     physics_;

    // UI / rendering -------------------------------------------------------
    Renderer          renderer_;

    // Logging --------------------------------------------------------------
    Logger            posLogger_;
    Logger            energyLogger_;

    // Config / state -------------------------------------------------------
    bool   paused_        = false;
    double speedFactor_   = 1.0;   // Multiplier for dt.
    bool   showEnergy_    = true;
    bool   showTrails_    = true;

    // Internal helpers -----------------------------------------------------
    void HandleInput();
    void Update(double dt);
    void ResetSimulation(int presetIndex = 0);

    // Time tracking --------------------------------------------------------
    double realTimeAccumulator_ = 0.0;
};
