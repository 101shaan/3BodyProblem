#include "Application.h"
#include "Constants.h"

#include <SFML/System.hpp>
#include <format>
#include <filesystem>

namespace fs = std::filesystem;

// -----------------------------------------------------------------------------
// ctor / dtor
// -----------------------------------------------------------------------------

Application::Application()
    : posLogger_{Constants::POSITION_FILE},
      energyLogger_{Constants::ENERGY_FILE}
{
}

Application::~Application() = default;

// -----------------------------------------------------------------------------
// initialization
// -----------------------------------------------------------------------------

bool Application::Initialize() {
    // Ensure folders exist.
    fs::create_directories("logs");
    fs::create_directories("config");

    // Load initial bodies --------------------------------------------------
    {
        ConfigManager cfg{Constants::CONFIG_FILE};
        bodies_ = cfg.LoadInitialBodies();
        if (bodies_.empty()) {
            // Fallback to simple preset.
            ResetSimulation(/*presetIndex*/0);
        }
    }

    // Init physics ---------------------------------------------------------
    physics_.Initialize(bodies_);

    return true;
}

// -----------------------------------------------------------------------------
// main loop
// -----------------------------------------------------------------------------

void Application::Run() {
    sf::Clock clock;
    double accumulator = 0.0;
    const double fixedDt = physics_.GetTimeStep();

    while (renderer_.ProcessEvents()) {
        // Real time since last frame.
        const double deltaReal = clock.restart().asSeconds();
        if (!paused_) {
            accumulator += deltaReal * speedFactor_;
            while (accumulator >= fixedDt) {
                HandleInput(); // allows realtime response even if physics steps multiple
                Update(fixedDt);
                accumulator -= fixedDt;
            }
        } else {
            HandleInput();
        }

        auto energy = physics_.CalculateEnergy(bodies_);
        renderer_.RenderFrame(bodies_, energy, showEnergy_, showTrails_);
    }
}

void Application::Shutdown() {
    // Nothing special for now.
}

// -----------------------------------------------------------------------------
// helpers
// -----------------------------------------------------------------------------

void Application::HandleInput() {
    auto& window = renderer_.GetWindow();

    if (!window.hasFocus()) return;

    if (sf::Keyboard::isKeyPressed(sf::Keyboard::Space)) {
        paused_ = !paused_;
    }
    if (sf::Keyboard::isKeyPressed(sf::Keyboard::Add) || sf::Keyboard::isKeyPressed(sf::Keyboard::Equal)) {
        speedFactor_ = std::min(speedFactor_ * Constants::SPEED_MULTIPLIER, Constants::MAX_SPEED);
    }
    if (sf::Keyboard::isKeyPressed(sf::Keyboard::Hyphen)) {
        speedFactor_ = std::max(speedFactor_ / Constants::SPEED_MULTIPLIER, Constants::MIN_SPEED);
    }
    if (sf::Keyboard::isKeyPressed(sf::Keyboard::R)) {
        ResetSimulation();
    }
    if (sf::Keyboard::isKeyPressed(sf::Keyboard::T)) {
        showTrails_ = !showTrails_;
    }
    if (sf::Keyboard::isKeyPressed(sf::Keyboard::E)) {
        showEnergy_ = !showEnergy_;
    }
    if (sf::Keyboard::isKeyPressed(sf::Keyboard::Num1)) ResetSimulation(1);
    if (sf::Keyboard::isKeyPressed(sf::Keyboard::Num2)) ResetSimulation(2);
    if (sf::Keyboard::isKeyPressed(sf::Keyboard::Num3)) ResetSimulation(3);
}

void Application::Update(double dt) {
    // Physics step
    physics_.StepRK4(bodies_);

    // Update trails
    for (auto& b : bodies_) b.UpdateTrail();

    // Logging --------------------------------------------------------------
    static uint64_t step = 0;
    if (step % Constants::LOG_FREQUENCY == 0) {
        // Positions
        for (const auto& b : bodies_) {
            posLogger_.WriteLine("{},{},{},{}", physics_.GetSimulationTime(), b.GetPosition().x, b.GetPosition().y, b.GetPosition().z);
        }
        // Energy
        auto energy = physics_.CalculateEnergy(bodies_);
        energyLogger_.WriteLine("{},{},{}", physics_.GetSimulationTime(), energy.total_energy, energy.energy_drift);
    }
    ++step;
}

void Application::ResetSimulation(int presetIndex) {
    bodies_.clear();

    if (presetIndex == 1) {
        // Figure-8 ---------------------------------------------------------
        bodies_.emplace_back(Constants::Presets::FIGURE8_MASS, Constants::Presets::FIGURE8_POS_1, Constants::Presets::FIGURE8_VEL_1, 5.0, Constants::BODY_COLOR_1);
        bodies_.emplace_back(Constants::Presets::FIGURE8_MASS, Constants::Presets::FIGURE8_POS_2, Constants::Presets::FIGURE8_VEL_2, 5.0, Constants::BODY_COLOR_2);
        bodies_.emplace_back(Constants::Presets::FIGURE8_MASS, Constants::Presets::FIGURE8_POS_3, Constants::Presets::FIGURE8_VEL_3, 5.0, Constants::BODY_COLOR_3);
    } else if (presetIndex == 2) {
        // Triangular orbit -----------------------------------------------
        double r = Constants::Presets::TRIANGLE_RADIUS;
        double v = Constants::Presets::TRIANGLE_VELOCITY;
        bodies_.emplace_back(1.0, glm::dvec3{-r, 0.0, 0.0}, glm::dvec3{0.0, v, 0.0}, 5.0, Constants::BODY_COLOR_1);
        bodies_.emplace_back(1.0, glm::dvec3{ r, 0.0, 0.0}, glm::dvec3{0.0, -v, 0.0}, 5.0, Constants::BODY_COLOR_2);
        bodies_.emplace_back(1.0, glm::dvec3{0.0, r * std::numbers::sqrt3, 0.0}, glm::dvec3{-v, 0.0, 0.0}, 5.0, Constants::BODY_COLOR_3);
    } else if (presetIndex == 3) {
        // Chaotic ---------------------------------------------------------
        bodies_.emplace_back(Constants::Presets::CHAOS_MASS_1, glm::dvec3{-10, 0, 0}, glm::dvec3{0, 1.6, 0}, 5.0, Constants::BODY_COLOR_1);
        bodies_.emplace_back(Constants::Presets::CHAOS_MASS_2, glm::dvec3{ 10, 0, 0}, glm::dvec3{0, -1.6, 0}, 5.0, Constants::BODY_COLOR_2);
        bodies_.emplace_back(Constants::Presets::CHAOS_MASS_3, glm::dvec3{0, 15, 0}, glm::dvec3{-1.2, 0, 0}, 5.0, Constants::BODY_COLOR_3);
    } else {
        // Simple default --------------------------------------------------
        bodies_.emplace_back(1.0, glm::dvec3{-5, 0, 0}, glm::dvec3{0, 0.8, 0}, 5.0, Constants::BODY_COLOR_1);
        bodies_.emplace_back(1.0, glm::dvec3{ 5, 0, 0}, glm::dvec3{0, -0.8, 0}, 5.0, Constants::BODY_COLOR_2);
        bodies_.emplace_back(1.0, glm::dvec3{0, 8, 0}, glm::dvec3{-0.8, 0, 0}, 5.0, Constants::BODY_COLOR_3);
    }

    physics_.Initialize(bodies_);
}
