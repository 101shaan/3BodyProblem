#pragma once

#include <SFML/Graphics.hpp>
#include <vector>
#include "Body.h"
#include "PhysicsEngine.h"
#include "Constants.h"

/**
 * @brief Simple 2-D projection renderer for the 3-body simulation.
 *
 * Uses SFML to draw each body as a circle and, optionally, their
 * historical trail. Energy statistics can be displayed as formatted
 * text in the upper-left corner of the window.
 *
 * The renderer purposefully keeps the interface minimal so that the
 * drawing strategy can be replaced without disturbing the rest of the
 * application.
 */
class Renderer {
public:
    Renderer(unsigned int width = Constants::WINDOW_WIDTH,
             unsigned int height = Constants::WINDOW_HEIGHT,
             const char* title = "3-Body Simulation");

    /**
     * @brief Polls SFML events and returns whether the window is still open.
     */
    bool ProcessEvents();

    /**
     * @brief Draw the current simulation frame.
     *
     * @param bodies          Bodies to render.
     * @param energyInfo      Energy information from the physics engine.
     * @param showEnergy      Whether to print the energy statistics.
     * @param showTrails      Whether to draw trails behind bodies.
     */
    void RenderFrame(const std::vector<Body>& bodies,
                     const PhysicsEngine::EnergyInfo& energyInfo,
                     bool showEnergy,
                     bool showTrails);

    /** Access to the internal SFML window (e.g. for focus queries). */
    [[nodiscard]] sf::RenderWindow& GetWindow() noexcept { return window_; }

private:
    sf::RenderWindow window_;
    sf::Font         font_;

    // Utility helpers ------------------------------------------------------

    sf::Vector2f WorldToScreen(const glm::dvec3& pos) const;
};
