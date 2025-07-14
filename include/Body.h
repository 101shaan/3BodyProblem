#pragma once

#include <glm/glm.hpp>
#include <vector>
#include <memory>
#include "Constants.h"

/**
 * @brief Represents a celestial body in the 3-body simulation
 * 
 * This class encapsulates all properties and behaviors of a gravitational body,
 * including mass, position, velocity, and rendering trail data. The class
 * supports physics calculations, collision detection, and trail management.
 */
class Body {
public:
    /**
     * @brief Default constructor creates a unit mass body at origin
     */
    Body();
    
    /**
     * @brief Construct a body with specified properties
     * @param mass Mass of the body (kg)
     * @param position Initial position (m)
     * @param velocity Initial velocity (m/s)
     * @param radius Physical radius for collision detection (m)
     * @param color RGB color for rendering
     */
    Body(double mass, 
         const glm::dvec3& position, 
         const glm::dvec3& velocity, 
         double radius,
         const glm::vec3& color = glm::vec3(1.0f, 1.0f, 1.0f));
    
    /**
     * @brief Copy constructor
     */
    Body(const Body& other);
    
    /**
     * @brief Move constructor
     */
    Body(Body&& other) noexcept;
    
    /**
     * @brief Copy assignment operator
     */
    Body& operator=(const Body& other);
    
    /**
     * @brief Move assignment operator
     */
    Body& operator=(Body&& other) noexcept;
    
    /**
     * @brief Destructor
     */
    ~Body() = default;
    
    // Getters
    [[nodiscard]] double GetMass() const noexcept { return mass_; }
    [[nodiscard]] const glm::dvec3& GetPosition() const noexcept { return position_; }
    [[nodiscard]] const glm::dvec3& GetVelocity() const noexcept { return velocity_; }
    [[nodiscard]] double GetRadius() const noexcept { return radius_; }
    [[nodiscard]] const glm::vec3& GetColor() const noexcept { return color_; }
    [[nodiscard]] bool IsActive() const noexcept { return active_; }
    
    // Setters
    void SetMass(double mass) noexcept { mass_ = mass; }
    void SetPosition(const glm::dvec3& position) noexcept { position_ = position; }
    void SetVelocity(const glm::dvec3& velocity) noexcept { velocity_ = velocity; }
    void SetRadius(double radius) noexcept { radius_ = radius; }
    void SetColor(const glm::vec3& color) noexcept { color_ = color; }
    void SetActive(bool active) noexcept { active_ = active; }
    
    /**
     * @brief Calculate kinetic energy of the body
     * @return Kinetic energy (J)
     */
    [[nodiscard]] double GetKineticEnergy() const noexcept;
    
    /**
     * @brief Calculate momentum of the body
     * @return Momentum vector (kg⋅m/s)
     */
    [[nodiscard]] glm::dvec3 GetMomentum() const noexcept;
    
    /**
     * @brief Calculate distance to another body
     * @param other The other body
     * @return Distance (m)
     */
    [[nodiscard]] double DistanceTo(const Body& other) const noexcept;
    
    /**
     * @brief Calculate squared distance to another body (faster than DistanceTo)
     * @param other The other body
     * @return Squared distance (m²)
     */
    [[nodiscard]] double SquaredDistanceTo(const Body& other) const noexcept;
    
    /**
     * @brief Check if this body is colliding with another
     * @param other The other body
     * @return True if bodies are colliding
     */
    [[nodiscard]] bool IsCollidingWith(const Body& other) const noexcept;
    
    /**
     * @brief Merge this body with another (conservation of momentum)
     * @param other The body to merge with
     * @return New merged body
     */
    [[nodiscard]] Body MergeWith(const Body& other) const;
    
    /**
     * @brief Apply gravitational force from another body
     * @param other The body exerting gravitational force
     * @param dt Time step (s)
     */
    void ApplyGravitationalForce(const Body& other, double dt);
    
    /**
     * @brief Update position and velocity using current acceleration
     * @param dt Time step (s)
     */
    void UpdateMotion(double dt);
    
    /**
     * @brief Add current position to trail
     */
    void UpdateTrail();
    
    /**
     * @brief Clear the position trail
     */
    void ClearTrail();
    
    /**
     * @brief Get the position trail for rendering
     * @return Vector of trail positions
     */
    [[nodiscard]] const std::vector<glm::dvec3>& GetTrail() const noexcept { return trail_; }
    
    /**
     * @brief Reset body to initial state
     * @param initialPosition Initial position
     * @param initialVelocity Initial velocity
     */
    void Reset(const glm::dvec3& initialPosition, const glm::dvec3& initialVelocity);
    
    /**
     * @brief Get string representation of body state
     * @return Formatted string with position, velocity, and mass
     */
    [[nodiscard]] std::string ToString() const;

private:
    // Physical properties
    double mass_;                    ///< Mass of the body (kg)
    glm::dvec3 position_;           ///< Current position (m)
    glm::dvec3 velocity_;           ///< Current velocity (m/s)
    glm::dvec3 acceleration_;       ///< Current acceleration (m/s²)
    double radius_;                 ///< Physical radius for collision detection (m)
    bool active_;                   ///< Whether this body is active in simulation
    
    // Rendering properties
    glm::vec3 color_;               ///< RGB color for rendering
    std::vector<glm::dvec3> trail_; ///< Position trail for rendering
    
    // Initial state for reset functionality
    glm::dvec3 initial_position_;   ///< Initial position for reset
    glm::dvec3 initial_velocity_;   ///< Initial velocity for reset
    
    /**
     * @brief Calculate gravitational acceleration from another body
     * @param other The body exerting gravitational force
     * @return Acceleration vector (m/s²)
     */
    [[nodiscard]] glm::dvec3 CalculateGravitationalAcceleration(const Body& other) const;
    
    /**
     * @brief Ensure trail doesn't exceed maximum size
     */
    void TrimTrail();
};

// Utility functions for body operations
namespace BodyUtils {
    /**
     * @brief Calculate center of mass for a collection of bodies
     * @param bodies Vector of bodies
     * @return Center of mass position
     */
    [[nodiscard]] glm::dvec3 CalculateCenterOfMass(const std::vector<Body>& bodies);
    
    /**
     * @brief Calculate total momentum of a collection of bodies
     * @param bodies Vector of bodies
     * @return Total momentum vector
     */
    [[nodiscard]] glm::dvec3 CalculateTotalMomentum(const std::vector<Body>& bodies);
    
    /**
     * @brief Calculate total mass of a collection of bodies
     * @param bodies Vector of bodies
     * @return Total mass
     */
    [[nodiscard]] double CalculateTotalMass(const std::vector<Body>& bodies);
}