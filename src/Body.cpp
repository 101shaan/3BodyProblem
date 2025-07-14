#include "Body.h"
#include <cmath>
#include <sstream>
#include <iomanip>
#include <algorithm>

Body::Body() 
    : mass_(1.0)
    , position_(0.0, 0.0, 0.0)
    , velocity_(0.0, 0.0, 0.0)
    , acceleration_(0.0, 0.0, 0.0)
    , radius_(1.0)
    , active_(true)
    , color_(1.0f, 1.0f, 1.0f)
    , initial_position_(0.0, 0.0, 0.0)
    , initial_velocity_(0.0, 0.0, 0.0) {
    trail_.reserve(Constants::MAX_TRAIL_POINTS);
}

Body::Body(double mass, 
           const glm::dvec3& position, 
           const glm::dvec3& velocity, 
           double radius,
           const glm::vec3& color)
    : mass_(mass)
    , position_(position)
    , velocity_(velocity)
    , acceleration_(0.0, 0.0, 0.0)
    , radius_(radius)
    , active_(true)
    , color_(color)
    , initial_position_(position)
    , initial_velocity_(velocity) {
    trail_.reserve(Constants::MAX_TRAIL_POINTS);
}

Body::Body(const Body& other)
    : mass_(other.mass_)
    , position_(other.position_)
    , velocity_(other.velocity_)
    , acceleration_(other.acceleration_)
    , radius_(other.radius_)
    , active_(other.active_)
    , color_(other.color_)
    , trail_(other.trail_)
    , initial_position_(other.initial_position_)
    , initial_velocity_(other.initial_velocity_) {
}

Body::Body(Body&& other) noexcept
    : mass_(other.mass_)
    , position_(std::move(other.position_))
    , velocity_(std::move(other.velocity_))
    , acceleration_(std::move(other.acceleration_))
    , radius_(other.radius_)
    , active_(other.active_)
    , color_(std::move(other.color_))
    , trail_(std::move(other.trail_))
    , initial_position_(std::move(other.initial_position_))
    , initial_velocity_(std::move(other.initial_velocity_)) {
}

Body& Body::operator=(const Body& other) {
    if (this != &other) {
        mass_ = other.mass_;
        position_ = other.position_;
        velocity_ = other.velocity_;
        acceleration_ = other.acceleration_;
        radius_ = other.radius_;
        active_ = other.active_;
        color_ = other.color_;
        trail_ = other.trail_;
        initial_position_ = other.initial_position_;
        initial_velocity_ = other.initial_velocity_;
    }
    return *this;
}

Body& Body::operator=(Body&& other) noexcept {
    if (this != &other) {
        mass_ = other.mass_;
        position_ = std::move(other.position_);
        velocity_ = std::move(other.velocity_);
        acceleration_ = std::move(other.acceleration_);
        radius_ = other.radius_;
        active_ = other.active_;
        color_ = std::move(other.color_);
        trail_ = std::move(other.trail_);
        initial_position_ = std::move(other.initial_position_);
        initial_velocity_ = std::move(other.initial_velocity_);
    }
    return *this;
}

double Body::GetKineticEnergy() const noexcept {
    // KE = (1/2) * m * v²
    const double speed_squared = glm::dot(velocity_, velocity_);
    return 0.5 * mass_ * speed_squared;
}

glm::dvec3 Body::GetMomentum() const noexcept {
    // p = m * v
    return mass_ * velocity_;
}

double Body::DistanceTo(const Body& other) const noexcept {
    const glm::dvec3 displacement = position_ - other.position_;
    return glm::length(displacement);
}

double Body::SquaredDistanceTo(const Body& other) const noexcept {
    const glm::dvec3 displacement = position_ - other.position_;
    return glm::dot(displacement, displacement);
}

bool Body::IsCollidingWith(const Body& other) const noexcept {
    if (!active_ || !other.active_) {
        return false;
    }
    
    const double collision_distance = Constants::COLLISION_FACTOR * (radius_ + other.radius_);
    return SquaredDistanceTo(other) <= (collision_distance * collision_distance);
}

Body Body::MergeWith(const Body& other) const {
    // Conservation of momentum: p_total = p1 + p2
    const glm::dvec3 total_momentum = GetMomentum() + other.GetMomentum();
    const double total_mass = mass_ + other.mass_;
    const glm::dvec3 new_velocity = total_momentum / total_mass;
    
    // Center of mass position: r_cm = (m1*r1 + m2*r2) / (m1 + m2)
    const glm::dvec3 new_position = (mass_ * position_ + other.mass_ * other.position_) / total_mass;
    
    // New radius assuming spherical bodies and constant density
    const double volume1 = (4.0 / 3.0) * M_PI * std::pow(radius_, 3);
    const double volume2 = (4.0 / 3.0) * M_PI * std::pow(other.radius_, 3);
    const double new_radius = std::cbrt((volume1 + volume2) * 3.0 / (4.0 * M_PI));
    
    // Average color weighted by mass
    const glm::vec3 new_color = (static_cast<float>(mass_) * color_ + 
                                static_cast<float>(other.mass_) * other.color_) / 
                               static_cast<float>(total_mass);
    
    return Body(total_mass, new_position, new_velocity, new_radius, new_color);
}

glm::dvec3 Body::CalculateGravitationalAcceleration(const Body& other) const {
    if (!active_ || !other.active_) {
        return glm::dvec3(0.0);
    }
    
    const glm::dvec3 displacement = other.position_ - position_;
    const double distance_squared = glm::dot(displacement, displacement);
    
    // Add softening parameter to prevent singularities
    const double softened_distance_squared = distance_squared + Constants::SOFTENING_PARAMETER;
    const double distance = std::sqrt(softened_distance_squared);
    
    // Newton's law of gravitation: F = G * m1 * m2 / r²
    // Acceleration: a = F / m = G * m_other / r²
    const double acceleration_magnitude = Constants::G * other.mass_ / softened_distance_squared;
    
    // Direction: unit vector from this body to other body
    const glm::dvec3 direction = displacement / distance;
    
    return acceleration_magnitude * direction;
}

void Body::ApplyGravitationalForce(const Body& other, double dt) {
    if (!active_ || !other.active_) {
        return;
    }
    
    const glm::dvec3 gravitational_acceleration = CalculateGravitationalAcceleration(other);
    acceleration_ += gravitational_acceleration;
}

void Body::UpdateMotion(double dt) {
    if (!active_) {
        return;
    }
    
    // Verlet integration for better stability
    // v(t + dt) = v(t) + a(t) * dt
    // r(t + dt) = r(t) + v(t) * dt + 0.5 * a(t) * dt²
    
    velocity_ += acceleration_ * dt;
    position_ += velocity_ * dt + 0.5 * acceleration_ * dt * dt;
    
    // Reset acceleration for next iteration
    acceleration_ = glm::dvec3(0.0);
}

void Body::UpdateTrail() {
    if (!active_) {
        return;
    }
    
    // Add current position to trail
    trail_.push_back(position_);
    
    // Trim trail if it exceeds maximum size
    TrimTrail();
}

void Body::TrimTrail() {
    if (trail_.size() > Constants::MAX_TRAIL_POINTS) {
        // Remove oldest points to maintain fixed size
        const size_t excess = trail_.size() - Constants::MAX_TRAIL_POINTS;
        trail_.erase(trail_.begin(), trail_.begin() + excess);
    }
}

void Body::ClearTrail() {
    trail_.clear();
}

void Body::Reset(const glm::dvec3& initialPosition, const glm::dvec3& initialVelocity) {
    initial_position_ = initialPosition;
    initial_velocity_ = initialVelocity;
    position_ = initialPosition;
    velocity_ = initialVelocity;
    acceleration_ = glm::dvec3(0.0);
    active_ = true;
    ClearTrail();
}

std::string Body::ToString() const {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(3);
    oss << "Body[m=" << mass_ << "kg, ";
    oss << "pos=(" << position_.x << "," << position_.y << "," << position_.z << "), ";
    oss << "vel=(" << velocity_.x << "," << velocity_.y << "," << velocity_.z << "), ";
    oss << "r=" << radius_ << "m, active=" << (active_ ? "true" : "false") << "]";
    return oss.str();
}

// Utility functions implementation
namespace BodyUtils {
    glm::dvec3 CalculateCenterOfMass(const std::vector<Body>& bodies) {
        glm::dvec3 center_of_mass(0.0);
        double total_mass = 0.0;
        
        for (const auto& body : bodies) {
            if (body.IsActive()) {
                center_of_mass += body.GetMass() * body.GetPosition();
                total_mass += body.GetMass();
            }
        }
        
        if (total_mass > 0.0) {
            center_of_mass /= total_mass;
        }
        
        return center_of_mass;
    }
    
    glm::dvec3 CalculateTotalMomentum(const std::vector<Body>& bodies) {
        glm::dvec3 total_momentum(0.0);
        
        for (const auto& body : bodies) {
            if (body.IsActive()) {
                total_momentum += body.GetMomentum();
            }
        }
        
        return total_momentum;
    }
    
    double CalculateTotalMass(const std::vector<Body>& bodies) {
        double total_mass = 0.0;
        
        for (const auto& body : bodies) {
            if (body.IsActive()) {
                total_mass += body.GetMass();
            }
        }
        
        return total_mass;
    }
}