#include "PhysicsEngine.h"
#include "Constants.h"
#include <cmath>
#include <algorithm>
#include <numeric>
#include <chrono>

// SystemState implementation
PhysicsEngine::SystemState::SystemState(size_t num_bodies) 
    : positions(num_bodies), velocities(num_bodies) {}

PhysicsEngine::SystemState PhysicsEngine::SystemState::operator+(const SystemState& other) const {
    SystemState result(positions.size());
    for (size_t i = 0; i < positions.size(); ++i) {
        result.positions[i] = positions[i] + other.positions[i];
        result.velocities[i] = velocities[i] + other.velocities[i];
    }
    return result;
}

PhysicsEngine::SystemState PhysicsEngine::SystemState::operator*(double scalar) const {
    SystemState result(positions.size());
    for (size_t i = 0; i < positions.size(); ++i) {
        result.positions[i] = positions[i] * scalar;
        result.velocities[i] = velocities[i] * scalar;
    }
    return result;
}

PhysicsEngine::SystemState& PhysicsEngine::SystemState::operator+=(const SystemState& other) {
    for (size_t i = 0; i < positions.size(); ++i) {
        positions[i] += other.positions[i];
        velocities[i] += other.velocities[i];
    }
    return *this;
}

PhysicsEngine::SystemState& PhysicsEngine::SystemState::operator*=(double scalar) {
    for (size_t i = 0; i < positions.size(); ++i) {
        positions[i] *= scalar;
        velocities[i] *= scalar;
    }
    return *this;
}

// PhysicsEngine implementation
PhysicsEngine::PhysicsEngine() 
    : time_step_(Constants::DEFAULT_TIME_STEP)
    , simulation_time_(0.0)
    , step_count_(0)
    , initial_energy_(0.0)
    , energy_initialized_(false)
    , adaptive_time_step_(false)
    , tolerance_(1e-6)
    , min_time_step_(Constants::MIN_TIME_STEP)
    , max_time_step_(Constants::MAX_TIME_STEP)
    , stats_{}
{
    step_times_.reserve(1000);
}

PhysicsEngine::PhysicsEngine(double time_step)
    : PhysicsEngine()
{
    SetTimeStep(time_step);
}

void PhysicsEngine::Initialize(const std::vector<Body>& bodies) {
    Reset();
    
    // Calculate initial energy for drift tracking
    EnergyInfo energy = CalculateEnergy(bodies);
    initial_energy_ = energy.total_energy;
    energy_initialized_ = true;
    
    // Initialize previous accelerations for Verlet integration
    prev_accelerations_.resize(bodies.size());
    std::vector<glm::dvec3> positions;
    positions.reserve(bodies.size());
    for (const auto& body : bodies) {
        positions.push_back(body.GetPosition());
    }
    
    auto accelerations = CalculateAccelerations(positions, bodies);
    std::copy(accelerations.begin(), accelerations.end(), prev_accelerations_.begin());
}

void PhysicsEngine::StepRK4(std::vector<Body>& bodies) {
    auto start_time = std::chrono::high_resolution_clock::now();
    
    // 4th-order Runge-Kutta integration
    // dy/dt = f(t, y)
    // k1 = h * f(t, y)
    // k2 = h * f(t + h/2, y + k1/2)
    // k3 = h * f(t + h/2, y + k2/2)
    // k4 = h * f(t + h, y + k3)
    // y_new = y + (k1 + 2*k2 + 2*k3 + k4) / 6
    
    const SystemState initial_state = BodiesToState(bodies);
    
    // Calculate k1
    const SystemState k1 = CalculateDerivatives(initial_state, bodies) * time_step_;
    
    // Calculate k2
    SystemState temp_state = initial_state + k1 * 0.5;
    StateToBodies(temp_state, bodies);
    const SystemState k2 = CalculateDerivatives(temp_state, bodies) * time_step_;
    
    // Calculate k3
    temp_state = initial_state + k2 * 0.5;
    StateToBodies(temp_state, bodies);
    const SystemState k3 = CalculateDerivatives(temp_state, bodies) * time_step_;
    
    // Calculate k4
    temp_state = initial_state + k3;
    StateToBodies(temp_state, bodies);
    const SystemState k4 = CalculateDerivatives(temp_state, bodies) * time_step_;
    
    // Final RK4 update
    const SystemState final_state = initial_state + (k1 + k2 * 2.0 + k3 * 2.0 + k4) * (1.0 / 6.0);
    StateToBodies(final_state, bodies);
    
    // Update trails
    for (auto& body : bodies) {
        body.UpdateTrail();
    }
    
    // Handle collisions
    bool collision_occurred = HandleCollisions(bodies);
    
    // Update simulation time and step count
    simulation_time_ += time_step_;
    step_count_++;
    
    // Calculate energy drift for performance tracking
    double energy_drift = 0.0;
    if (energy_initialized_) {
        EnergyInfo energy = CalculateEnergy(bodies);
        energy_drift = std::abs(energy.energy_drift);
    }
    
    // Update performance statistics
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
    UpdateStats(duration.count() / 1000.0, collision_occurred, energy_drift);
}

void PhysicsEngine::StepEuler(std::vector<Body>& bodies) {
    auto start_time = std::chrono::high_resolution_clock::now();
    
    // Simple Euler integration: not recommended for orbital mechanics
    // v(t+dt) = v(t) + a(t) * dt
    // r(t+dt) = r(t) + v(t) * dt
    
    // Calculate accelerations
    std::vector<glm::dvec3> positions;
    positions.reserve(bodies.size());
    for (const auto& body : bodies) {
        positions.push_back(body.GetPosition());
    }
    
    auto accelerations = CalculateAccelerations(positions, bodies);
    
    // Update velocities and positions
    for (size_t i = 0; i < bodies.size(); ++i) {
        if (bodies[i].IsActive()) {
            glm::dvec3 new_velocity = bodies[i].GetVelocity() + accelerations[i] * time_step_;
            glm::dvec3 new_position = bodies[i].GetPosition() + bodies[i].GetVelocity() * time_step_;
            
            bodies[i].SetVelocity(new_velocity);
            bodies[i].SetPosition(new_position);
            bodies[i].UpdateTrail();
        }
    }
    
    // Handle collisions
    bool collision_occurred = HandleCollisions(bodies);
    
    // Update simulation time and step count
    simulation_time_ += time_step_;
    step_count_++;
    
    // Update performance statistics
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
    UpdateStats(duration.count() / 1000.0, collision_occurred, 0.0);
}

void PhysicsEngine::StepVerlet(std::vector<Body>& bodies) {
    auto start_time = std::chrono::high_resolution_clock::now();
    
    // Velocity Verlet integration - better energy conservation than Euler
    // r(t+dt) = r(t) + v(t)*dt + 0.5*a(t)*dt²
    // v(t+dt) = v(t) + 0.5*[a(t) + a(t+dt)]*dt
    
    // Calculate current accelerations
    std::vector<glm::dvec3> positions;
    positions.reserve(bodies.size());
    for (const auto& body : bodies) {
        positions.push_back(body.GetPosition());
    }
    
    auto current_accelerations = CalculateAccelerations(positions, bodies);
    
    // Update positions using current velocity and acceleration
    for (size_t i = 0; i < bodies.size(); ++i) {
        if (bodies[i].IsActive()) {
            glm::dvec3 new_position = bodies[i].GetPosition() + 
                                     bodies[i].GetVelocity() * time_step_ + 
                                     0.5 * prev_accelerations_[i] * time_step_ * time_step_;
            bodies[i].SetPosition(new_position);
        }
    }
    
    // Calculate new accelerations at new positions
    positions.clear();
    for (const auto& body : bodies) {
        positions.push_back(body.GetPosition());
    }
    
    auto new_accelerations = CalculateAccelerations(positions, bodies);
    
    // Update velocities using average of old and new accelerations
    for (size_t i = 0; i < bodies.size(); ++i) {
        if (bodies[i].IsActive()) {
            glm::dvec3 new_velocity = bodies[i].GetVelocity() + 
                                     0.5 * (prev_accelerations_[i] + new_accelerations[i]) * time_step_;
            bodies[i].SetVelocity(new_velocity);
            bodies[i].UpdateTrail();
        }
    }
    
    // Store current accelerations for next step
    prev_accelerations_ = new_accelerations;
    
    // Handle collisions
    bool collision_occurred = HandleCollisions(bodies);
    
    // Update simulation time and step count
    simulation_time_ += time_step_;
    step_count_++;
    
    // Update performance statistics
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
    UpdateStats(duration.count() / 1000.0, collision_occurred, 0.0);
}

bool PhysicsEngine::HandleCollisions(std::vector<Body>& bodies) {
    bool collision_occurred = false;
    
    // Check all pairs of bodies for collisions
    for (size_t i = 0; i < bodies.size(); ++i) {
        for (size_t j = i + 1; j < bodies.size(); ++j) {
            if (bodies[i].IsActive() && bodies[j].IsActive() && 
                bodies[i].IsCollidingWith(bodies[j])) {
                
                // Merge the two bodies
                Body merged_body = bodies[i].MergeWith(bodies[j]);
                
                // Keep the merged body in the first position
                bodies[i] = merged_body;
                
                // Deactivate the second body
                bodies[j].SetActive(false);
                
                collision_occurred = true;
                stats_.collision_count++;
                
                // Only handle one collision per timestep to avoid complications
                return collision_occurred;
            }
        }
    }
    
    return collision_occurred;
}

PhysicsEngine::EnergyInfo PhysicsEngine::CalculateEnergy(const std::vector<Body>& bodies) const {
    EnergyInfo energy;
    
    // Calculate kinetic energy
    for (const auto& body : bodies) {
        if (body.IsActive()) {
            energy.kinetic_energy += body.GetKineticEnergy();
        }
    }
    
    // Calculate potential energy
    for (size_t i = 0; i < bodies.size(); ++i) {
        for (size_t j = i + 1; j < bodies.size(); ++j) {
            if (bodies[i].IsActive() && bodies[j].IsActive()) {
                energy.potential_energy += CalculatePotentialEnergy(bodies[i], bodies[j]);
            }
        }
    }
    
    // Total energy
    energy.total_energy = energy.kinetic_energy + energy.potential_energy;
    
    // Energy drift calculation
    if (energy_initialized_) {
        energy.energy_drift = energy.total_energy - initial_energy_;
        if (std::abs(initial_energy_) > 1e-15) {
            energy.relative_drift = (energy.energy_drift / initial_energy_) * 100.0;
        }
    }
    
    return energy;
}

double PhysicsEngine::CalculatePotentialEnergy(const Body& body1, const Body& body2) const {
    const double distance = body1.DistanceTo(body2);
    
    // Gravitational potential energy: U = -G * m1 * m2 / r
    // Add softening to prevent singularities
    const double softened_distance = std::max(distance, Constants::SOFTENING_PARAMETER);
    return -Constants::G * body1.GetMass() * body2.GetMass() / softened_distance;
}

bool PhysicsEngine::IsSystemStable(const std::vector<Body>& bodies) const {
    // Simple stability check: bodies shouldn't be too far apart or moving too fast
    const double max_distance = 1000.0 * Constants::AU;  // 1000 AU
    const double max_speed = 1e6;  // 1 million m/s
    
    for (const auto& body : bodies) {
        if (!body.IsActive()) continue;
        
        // Check if body is too far from origin
        if (glm::length(body.GetPosition()) > max_distance) {
            return false;
        }
        
        // Check if body is moving too fast
        if (glm::length(body.GetVelocity()) > max_speed) {
            return false;
        }
    }
    
    return true;
}

void PhysicsEngine::SetTimeStep(double time_step) noexcept {
    time_step_ = std::clamp(time_step, min_time_step_, max_time_step_);
}

void PhysicsEngine::Reset() {
    simulation_time_ = 0.0;
    step_count_ = 0;
    energy_initialized_ = false;
    stats_ = {};
    step_times_.clear();
}

void PhysicsEngine::SetAdaptiveTimeStep(bool adaptive, double tolerance) {
    adaptive_time_step_ = adaptive;
    tolerance_ = tolerance;
}

PhysicsEngine::PerformanceStats PhysicsEngine::GetPerformanceStats() const {
    PerformanceStats result = stats_;
    result.total_steps = step_count_;
    
    if (!step_times_.empty()) {
        result.avg_step_time = std::accumulate(step_times_.begin(), step_times_.end(), 0.0) / step_times_.size();
    }
    
    return result;
}

PhysicsEngine::SystemState PhysicsEngine::BodiesToState(const std::vector<Body>& bodies) const {
    SystemState state(bodies.size());
    
    for (size_t i = 0; i < bodies.size(); ++i) {
        state.positions[i] = bodies[i].GetPosition();
        state.velocities[i] = bodies[i].GetVelocity();
    }
    
    return state;
}

void PhysicsEngine::StateToBodies(const SystemState& state, std::vector<Body>& bodies) const {
    for (size_t i = 0; i < bodies.size() && i < state.positions.size(); ++i) {
        bodies[i].SetPosition(state.positions[i]);
        bodies[i].SetVelocity(state.velocities[i]);
    }
}

PhysicsEngine::SystemState PhysicsEngine::CalculateDerivatives(const SystemState& state, const std::vector<Body>& bodies) const {
    SystemState derivatives(state.positions.size());
    
    // Position derivatives are velocities
    derivatives.positions = state.velocities;
    
    // Velocity derivatives are accelerations
    derivatives.velocities = CalculateAccelerations(state.positions, bodies);
    
    return derivatives;
}

std::vector<glm::dvec3> PhysicsEngine::CalculateAccelerations(const std::vector<glm::dvec3>& positions, const std::vector<Body>& bodies) const {
    std::vector<glm::dvec3> accelerations(positions.size(), glm::dvec3(0.0));
    
    // Calculate gravitational accelerations between all pairs
    for (size_t i = 0; i < bodies.size(); ++i) {
        if (!bodies[i].IsActive()) continue;
        
        for (size_t j = 0; j < bodies.size(); ++j) {
            if (i == j || !bodies[j].IsActive()) continue;
            
            const glm::dvec3 displacement = positions[j] - positions[i];
            const double distance_squared = glm::dot(displacement, displacement);
            
            // Add softening parameter
            const double softened_distance_squared = distance_squared + Constants::SOFTENING_PARAMETER;
            const double distance = std::sqrt(softened_distance_squared);
            
            // Calculate acceleration magnitude
            const double acceleration_magnitude = Constants::G * bodies[j].GetMass() / softened_distance_squared;
            
            // Add acceleration contribution
            accelerations[i] += acceleration_magnitude * (displacement / distance);
        }
    }
    
    return accelerations;
}

double PhysicsEngine::EstimateError(const std::vector<Body>& bodies, double dt) const {
    // Simple error estimation for adaptive time stepping
    // This is a basic implementation - more sophisticated methods exist
    double max_error = 0.0;
    
    for (const auto& body : bodies) {
        if (!body.IsActive()) continue;
        
        const double velocity_magnitude = glm::length(body.GetVelocity());
        const double position_magnitude = glm::length(body.GetPosition());
        
        // Estimate local truncation error
        const double error = dt * dt * velocity_magnitude / (position_magnitude + 1.0);
        max_error = std::max(max_error, error);
    }
    
    return max_error;
}

double PhysicsEngine::AdjustTimeStep(double error) const {
    if (error < tolerance_ * 0.1) {
        // Error is very small, can increase time step
        return std::min(time_step_ * 1.2, max_time_step_);
    } else if (error > tolerance_) {
        // Error is too large, decrease time step
        return std::max(time_step_ * 0.8, min_time_step_);
    }
    
    // Error is acceptable, keep current time step
    return time_step_;
}

void PhysicsEngine::UpdateStats(double step_time, bool collision_occurred, double energy_drift) const {
    step_times_.push_back(step_time);
    
    // Keep only recent step times for moving average
    if (step_times_.size() > 100) {
        step_times_.erase(step_times_.begin());
    }
    
    stats_.max_energy_drift = std::max