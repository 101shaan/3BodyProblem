#pragma once

#include "Body.h"
#include <vector>
#include <memory>
#include <functional>

/**
 * @brief Physics engine for N-body gravitational simulation
 * 
 * This class implements a 4th-order Runge-Kutta integrator for solving
 * the gravitational N-body problem. It handles collision detection,
 * energy calculations, and provides various integration methods.
 */
class PhysicsEngine {
public:
    /**
     * @brief Structure to hold system state for RK4 integration
     */
    struct SystemState {
        std::vector<glm::dvec3> positions;    ///< Position vectors
        std::vector<glm::dvec3> velocities;   ///< Velocity vectors
        
        SystemState() = default;
        explicit SystemState(size_t num_bodies);
        SystemState(const SystemState& other) = default;
        SystemState(SystemState&& other) = default;
        SystemState& operator=(const SystemState& other) = default;
        SystemState& operator=(SystemState&& other) = default;
        
        SystemState operator+(const SystemState& other) const;
        SystemState operator*(double scalar) const;
        SystemState& operator+=(const SystemState& other);
        SystemState& operator*=(double scalar);
    };
    
    /**
     * @brief Energy information for the system
     */
    struct EnergyInfo {
        double kinetic_energy;      ///< Total kinetic energy (J)
        double potential_energy;    ///< Total potential energy (J)
        double total_energy;        ///< Total energy (J)
        double energy_drift;        ///< Energy drift from initial (J)
        double relative_drift;      ///< Relative energy drift (%)
        
        EnergyInfo() : kinetic_energy(0.0), potential_energy(0.0), 
                      total_energy(0.0), energy_drift(0.0), relative_drift(0.0) {}
    };
    
    /**
     * @brief Default constructor
     */
    PhysicsEngine();
    
    /**
     * @brief Constructor with specified time step
     * @param time_step Integration time step (s)
     */
    explicit PhysicsEngine(double time_step);
    
    /**
     * @brief Destructor
     */
    ~PhysicsEngine() = default;
    
    /**
     * @brief Initialize the physics engine with bodies
     * @param bodies Vector of bodies to simulate
     */
    void Initialize(const std::vector<Body>& bodies);
    
    /**
     * @brief Perform one integration step using RK4
     * @param bodies Vector of bodies to update
     */
    void StepRK4(std::vector<Body>& bodies);
    
    /**
     * @brief Perform one integration step using simple Euler method
     * @param bodies Vector of bodies to update
     */
    void StepEuler(std::vector<Body>& bodies);
    
    /**
     * @brief Perform one integration step using Velocity Verlet
     * @param bodies Vector of bodies to update
     */
    void StepVerlet(std::vector<Body>& bodies);
    
    /**
     * @brief Handle collisions between bodies
     * @param bodies Vector of bodies to check for collisions
     * @return True if any collisions occurred
     */
    bool HandleCollisions(std::vector<Body>& bodies);
    
    /**
     * @brief Calculate total energy of the system
     * @param bodies Vector of bodies
     * @return Energy information structure
     */
    EnergyInfo CalculateEnergy(const std::vector<Body>& bodies) const;
    
    /**
     * @brief Calculate gravitational potential energy between two bodies
     * @param body1 First body
     * @param body2 Second body
     * @return Potential energy (J)
     */
    double CalculatePotentialEnergy(const Body& body1, const Body& body2) const;
    
    /**
     * @brief Check if the system is stable (bounded orbits)
     * @param bodies Vector of bodies
     * @return True if system appears stable
     */
    bool IsSystemStable(const std::vector<Body>& bodies) const;
    
    /**
     * @brief Get/Set time step
     */
    double GetTimeStep() const noexcept { return time_step_; }
    void SetTimeStep(double time_step) noexcept;
    
    /**
     * @brief Get simulation time
     */
    double GetSimulationTime() const noexcept { return simulation_time_; }
    
    /**
     * @brief Get number of integration steps performed
     */
    uint64_t GetStepCount() const noexcept { return step_count_; }
    
    /**
     * @brief Reset simulation time and step count
     */
    void Reset();
    
    /**
     * @brief Get initial energy (for drift calculation)
     */
    double GetInitialEnergy() const noexcept { return initial_energy_; }
    
    /**
     * @brief Set adaptive time stepping
     * @param adaptive Enable/disable adaptive time stepping
     * @param tolerance Error tolerance for adaptive stepping
     */
    void SetAdaptiveTimeStep(bool adaptive, double tolerance = 1e-6);
    
    /**
     * @brief Get performance statistics
     */
    struct PerformanceStats {
        double avg_step_time;       ///< Average time per integration step (ms)
        uint64_t collision_count;   ///< Number of collisions detected
        uint64_t total_steps;       ///< Total integration steps
        double max_energy_drift;    ///< Maximum energy drift observed
    };
    
    PerformanceStats GetPerformanceStats() const;

private:
    double time_step_;              ///< Integration time step (s)
    double simulation_time_;        ///< Current simulation time (s)
    uint64_t step_count_;          ///< Number of integration steps performed
    double initial_energy_;         ///< Initial total energy for drift calculation
    bool energy_initialized_;       ///< Whether initial energy has been set
    
    // Adaptive time stepping
    bool adaptive_time_step_;       ///< Enable adaptive time stepping
    double tolerance_;              ///< Error tolerance for adaptive stepping
    double min_time_step_;          ///< Minimum allowed time step
    double max_time_step_;          ///< Maximum allowed time step
    
    // Performance tracking
    mutable PerformanceStats stats_;
    mutable std::vector<double> step_times_;
    
    // Previous state for Verlet integration
    std::vector<glm::dvec3> prev_accelerations_;
    
    /**
     * @brief Convert bodies to system state
     * @param bodies Vector of bodies
     * @return System state structure
     */
    SystemState BodiesToState(const std::vector<Body>& bodies) const;
    
    /**
     * @brief Apply system state to bodies
     * @param state System state structure
     * @param bodies Vector of bodies to update
     */
    void StateToBodies(const SystemState& state, std::vector<Body>& bodies) const;
    
    /**
     * @brief Calculate system derivatives for RK4 integration
     * @param state Current system state
     * @param bodies Vector of bodies (for masses)
     * @return Derivative of system state
     */
    SystemState CalculateDerivatives(const SystemState& state, const std::vector<Body>& bodies) const;
    
    /**
     * @brief Calculate all gravitational accelerations
     * @param positions Current positions
     * @param bodies Vector of bodies (for masses)
     * @return Vector of accelerations
     */
    std::vector<glm::dvec3> CalculateAccelerations(const std::vector<glm::dvec3>& positions, 
                                                   const std::vector<Body>& bodies) const;
    
    /**
     * @brief Estimate integration error for adaptive time stepping
     * @param bodies Vector of bodies
     * @param dt Current time step
     * @return Estimated error
     */
    double EstimateError(const std::vector<Body>& bodies, double dt) const;
    
    /**
     * @brief Adjust time step based on error estimate
     * @param error Current error estimate
     * @return New time step
     */
    double AdjustTimeStep(double error) const;
    
    /**
     * @brief Update performance statistics
     * @param step_time Time taken for this step (ms)
     * @param collision_occurred Whether a collision occurred
     * @param energy_drift Current energy drift
     */
    void UpdateStats(double step_time, bool collision_occurred, double energy_drift) const;
};