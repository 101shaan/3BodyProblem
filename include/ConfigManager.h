#pragma once

#include <nlohmann/json.hpp>
#include <string>
#include <vector>
#include "Body.h"

/**
 * @brief Loads initial conditions from JSON configuration files.
 */
class ConfigManager {
public:
    explicit ConfigManager(const std::string& jsonPath);

    /**
     * @brief Parses the JSON and returns a vector of bodies representing the initial state.
     */
    std::vector<Body> LoadInitialBodies() const;

private:
    nlohmann::json root_;
};
