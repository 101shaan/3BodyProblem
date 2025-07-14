#include "ConfigManager.h"
#include "Constants.h"
#include <fstream>

ConfigManager::ConfigManager(const std::string& jsonPath) {
    std::ifstream ifs{jsonPath};
    if (ifs) {
        ifs >> root_;
    }
}

std::vector<Body> ConfigManager::LoadInitialBodies() const {
    std::vector<Body> bodies;
    for (const auto& item : root_["bodies"]) {
        double mass   = item.value("mass", 1.0);
        double radius = item.value("radius", 1.0);
        glm::dvec3 pos{item["position"][0].get<double>(), item["position"][1].get<double>(), item["position"][2].get<double>()};
        glm::dvec3 vel{item["velocity"][0].get<double>(), item["velocity"][1].get<double>(), item["velocity"][2].get<double>()};
        glm::vec3  color{item["color"][0].get<float>(), item["color"][1].get<float>(), item["color"][2].get<float>()};
        bodies.emplace_back(mass, pos, vel, radius, color);
    }
    return bodies;
}
