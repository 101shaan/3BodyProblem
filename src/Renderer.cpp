#include "Renderer.h"

#include <format>
#include <numbers>
#include <ranges>

Renderer::Renderer(unsigned int width, unsigned int height, const char* title)
    : window_{sf::VideoMode{width, height}, title, sf::Style::Titlebar | sf::Style::Close}
{
    window_.setVerticalSyncEnabled(true);

    // Try to load a system font. Fallback to default if not found.
#ifdef _WIN32
    const char* defaultFont = "C:\\Windows\\Fonts\\consola.ttf";
#else
    const char* defaultFont = "/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf";
#endif

    if (!font_.loadFromFile(defaultFont)) {
        // If loading fails, create a minimal font texture so we can still run.
        font_.loadFromMemory(nullptr, 0); // empty => SFML will handle gracefully
    }
}

bool Renderer::ProcessEvents() {
    sf::Event ev;
    while (window_.pollEvent(ev)) {
        if (ev.type == sf::Event::Closed) {
            window_.close();
        }
    }
    return window_.isOpen();
}

void Renderer::RenderFrame(const std::vector<Body>& bodies,
                           const PhysicsEngine::EnergyInfo& energyInfo,
                           bool showEnergy,
                           bool showTrails) {
    window_.clear(sf::Color(static_cast<sf::Uint8>(Constants::BACKGROUND_COLOR.r * 255.f),
                            static_cast<sf::Uint8>(Constants::BACKGROUND_COLOR.g * 255.f),
                            static_cast<sf::Uint8>(Constants::BACKGROUND_COLOR.b * 255.f)));

    // Draw trails first so they appear under bodies.
    if (showTrails) {
        for (const auto& body : bodies) {
            const auto& trail = body.GetTrail();
            if (trail.size() < 2) continue;

            sf::VertexArray va(sf::LineStrip, trail.size());
            size_t idx = 0;
            for (const auto& p : trail) {
                auto s = WorldToScreen(p);
                va[idx].position = {s.x, s.y};
                va[idx].color = sf::Color(180, 180, 180, static_cast<sf::Uint8>(255 * std::pow(Constants::TRAIL_FADE_RATE, static_cast<float>(trail.size() - idx))));
                ++idx;
            }
            window_.draw(va);
        }
    }

    // Draw bodies.
    for (const auto& body : bodies) {
        if (!body.IsActive()) continue;

        const float radius = static_cast<float>(std::clamp(body.GetRadius(), static_cast<double>(Constants::MIN_BODY_SIZE), static_cast<double>(Constants::MAX_BODY_SIZE)));
        sf::CircleShape circle{radius};
        circle.setOrigin(radius, radius);
        circle.setFillColor({static_cast<sf::Uint8>(body.GetColor().r * 255.f),
                             static_cast<sf::Uint8>(body.GetColor().g * 255.f),
                             static_cast<sf::Uint8>(body.GetColor().b * 255.f)});
        auto screenPos = WorldToScreen(body.GetPosition());
        circle.setPosition(screenPos);
        window_.draw(circle);
    }

    // Energy statistics.
    if (showEnergy) {
        std::string energyStr = std::format("E_tot = {:.3e}  |  drift = {:.3e} ({:.2f}%)",
                                            energyInfo.total_energy,
                                            energyInfo.energy_drift,
                                            energyInfo.relative_drift);
        sf::Text text{energyStr, font_, 14};
        text.setFillColor(sf::Color::White);
        text.setPosition(10.f, 10.f);
        window_.draw(text);
    }

    window_.display();
}

sf::Vector2f Renderer::WorldToScreen(const glm::dvec3& pos) const {
    // Simple orthographic projection mapping x -> horizontal, y -> vertical.
    // We scale to fit within window with some padding.
    constexpr double scale = 50.0; // pixels per simulation unit (tweakable)
    const double halfW = window_.getSize().x / 2.0;
    const double halfH = window_.getSize().y / 2.0;
    return {static_cast<float>(halfW + pos.x * scale), static_cast<float>(halfH - pos.y * scale)};
}
