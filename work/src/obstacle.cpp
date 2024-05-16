#include "obstacle.hpp"
#include "boid.hpp"
#include <glm/glm.hpp>
#include <glm/gtc/constants.hpp>

bool Obstacle::isColliding(const Boid &boid) const {
  return glm::distance(m_position, boid.position()) < m_radius;
}

glm::vec3 Obstacle::avoid(const Boid &boid) const {
  glm::vec3 toObstacle = m_position - boid.position();
  float distance = glm::length(toObstacle);
  float scaler = 1.0f - glm::clamp(distance / m_radius, 0.0f, 1.0f);
  return glm::normalize(toObstacle) * scaler;
}