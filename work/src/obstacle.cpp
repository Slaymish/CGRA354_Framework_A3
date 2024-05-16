#include "obstacle.hpp"
#include "boid.hpp"
#include <glm/glm.hpp>
#include <glm/gtc/constants.hpp>

bool Obstacle::isColliding(const Boid &boid) const {
  return glm::distance(m_position, boid.position()) < m_radius;
}

glm::vec3 Obstacle::avoid(const Boid &boid, float neighbourRadius) const {
  glm::vec3 toBoid = boid.position() - m_position;
  float distance = glm::length(toBoid);

  if (distance < m_radius + neighbourRadius) {
    glm::vec3 normal = glm::normalize(
        toBoid); // Normal vector pointing from the obstacle to the boid
    glm::vec3 avoidanceForce = glm::reflect(boid.velocity(), normal);

    // Scale the avoidance force based on how close the boid is to the obstacle
    float strength = (1.0f - distance / (m_radius + neighbourRadius)) *
                     (m_radius + neighbourRadius - distance);
    return glm::normalize(avoidanceForce) * strength;
  }

  return glm::vec3(0);
}