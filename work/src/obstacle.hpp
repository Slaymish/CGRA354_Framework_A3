#ifndef OBSTACLE_HPP
#define OBSTACLE_HPP

#include <glm/glm.hpp>

class Boid;

class Obstacle {
private:
  glm::vec3 m_position;
  float m_radius;

public:
  Obstacle(glm::vec3 pos, float rad) : m_position(pos), m_radius(rad) {}

  glm::vec3 position() const { return m_position; }
  float radius() const { return m_radius; }

  bool isColliding(const Boid &boid) const;
  glm::vec3 avoid(const Boid &boid) const;
};

#endif // OBSTACLE_HPP