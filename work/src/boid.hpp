#ifndef BOID_HPP
#define BOID_HPP

// glm
#include <glm/glm.hpp>

// project
#include "scene.hpp"

class Boid {
private:
  glm::vec3 m_position;
  glm::vec3 m_velocity;
  glm::vec3 m_acceleration;

  int flockID; // if -1, then it is a predator

public:
  Boid(glm::vec3 pos, glm::vec3 dir, int flock)
      : m_position(pos), m_velocity(dir), m_acceleration(glm::vec3(0, 0, 0)),
        flockID(flock) {}

  glm::vec3 position() const { return m_position; }
  glm::vec3 velocity() const { return m_velocity; }
  glm::vec3 acceleration() const { return m_acceleration; }
  int flock() const { return flockID; }
  bool isPredator() const { return flockID == -1; }

  glm::vec3 color() const;

  void calculateForces(Scene *scene);
  void update(float timestep, Scene *scene);
};

#endif // BOID_HPP