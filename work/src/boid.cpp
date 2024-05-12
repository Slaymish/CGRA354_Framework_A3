
// glm
#include <glm/gtc/random.hpp>

// project
#include "boid.hpp"
#include "cgra/cgra_mesh.hpp"
#include "scene.hpp"

using namespace glm;
using namespace std;

vec3 Boid::color() const { return vec3(0, 1, 0); }

void Boid::calculateForces(Scene *scene) {
  //-------------------------------------------------------------
  // [Assignment 3] :
  // Calculate the forces affecting the boid and update the
  // acceleration (assuming mass = 1).
  // Do NOT update velocity or position in this function.
  // Core :
  //  - Cohesion
  //  - Alignment
  //  - Avoidance
  //  - Soft Bound (optional)
  // Completion :
  //  - Cohesion and Alignment with only boids in the same flock
  //  - Predator Avoidance (boids only)
  //  - Predator Chase (predator only)
  // Challenge :
  //  - Obstacle avoidance
  //-------------------------------------------------------------

  // YOUR CODE GOES HERE
  // ...

  // Alignment

  // average_velocity = 0
  // foreach other boid in boids:
  //  average_velocity += other_boid.velocity

  // average_velocity /= neighbours
  // alignment = average_neighbour_position - boid.position

  // Alignment

  glm::vec3 average_velocity = glm::vec3(0);
  glm::vec3 average_position = glm::vec3(0);
  glm::vec3 avoidance = glm::vec3(0);
  int neighbours = 0;

  for (const Boid &other : scene->boids()) {
    if (&other != this && glm::distance(other.position(), this->position()) <
                              scene->boidRadius()) {
      average_velocity += other.velocity(); // for alignment
      average_position += other.position(); // for cohesion
      avoidance +=
          (this->position() - other.position()) /
          glm::distance(this->position(), other.position()); // for
                                                             // avoidance
      neighbours++;
    }
  }

  if (neighbours > 0) {
    average_velocity /= neighbours;
    average_position /= neighbours;

    // Alignment
    glm::vec3 alignment =
        (average_velocity - this->velocity()) * scene->alignmentWeight();

    // Cohesion
    glm::vec3 cohesion =
        (average_position - this->position()) * scene->cohesionWeight();

    // Avoidance
    glm::vec3 avoidanceForce = avoidance * scene->avoidanceWeight();

    m_acceleration += alignment + cohesion + avoidanceForce;
  }

  // Soft Bound
  glm::vec3 bound = scene->bound();
  float soft_bound = scene->softBound();
  glm::vec3 soft_bound_force = glm::vec3(0);

  if (m_position.x < -bound.x + soft_bound)
    soft_bound_force.x = 1;
  if (m_position.x > bound.x - soft_bound)
    soft_bound_force.x = -1;
  if (m_position.y < -bound.y + soft_bound)
    soft_bound_force.y = 1;
  if (m_position.y > bound.y - soft_bound)
    soft_bound_force.y = -1;
  if (m_position.z < -bound.z + soft_bound)
    soft_bound_force.z = 1;
  if (m_position.z > bound.z - soft_bound)
    soft_bound_force.z = -1;

  m_acceleration += soft_bound_force;
}

void Boid::update(float timestep, Scene *scene) {
  //-------------------------------------------------------------
  // [Assignment 3] :
  // Integrate the velocity of the boid using the timestep.
  // Update the position of the boid using the new velocity.
  // Take into account the bounds of the scene which may
  // require you to change the velocity (if bouncing) or
  // change the position (if wrapping).
  //-------------------------------------------------------------

  // update velocity
  m_velocity += m_acceleration * timestep;

  // Limit speed
  if (glm::length(m_velocity) > scene->maxSpeed()) {
    m_velocity = glm::normalize(m_velocity) * scene->maxSpeed();
  }

  // update position
  m_position += m_velocity * timestep;

  // Bounce at the walls
  glm::vec3 bound = scene->bound();
  if (m_position.x < -bound.x || m_position.x > bound.x)
    m_velocity.x = -m_velocity.x;
  if (m_position.y < -bound.y || m_position.y > bound.y)
    m_velocity.y = -m_velocity.y;
  if (m_position.z < -bound.z || m_position.z > bound.z)
    m_velocity.z = -m_velocity.z;
}