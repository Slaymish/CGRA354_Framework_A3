
// glm
#include <glm/gtc/random.hpp>

// project
#include "boid.hpp"
#include "cgra/cgra_mesh.hpp"
#include "scene.hpp"

using namespace glm;
using namespace std;

vec3 Boid::color() const {
  // use m_num_flocks to determine the number of colors
  // use flockID to determine the color of the boid
  // -1 is predator (red)
  // return the color as a vec3

  std::vector<vec3> colors = {
      vec3(0, 1, 0), // green
      vec3(0, 0, 1), // blue
      vec3(1, 1, 0), // yellow
      vec3(1, 0, 1), // magenta
      vec3(0, 1, 1), // cyan
  };

  if (flockID == -1) {
    return vec3(1, 0, 0); // red
  }

  return colors[flockID % colors.size()];
}

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

  if (isPredator()) {
    glm::vec3 chase_force = glm::vec3(0);
    int prey_count = 0;

    for (const Boid &other : scene->boids()) {
      if (!other.isPredator() &&
          glm::distance(other.position(), this->position()) <
              scene->boidRadius()) {
        chase_force += (other.position() - this->position()) /
                       glm::distance(other.position(), this->position());
        prey_count++;
      }
    }

    if (prey_count > 0) {
      chase_force /= prey_count;
      chase_force = glm::normalize(chase_force) * scene->predatorChaseWeight();
      m_acceleration = chase_force;
    } else {
      m_acceleration = glm::vec3(0); // No prey nearby, no acceleration
    }
  } else {
    glm::vec3 average_velocity = glm::vec3(0);
    glm::vec3 average_position = glm::vec3(0);
    glm::vec3 avoidance = glm::vec3(0);
    glm::vec3 predator_avoidance = glm::vec3(0);
    int neighbours = 0;
    bool predatorNearby = false;

    for (const Boid &other : scene->boids()) {
      float distance = glm::distance(other.position(), this->position());
      if (&other != this && distance < scene->boidRadius()) {
        if (other.isPredator()) {
          predatorNearby = true;
          predator_avoidance +=
              (this->position() - other.position()) / distance;
        } else if (other.flockID == this->flockID) {
          average_velocity += other.velocity(); // for alignment
          average_position += other.position(); // for cohesion
          avoidance +=
              (this->position() - other.position()) / distance; // for avoidance
          neighbours++;
        }
      }
    }

    if (neighbours > 0) {
      average_velocity /= neighbours;
      average_position /= neighbours;

      // Alignment
      glm::vec3 alignment = average_velocity - m_velocity;
      alignment = glm::normalize(alignment) * scene->alignmentWeight();

      // Cohesion
      glm::vec3 cohesion = average_position - this->position();
      cohesion = glm::normalize(cohesion) * scene->cohesionWeight();

      // Avoidance
      glm::vec3 avoidanceForce =
          glm::normalize(avoidance) * scene->avoidanceWeight();

      m_acceleration =
          alignment + cohesion + avoidanceForce; // update acceleration
    } else {
      m_acceleration = glm::vec3(0); // No neighbours, no acceleration
    }

    // Predator Avoidance
    if (predatorNearby) {
      predator_avoidance =
          glm::normalize(predator_avoidance) * scene->predatorAvoidanceWeight();
      m_acceleration += predator_avoidance;
    }
  }

  // Obstacle Avoidance
  glm::vec3 obstacle_avoidance = glm::vec3(0);
  for (const Obstacle &obstacle : scene->obstacles()) {
    obstacle_avoidance += obstacle.avoid(*this);
  }

  if (glm::length(obstacle_avoidance) > 0) {
    obstacle_avoidance =
        glm::normalize(obstacle_avoidance) * scene->avoidanceWeight();
    m_acceleration += obstacle_avoidance;
  }

  // Soft Bound
  glm::vec3 bound = scene->bound();
  float soft_bound = scene->softBound();
  glm::vec3 soft_bound_force = glm::vec3(0);

  if (m_position.x < -bound.x + soft_bound)
    soft_bound_force.x = (-bound.x + soft_bound - m_position.x);
  if (m_position.x > bound.x - soft_bound)
    soft_bound_force.x = (bound.x - soft_bound - m_position.x);
  if (m_position.y < -bound.y + soft_bound)
    soft_bound_force.y = (-bound.y + soft_bound - m_position.y);
  if (m_position.y > bound.y - soft_bound)
    soft_bound_force.y = (bound.y - soft_bound - m_position.y);
  if (m_position.z < -bound.z + soft_bound)
    soft_bound_force.z = (-bound.z + soft_bound - m_position.z);
  if (m_position.z > bound.z - soft_bound)
    soft_bound_force.z = (bound.z - soft_bound - m_position.z);

  if (glm::length(soft_bound_force) > 0) {
    soft_bound_force =
        glm::normalize(soft_bound_force) * scene->softBoundWeight();

    soft_bound_force *=
        glm::clamp(glm::distance(m_position, glm::vec3(0)), 1.0f, 5.0f);

    m_acceleration += soft_bound_force;
  }
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

  // Update velocity with acceleration
  m_velocity += m_acceleration * timestep;

  // Limit speed
  float speed = glm::length(m_velocity);
  if (speed > scene->maxSpeed()) {
    m_velocity = glm::normalize(m_velocity) * scene->maxSpeed();
  } else if (speed < scene->minSpeed()) {
    m_velocity = glm::normalize(m_velocity) * scene->minSpeed();
  }

  // Update position
  m_position += m_velocity * timestep;

  // Bounce at the walls
  glm::vec3 bound = scene->bound(); // half-size of the bounding box
  if (scene->bounce()) {
    if (m_position.x < -bound.x || m_position.x > bound.x)
      m_velocity.x *= -1;
    if (m_position.y < -bound.y || m_position.y > bound.y)
      m_velocity.y *= -1;
    if (m_position.z < -bound.z || m_position.z > bound.z)
      m_velocity.z *= -1;
  }

  // Wrap around the walls
  if (scene->wrap()) {
    if (m_position.x < -bound.x)
      m_position.x = bound.x;
    if (m_position.x > bound.x)
      m_position.x = -bound.x;
    if (m_position.y < -bound.y)
      m_position.y = bound.y;
    if (m_position.y > bound.y)
      m_position.y = -bound.y;
    if (m_position.z < -bound.z)
      m_position.z = bound.z;
    if (m_position.z > bound.z)
      m_position.z = -bound.z;
  }
}