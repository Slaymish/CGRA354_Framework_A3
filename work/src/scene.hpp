
#pragma once

// std
#include <vector>

// glm
#include <glm/glm.hpp>

// project
#include "cgra/cgra_mesh.hpp"
#include "cgra/cgra_shader.hpp"

// boid class (forward declaration)
class Boid;

class Scene {
private:
  // opengl draw data
  GLuint m_color_shader = 0;
  GLuint m_aabb_shader = 0;
  GLuint m_skymap_shader = 0;
  cgra::gl_mesh m_simple_boid_mesh;
  cgra::gl_mesh m_boid_mesh;
  cgra::gl_mesh m_predator_mesh;
  cgra::gl_mesh m_sphere_mesh;

  // draw status
  bool m_show_aabb = true;
  bool m_show_axis = false;
  bool m_show_skymap = false;

  // Boid Weights
  float m_avoidance_weight = 0.0f;
  float m_cohesion_weight = 0.0f;
  float m_alignment_weight = 0.0f;
  float m_min_speed = 0.0f;
  float m_max_speed = 0.0f;
  float m_soft_bound = 0.0f;

  float m_boid_radius = 5.0f;

  // scene data
  glm::vec3 m_bound_hsize = glm::vec3(20);
  std::vector<Boid> m_boids;

  //-------------------------------------------------------------
  // [Assignment 3] :
  // Create variables for keeping track of the boid parameters
  // such as min and max speed etc. These paramters can either be
  // public, or private with getter functions.
  //-------------------------------------------------------------

  // YOUR CODE GOES HERE
  // ...

public:
  Scene();

  // functions that load the scene
  void loadCore();
  void loadCompletion();
  void loadChallenge();

  // called every frame, with timestep in seconds
  void update(float timestep);

  // called every frame, with the given projection and view matrix
  void draw(const glm::mat4 &proj, const glm::mat4 &view);

  // called every frame (to fill out a ImGui::TreeNode)
  void renderGUI();

  // returns a const reference to the boids vector
  const std::vector<Boid> &boids() const { return m_boids; }

  // returns the half-size of the bounding box (centered around the origin)
  glm::vec3 bound() const { return m_bound_hsize; }

  float alignmentWeight() const { return m_alignment_weight; }
  float avoidanceWeight() const { return m_avoidance_weight; }
  float cohesionWeight() const { return m_cohesion_weight; }
  float minSpeed() const { return m_min_speed; }
  float maxSpeed() const { return m_max_speed; }
  float boidRadius() const { return m_boid_radius; }
  float softBound() const { return m_soft_bound; }
};