#ifndef SCENE_HPP
#define SCENE_HPP

// std
#include <vector>

// glm
#include <glm/glm.hpp>

// project
#include "boid.hpp" // Include Boid definition first
#include "cgra/cgra_mesh.hpp"
#include "cgra/cgra_shader.hpp"
#include "obstacle.hpp" // Then include Obstacle

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

  // Bounce/wrap
  bool m_bounce = false;
  bool m_wrap = false;

  int m_num_boids = 100;
  int m_num_predators = 1;
  int m_num_flocks = 3;
  int m_num_boids_per_flock = 100;

  // Boid Weights
  float m_avoidance_weight = 0.2f;
  float m_cohesion_weight = 0.4f;
  float m_alignment_weight = 0.5f;
  float m_min_speed = 1.0f;
  float m_max_speed = 5.0f;
  float m_soft_bound = 0.1f;
  float m_soft_bound_weight = 0.5f;

  // Predator Weights
  float m_predator_avoidance_weight = 0.0f;
  float m_predator_chase_weight = 0.0f;

  float m_boid_radius = 5.0f;

  // scene data
  glm::vec3 m_bound_hsize = glm::vec3(20);
  std::vector<Boid> m_boids;
  std::vector<Obstacle> m_obstacles;

  // obstacle data
  int m_num_obstacles = 10;
  float m_obstacle_radius = 2.0f;

  //-------------------------------------------------------------
  // [Assignment 3] :
  // Create variables for keeping track of the boid parameters
  // such as min and max speed etc. These parameters can either be
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

  const std::vector<Obstacle> &obstacles() const { return m_obstacles; }

  // returns the half-size of the bounding box (centered around the origin)
  glm::vec3 bound() const { return m_bound_hsize; }

  float alignmentWeight() const { return m_alignment_weight; }
  float avoidanceWeight() const { return m_avoidance_weight; }
  float cohesionWeight() const { return m_cohesion_weight; }
  float minSpeed() const { return m_min_speed; }
  float maxSpeed() const { return m_max_speed; }
  float boidRadius() const { return m_boid_radius; }
  float softBound() const { return m_soft_bound; }
  float softBoundWeight() const { return m_soft_bound_weight; }
  float predatorAvoidanceWeight() const { return m_predator_avoidance_weight; }
  float predatorChaseWeight() const { return m_predator_chase_weight; }
  bool bounce() const { return m_bounce; }
  bool wrap() const { return m_wrap; }
  // In scene.hpp, add the drawObstacles declaration:
  void drawObstacles(const glm::mat4 &proj, const glm::mat4 &view);
};

#endif // SCENE_HPP