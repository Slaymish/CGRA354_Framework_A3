
// std
#include <random>

// stb
#include <stb_image.h>

// imgui
#include <imgui.h>

// glm
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/random.hpp>
#include <glm/gtc/type_ptr.hpp>

// project

#include "boid.hpp"
#include "cgra/cgra_geometry.hpp"
#include "cgra/cgra_image.hpp"
#include "cgra/cgra_wavefront.hpp"
#include "obstacle.hpp"
#include "scene.hpp"

using namespace glm;
using namespace std;

Scene::Scene() {

  // load meshes
  cgra::mesh_builder simple_boid_md =
      cgra::load_wavefront_data(CGRA_WORKDIR + string("res/models/boid.obj"));
  m_simple_boid_mesh = simple_boid_md.build();

  cgra::mesh_builder boid_md = cgra::load_wavefront_data(
      CGRA_WORKDIR + string("res/models/spaceship_boid.obj"));
  m_boid_mesh = boid_md.build();

  cgra::mesh_builder predator_md = cgra::load_wavefront_data(
      CGRA_WORKDIR + string("res/models/predator_boid.obj"));
  m_predator_mesh = predator_md.build();

  cgra::mesh_builder sphere_md =
      cgra::load_wavefront_data(CGRA_WORKDIR + string("res/models/sphere.obj"));
  m_sphere_mesh = sphere_md.build();

  // load color shader
  cgra::shader_builder color_sp;
  color_sp.set_shader(GL_VERTEX_SHADER,
                      CGRA_WORKDIR + string("res/shaders/simple_color.glsl"));
  color_sp.set_shader(GL_FRAGMENT_SHADER,
                      CGRA_WORKDIR + string("res/shaders/simple_color.glsl"));
  m_color_shader = color_sp.build();

  // load aabb shader
  cgra::shader_builder aabb_sp;
  aabb_sp.set_shader(GL_VERTEX_SHADER,
                     CGRA_WORKDIR + string("res/shaders/aabb.glsl"));
  aabb_sp.set_shader(GL_GEOMETRY_SHADER,
                     CGRA_WORKDIR + string("res/shaders/aabb.glsl"));
  aabb_sp.set_shader(GL_FRAGMENT_SHADER,
                     CGRA_WORKDIR + string("res/shaders/aabb.glsl"));
  m_aabb_shader = aabb_sp.build();

  // load skymap shader
  cgra::shader_builder skymap_sp;
  skymap_sp.set_shader(GL_VERTEX_SHADER,
                       CGRA_WORKDIR + string("res/shaders/skymap.glsl"));
  skymap_sp.set_shader(GL_GEOMETRY_SHADER,
                       CGRA_WORKDIR + string("res/shaders/skymap.glsl"));
  skymap_sp.set_shader(GL_FRAGMENT_SHADER,
                       CGRA_WORKDIR + string("res/shaders/skymap.glsl"));
  m_skymap_shader = skymap_sp.build();
}

void Scene::loadCore() {
  //-------------------------------------------------------------
  // [Assignment 3] (Core) :
  // Initialize the scene with 100-300 boids in random locations
  // inside the current bound size.
  //-------------------------------------------------------------

  // YOUR CODE GOES HERE
  // ...

  // CLear all related data
  m_boids.clear();
  m_obstacles.clear();

  for (int i = 0; i <= m_num_boids; i++) {
    m_boids.push_back(
        Boid(linearRand(-m_bound_hsize * 0.9f, m_bound_hsize * 0.9f),
             sphericalRand(1.0), 0));
  }
}

void Scene::loadCompletion() {
  //-------------------------------------------------------------
  // [Assignment 3] (Completion) :
  // Initialize the scene with 2 different flocks of boids,
  // 75-150 in each flock, in random locations inside the current
  // bound size. Additionally include at least one Predator.
  //-------------------------------------------------------------

  m_boids.clear();
  m_obstacles.clear();

  for (int i = 0; i < m_num_flocks; i++) {
    for (int j = 0; j < m_num_boids_per_flock; j++) {
      m_boids.push_back(
          Boid(linearRand(-m_bound_hsize * 0.9f, m_bound_hsize * 0.9f),
               sphericalRand(1.0), i));
    }
  }

  for (int i = 0; i < m_num_predators; i++) {
    m_boids.push_back(
        Boid(linearRand(-m_bound_hsize * 0.9f, m_bound_hsize * 0.9f),
             sphericalRand(1.0), -1));
  }
}

void Scene::loadChallenge() {
  //-------------------------------------------------------------
  // [Assignment 3] (Challenge) :
  // Initalize the scene with 100-300 boids in random locations
  // inside the current bound size. Additionally add at least
  // three spheres with a large radius inside the bounds.
  //-------------------------------------------------------------

  // YOUR CODE GOES HERE

  m_boids.clear();
  m_obstacles.clear();

  for (int i = 0; i <= m_num_boids; i++) {
    m_boids.push_back(
        Boid(linearRand(-m_bound_hsize * 0.9f, m_bound_hsize * 0.9f),
             sphericalRand(1.0), 0));
  }

  for (int i = 0; i < m_num_obstacles; i++) {
    vec3 center = vec3(0.0f); // Center of the box
    vec3 position =
        center + linearRand(-m_bound_hsize * 0.8f, m_bound_hsize * 0.8f);
    m_obstacles.push_back(Obstacle(position, m_obstacle_radius));
  }
}

void Scene::update(float timestep) {
  for (Boid &b : m_boids) {
    b.calculateForces(this);
  }

  for (Boid &b : m_boids) {
    b.update(timestep, this);
  }
}

void Scene::draw(const mat4 &proj, const mat4 &view) {

  // draw skymap (magically)
  //
  if (m_show_skymap) {
    static GLuint tex = 0;
    if (!tex) {
      tex = cgra::rgba_image(CGRA_WORKDIR + string("res/textures/sky.jpg"))
                .uploadTexture();
    }
    glUseProgram(m_skymap_shader);
    glUniformMatrix4fv(
        glGetUniformLocation(m_skymap_shader, "uProjectionMatrix"), 1, false,
        value_ptr(proj));
    glUniformMatrix4fv(
        glGetUniformLocation(m_skymap_shader, "uModelViewMatrix"), 1, false,
        value_ptr(view));
    glUniform1f(glGetUniformLocation(m_skymap_shader, "uZDistance"), 1000.0f);
    glActiveTexture(GL_TEXTURE0); // Set the location for binding the texture
    glBindTexture(GL_TEXTURE_2D, tex); // Bind the texture
    glUniform1i(
        glGetUniformLocation(m_skymap_shader, "uSkyMap"),
        0); // Set our sampler (texture0) to use GL_TEXTURE0 as the source
    cgra::draw_dummy(12);
  }

  // draw axis (magically)
  //
  if (m_show_axis) {
    cgra::drawAxis(view, proj);
  }

  // draw the aabb (magically)
  //
  if (m_show_aabb) {
    glUseProgram(m_aabb_shader);
    glUniformMatrix4fv(glGetUniformLocation(m_aabb_shader, "uProjectionMatrix"),
                       1, false, value_ptr(proj));
    glUniformMatrix4fv(glGetUniformLocation(m_aabb_shader, "uModelViewMatrix"),
                       1, false, value_ptr(view));
    glUniform3fv(glGetUniformLocation(m_aabb_shader, "uColor"), 1,
                 value_ptr(vec3(0.8, 0.8, 0.8)));
    glUniform3fv(glGetUniformLocation(m_aabb_shader, "uMax"), 1,
                 value_ptr(m_bound_hsize));
    glUniform3fv(glGetUniformLocation(m_aabb_shader, "uMin"), 1,
                 value_ptr(-m_bound_hsize));
    cgra::draw_dummy(12);
  }

  // draw boids
  //
  for (const Boid &b : m_boids) {

    // get the boid direction (default to z if no velocity)
    vec3 dir = normalize(b.velocity());
    if (dir.x != dir.x)
      dir = vec3(0, 0, 1);

    // calculate the model matrix
    mat4 model(1);

    // rotate the model to point it in the direction of its velocity

    // pitch rotation
    if (dir.y != 0) {
      float angle = -asin(dir.y);
      model = rotate(mat4(1), angle, vec3(1, 0, 0)) * model;
    }

    // yaw rotation
    if (dir.x != 0 || dir.z != 0) {
      float angle = atan2(dir.x, dir.z);
      model = rotate(mat4(1), angle, vec3(0, 1, 0)) * model;
    }

    // translate the model to its worldspace position

    // translate by m_position
    model = translate(mat4(1), b.position()) * model;

    // calculate the modelview matrix
    mat4 modelview = view * model;

    // load shader and variables
    glUseProgram(m_color_shader);
    glUniformMatrix4fv(
        glGetUniformLocation(m_color_shader, "uProjectionMatrix"), 1, false,
        value_ptr(proj));
    glUniformMatrix4fv(glGetUniformLocation(m_color_shader, "uModelViewMatrix"),
                       1, false, value_ptr(modelview));
    glUniform3fv(glGetUniformLocation(m_color_shader, "uColor"), 1,
                 value_ptr(b.color()));

    // draw
    m_simple_boid_mesh.draw();
  }

  // draw obstacles
  //

  if (m_obstacles.size() > 0) {
    drawObstacles(proj, view);
  }
}

void Scene::drawObstacles(const mat4 &proj, const mat4 &view) {
  for (const Obstacle &o : m_obstacles) {
    mat4 model(1);
    model = translate(mat4(1), o.position()) * scale(mat4(1), vec3(o.radius()));
    mat4 modelview = view * model;

    glUseProgram(m_color_shader);
    glUniformMatrix4fv(
        glGetUniformLocation(m_color_shader, "uProjectionMatrix"), 1, false,
        value_ptr(proj));

    glUniformMatrix4fv(glGetUniformLocation(m_color_shader, "uModelViewMatrix"),
                       1, false, value_ptr(modelview));

    // Set the color to red
    glUniform3fv(glGetUniformLocation(m_color_shader, "uColor"), 1,
                 value_ptr(vec3(1.0, 0.0, 0.0)));

    m_sphere_mesh.draw();
  }
}

void Scene::renderGUI() {

  ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing,
                      ImVec2(10, 10)); // Add spacing between items

  if (ImGui::Button("Core", ImVec2(80, 0))) {
    loadCore();
  }
  ImGui::SameLine();
  if (ImGui::Button("Completion", ImVec2(80, 0))) {
    loadCompletion();
  }
  ImGui::SameLine();
  if (ImGui::Button("Challenge", ImVec2(80, 0))) {
    loadChallenge();
  }

  ImGui::Checkbox("Draw Bound", &m_show_aabb);
  ImGui::Checkbox("Draw Axis", &m_show_axis);
  ImGui::Checkbox("Draw Skybox", &m_show_skymap);

  ImGui::Separator();

  ImGui::Text("Core");
  ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, ImVec2(2, 2));

  // General settings
  ImGui::Text("Avoidance Weight");
  ImGui::SliderFloat("##Avoidance Weight", &m_avoidance_weight, 0, 1.0, "%.2f");

  ImGui::Text("Cohesion Weight");
  ImGui::SliderFloat("##Cohesion Weight", &m_cohesion_weight, 0, 1.0, "%.2f");

  ImGui::Text("Alignment Weight");
  ImGui::SliderFloat("##Alignment Weight", &m_alignment_weight, 0, 1.0, "%.2f");

  ImGui::Text("Min Speed");
  ImGui::SliderFloat("##Min Speed", &m_min_speed, 0, 10.0, "%.0f");

  ImGui::Text("Max Speed");
  ImGui::SliderFloat("##Max Speed", &m_max_speed, 1, 10.0, "%.0f");

  ImGui::Text("Boid Neighbour Radius");
  ImGui::SliderFloat("##Boid Neighbour Radius", &m_boid_radius, 0, 10.0,
                     "%.2f");

  ImGui::Text("Soft Bound");
  ImGui::SliderFloat("##Soft Bound", &m_soft_bound, 0, 10.0, "%.2f");

  ImGui::Text("Soft Bound Weight");
  ImGui::SliderFloat("##Soft Bound Weight", &m_soft_bound_weight, 0, 1.0,
                     "%.2f");

  ImGui::Checkbox("Bounce", &m_bounce);
  ImGui::Checkbox("Wrap", &m_wrap);

  ImGui::Separator();
  ImGui::Text("Completion");

  // Flock settings
  ImGui::Text("Number of Flocks");
  ImGui::SliderInt("##Number of Flocks", &m_num_flocks, 0, 10);

  ImGui::Text("Boids per Flock");
  ImGui::SliderInt("##Boids per Flock", &m_num_boids_per_flock, 0, 300);

  // Predator settings
  ImGui::Text("Number of Predators");
  ImGui::SliderInt("##Number of Predators", &m_num_predators, 0, 10);

  ImGui::Text("Predator Avoidance Weight");
  ImGui::SliderFloat("##Predator Avoidance Weight",
                     &m_predator_avoidance_weight, 0, 1.0, "%.2f");

  ImGui::Text("Predator Chase Weight");
  ImGui::SliderFloat("##Predator Chase Weight", &m_predator_chase_weight, 0,
                     1.0, "%.2f");

  ImGui::Separator();
  ImGui::Text("Challenge");

  // Obstacle settings
  ImGui::Text("Number of Obstacles");
  ImGui::SliderInt("##Number of Obstacles", &m_num_obstacles, 0, 10);

  ImGui::Text("Obstacle Radius");
  ImGui::SliderFloat("##Obstacle Radius", &m_obstacle_radius, 0, 10.0, "%.2f");

  ImGui::Text("Obstacle Avoidance Weight");
  ImGui::SliderFloat("##Obstacle Avoidance Weight",
                     &m_obstacle_avoidance_weight, -5.0, 5.0, "%.2f");

  ImGui::PopStyleVar(); // Pop frame padding style
  ImGui::PopStyleVar(); // Pop item spacing style
}