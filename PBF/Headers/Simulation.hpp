#pragma once

#include <glm/glm.hpp>
#include <glad/glad.h>
#include <iostream>
#include <vector>
#include <Particle.hpp>
#include <Mesh.hpp>

static const float MIN_VEL = 0.9f;

typedef std::pair<glm::vec3, float> Impulse;

class Simulation
{
public:
	unsigned int n_particles;
	const unsigned int n_cells;
	const glm::vec3 particle_generation_location;
	const glm::vec3 grid_generation_location;
	const float floor_border;
	const float width_border;
	const float length_border;
	const float generation_distance_interval;
	float sphere_radius;
	const float cor = 1.0f;

	const Mesh* particle_mesh;

	std::vector<Particle> particles;
	std::vector<glm::vec3> grid;

	Simulation(unsigned int n_particles, unsigned int n_cells, glm::vec3 particle_generation_location, glm::vec3 grid_generation_location, float floor_border, float width_border, float length_border, float gen_interval, bool distance_gen, Mesh* particle_mesh);
	~Simulation();

	/// <summary>
	/// Checks for Collision in a stupid way
	/// </summary>
	void CheckCollisionSimple();

	/// <summary>
	/// Function to add random wind effect
	/// </summary>
	/// <param name="force">: amount of wind</param>
	void RandomWind(float force);

private:
	unsigned int initial_distance;
	float cell_distance;

	/// <summary>
	/// Generate Particles according to number
	/// </summary>
	void GenerateParticles();

	/// <summary>
	/// Generate Particles according to interval between them
	/// </summary>
	void GenerateParticles2();

	/// <summary>
	/// Generate Grid according to number
	/// </summary>
	void GenerateGrid();

	/// <summary>
	/// Calculate Sphere radius for collision detection
	/// </summary>
	inline void CalcSphereRadius()
	{
		//sphere_radius = std::abs(glm::distance(particle_mesh->m_vertices[0].position, particles[0].com));
		sphere_radius = std::abs(glm::distance(particle_mesh->m_vertices[0].position, glm::vec3(0.0f)));
		std::cout << "Radius was " << sphere_radius << std::endl;
	}
};