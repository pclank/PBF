#pragma once

#include <glm/glm.hpp>
#include <iostream>
#include <vector>
#include <Particle.hpp>

class Simulation
{
public:
	const glm::vec3 particle_generation_location;
	const glm::vec3 grid_generation_location;
	const float floor_border;
	const float width_border;
	const float length_border;

	std::vector<Particle> particles;
	std::vector<glm::vec3> grid;

	Simulation(unsigned int n_particles, unsigned int n_cells, glm::vec3 particle_generation_location, glm::vec3 grid_generation_location, float floor_border, float width_border, float length_border);
	~Simulation();

	/// <summary>
	/// Generate Particles according to number
	/// </summary>
	/// <param name="n_particles">: number of particles to generate</param>
	void GenerateParticles();

	/// <summary>
	/// Generate Grid according to number
	/// </summary>
	/// <param name="n_cells">: number of grids to generate</param>
	void GenerateGrid();

private:
	const unsigned int n_particles;
	unsigned int initial_distance;
	const unsigned int n_cells;
	float cell_distance;
};