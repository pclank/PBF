#pragma once

#include <glm/glm.hpp>
#include <glad/glad.h>
#include <iostream>
#include <vector>
#include <Particle.hpp>
#include <Mesh.hpp>
#include <Physics.hpp>
#include <AuxMath.hpp>

static const float MIN_VEL = 1.0f;
static const float REST_DENSITY = 1000.0f;
static const float RELAXATION = 10.0f;
static const float VISCOSITY_C = 0.01f;
static const unsigned int SOLVER_ITER = 3;

typedef std::pair<glm::vec3, float> Impulse;

struct Cell {
	glm::vec2 pos = glm::vec2(0.0f);
	std::vector<unsigned int> neighbors;
};

// Hardcore Color-table for 30 cell - Visual Debugging
static const float ColorTable[] = {
	0.1f, 0.1f, 0.5f,
	0.2f, 0.1f, 0.5f,
	0.3f, 0.1f, 0.5f,
	0.4f, 0.4f, 0.5f,
	0.5f, 0.1f, 0.5f,
	0.6f, 0.3f, 0.5f,
	0.7f, 0.1f, 0.5f,
	0.8f, 0.1f, 0.5f,
	0.9f, 0.6f, 0.5f,
	1.0f, 0.1f, 0.5f,
	0.1f, 0.1f, 0.9f,
	0.2f, 0.9f, 0.9f,
	0.3f, 0.8f, 0.9f,
	0.4f, 0.7f, 0.9f,
	0.5f, 0.4f, 0.9f,
	0.6f, 0.5f, 0.9f,
	0.7f, 0.3f, 0.9f,
	0.7f, 0.6f, 0.9f,
	0.7f, 0.2f, 0.9f,
	0.7f, 0.1f, 0.9f,
	0.7f, 0.0f, 0.9f,
	0.7f, 0.9f, 0.9f,
	0.7f, 0.8f, 0.9f,
	0.2f, 0.9f, 0.9f,
	0.1f, 0.35f, 0.9f,
	0.0f, 0.3f, 0.9f,
	0.6f, 0.25f, 0.9f,
	0.8f, 0.5f, 0.9f,
	0.9f, 0.5f, 0.9f,
	1.0f, 0.5f, 0.9f
};

class Simulation
{
public:
	unsigned int n_particles;
	unsigned int n_cells;
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
	std::vector<Cell> grid;
	std::map<unsigned int, unsigned int> cell_map;				// Maps bit indices to actual vector indices of grid

	bool sim_running = false;
	bool step_run = false;										// Run simulation step by step

	Simulation(unsigned int n_particles, float cell_size, glm::vec3 particle_generation_location, glm::vec3 grid_generation_location, float floor_border, float width_border, float length_border, float gen_interval, bool distance_gen, Mesh* particle_mesh);
	~Simulation();

	/// <summary>
	/// Handles simulation update per frame
	/// </summary>
	/// <param name="dt">: delta time</param>
	void TickSimulation(const float dt);

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
	/// Checks for Collision in a stupid way
	/// </summary>
	void CheckCollisionSimple();

	/// <summary>
	/// Handle border collision in the simplest way
	/// </summary>
	void StupidBorderCollision();

	/// <summary>
	/// Checks for collisions between the particles
	/// </summary>
	void ParticleCollisionDetection();

	/// <summary>
	/// Calculate Sphere radius for collision detection
	/// </summary>
	inline void CalcSphereRadius()
	{
		sphere_radius = std::abs(glm::distance(particle_mesh->m_vertices[0].position, glm::vec3(0.0f)));
		std::cout << "Radius was " << sphere_radius << std::endl;
	}

	/// <summary>
	/// Assign Particles to neighborhoods
	/// </summary>
	void FindNeighbors();

	/// <summary>
	/// SPH Density Estimator
	/// </summary>
	/// <param name="p1">: the particle</param>
	/// <returns>: the density</returns>
	inline float EstimateDensity(const Particle p1)
	{
		float density = 0.0f;
		for (int i = 0; i < grid[cell_map[p1.cell]].neighbors.size(); i++)
		{
			if (p1.id == particles[grid[cell_map[p1.cell]].neighbors[i]].id)
				continue;

			//density += CalculatePoly6Kernel(p1.com - particles[grid[cell_map[p1.cell]].neighbors[i]].com);
			density += CalculatePoly6Kernel(p1.pred_com - particles[grid[cell_map[p1.cell]].neighbors[i]].pred_com);
		}

		return density;
	}

	/// <summary>
	/// Calculates the density constraint of a particle
	/// </summary>
	/// <param name="p1">: the particle</param>
	/// <returns>: the density constraint</returns>
	inline float CalculateDensityConstraint(const Particle p1)
	{
		return (EstimateDensity(p1) / REST_DENSITY) - 1;
	}

	/// <summary>
	/// Calculates the lambda of a particle
	/// </summary>
	/// <param name="p1">: the particle</param>
	/// <returns>: the lambda</returns>
	inline float CalculateLambda(const Particle p1)
	{
		float denominator = 0.0f;

		// Every particle k in neighborhood
		for (int i = 0; i < grid[cell_map[p1.cell]].neighbors.size(); i++)
		{
			if (p1.id == particles[grid[cell_map[p1.cell]].neighbors[i]].id)
			{
				glm::vec3 gradient(0.0f);
				for (int j = 0; j < grid[cell_map[p1.cell]].neighbors.size(); j++)
				{
					//gradient += CalculatePoly6Gradient(p1.com - particles[grid[cell_map[p1.cell]].neighbors[i]].com);
					//gradient += CalculatePoly6Gradient(p1.pred_com - particles[grid[cell_map[p1.cell]].neighbors[i]].pred_com);
					gradient += CalculateSpikyGradient(p1.pred_com - particles[grid[cell_map[p1.cell]].neighbors[i]].pred_com);
				}

				denominator += glm::dot(gradient / REST_DENSITY, gradient / REST_DENSITY);
			}
			else
			{
				// TODO: Make sure dot product is okay!
				//const glm::vec3 gradient = CalculatePoly6Gradient(p1.com - particles[grid[cell_map[p1.cell]].neighbors[i]].com) / REST_DENSITY;
				//const glm::vec3 gradient = CalculatePoly6Gradient(p1.pred_com - particles[grid[cell_map[p1.cell]].neighbors[i]].pred_com) / REST_DENSITY;
				const glm::vec3 gradient = CalculateSpikyGradient(p1.pred_com - particles[grid[cell_map[p1.cell]].neighbors[i]].pred_com) / REST_DENSITY;

				denominator -= glm::dot(gradient, gradient);
			}
		}

		return -CalculateDensityConstraint(p1) / (denominator + RELAXATION);
	}

	/// <summary>
	/// Calculate the Position Update dp of a particle
	/// </summary>
	/// <param name="p1">: the particle</param>
	/// <returns>: vector dp</returns>
	inline glm::vec3 CalculatePositionUpdate(const Particle p1)
	{
		glm::vec3 dp(0.0f);
		for (int i = 0; i < grid[cell_map[p1.cell]].neighbors.size(); i++)
		{
			//const glm::vec3 distance_vector = p1.com - particles[grid[cell_map[p1.cell]].neighbors[i]].com;
			const glm::vec3 distance_vector = p1.pred_com - particles[grid[cell_map[p1.cell]].neighbors[i]].pred_com;
			const float first_factor = p1.lambda + particles[grid[cell_map[p1.cell]].neighbors[i]].lambda + CalculateArtificialPressure(distance_vector);

			//dp += first_factor * CalculatePoly6Gradient(distance_vector);
			dp += first_factor * CalculateSpikyGradient(distance_vector);
		}

		return dp / REST_DENSITY;
	}

	/// <summary>
	/// Calculate XSPH Viscosity for a particle
	/// </summary>
	/// <param name="p1">: the particle</param>
	/// <returns>: the viscosity vector</returns>
	inline glm::vec3 CalculateXSPHViscosity(const Particle p1)
	{
		glm::vec3 sum(0.0f);
		for (int i = 0; i < grid[cell_map[p1.cell]].neighbors.size(); i++)
		{
			//const glm::vec3 distance_vector = p1.com - particles[grid[cell_map[p1.cell]].neighbors[i]].com;
			const glm::vec3 distance_vector = p1.pred_com - particles[grid[cell_map[p1.cell]].neighbors[i]].pred_com;
			sum += (particles[grid[cell_map[p1.cell]].neighbors[i]].velocity - p1.velocity) * CalculatePoly6Kernel(distance_vector);
		}

		return p1.velocity + VISCOSITY_C * sum;
	}
};