#pragma once

#define NOMINMAX
#include <glm/glm.hpp>
#include <glad/glad.h>
#include <iostream>
#include <vector>
#include <Particle.hpp>
#include <Mesh.hpp>
#include <Physics.hpp>
#include <AuxMath.hpp>
#include <omp.h>

#define PARALLEL
#define NEIGH_PARALLEL
#define IGNORE_SELF
#define CONSTRAIN_CELLS
#define SPIKY_GRAD
#define NULLIFY_VELOCITY
//#define VORTICITY
#define VISCOSITY

static const float MIN_VEL = 1.0f;
static const float REST_DENSITY = 1000.0f;
static const float RELAXATION = 10.0f;
static const float VISCOSITY_C = 0.01f;
static const float FLUID_DENSITY = 800.0f;
//static const float FLUID_DENSITY = REST_DENSITY;
static const float FLUID_DENSITY_SQUARED = FLUID_DENSITY * FLUID_DENSITY;
static const float VORTICITY_COEFF = 1.0f;
static const unsigned int SOLVER_ITER = 3;
static const int GRID_HEIGHT = 5;
static const float BORDER_COLLISION_INTERVAL = 0.1f;
static const float smooth_factor = 1.0f;

typedef std::pair<glm::vec3, float> Impulse;

struct Cell {
	glm::vec3 pos = glm::vec3(0.0f);
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
	float height_border;
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
	inline float EstimateDensity(const Particle& p1)
	{
		float density = 0.0f;
		//#pragma omp parallel for reduction (+:density)
		for (int i = 0; i < grid[cell_map[p1.cell]].neighbors.size(); i++)
		{
			if (p1.id == grid[cell_map[p1.cell]].neighbors[i])
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
	inline float CalculateDensityConstraint(const Particle& p1)
	{
		return (EstimateDensity(p1) / REST_DENSITY) - 1.0f;
	}

	/// <summary>
	/// Calculates the lambda of a particle
	/// </summary>
	/// <param name="p1">: the particle</param>
	/// <returns>: the lambda</returns>
	inline float CalculateLambda(const Particle& p1)
	{
		float denominator = 0.0f;

		// Every particle k in neighborhood
		//#pragma omp parallel for reduction (+:denominator)
		for (int i = 0; i < grid[cell_map[p1.cell]].neighbors.size(); i++)
		{
			if (p1.id == grid[cell_map[p1.cell]].neighbors[i])
			{
				glm::vec3 gradient(0.0f);
				for (int j = 0; j < grid[cell_map[p1.cell]].neighbors.size(); j++)
				{
					//gradient += CalculatePoly6Gradient(p1.com - particles[grid[cell_map[p1.cell]].neighbors[i]].com);
#ifndef SPIKY_GRAD
					gradient += CalculatePoly6Gradient(p1.pred_com - particles[grid[cell_map[p1.cell]].neighbors[i]].pred_com);
#else
					gradient += CalculateSpikyGradient(p1.pred_com - particles[grid[cell_map[p1.cell]].neighbors[i]].pred_com);
#endif // !SPIKY_GRAD
				}

				denominator += glm::dot(gradient / REST_DENSITY, gradient / REST_DENSITY);
			}
			else
			{
				// TODO: Make sure dot product is okay!
				//const glm::vec3 gradient = CalculatePoly6Gradient(p1.com - particles[grid[cell_map[p1.cell]].neighbors[i]].com) / REST_DENSITY;
#ifndef SPIKY_GRAD
				const glm::vec3 gradient = CalculatePoly6Gradient(p1.pred_com - particles[grid[cell_map[p1.cell]].neighbors[i]].pred_com) / REST_DENSITY;
#else
				const glm::vec3 gradient = CalculateSpikyGradient(p1.pred_com - particles[grid[cell_map[p1.cell]].neighbors[i]].pred_com) / REST_DENSITY;
#endif // !SPIKY_GRAD

				denominator -= glm::dot(gradient, gradient);
			}
		}

		return -CalculateDensityConstraint(p1) / (denominator + RELAXATION);
	}

	inline float CalculateLambda2(const Particle& p1)
	{
		float sum_grad_C2 = 0.0f;
		glm::vec3 gradC_i(0.0f);

		//const float C = CalculateDensityConstraint(p1);
		const float C = std::max(CalculateDensityConstraint(p1) - 1.0f, 0.0f);			// clamp to prevent particle clumping at surface

		if (C == 0.0f)
		{
			return 0.0f;
		}

		// Every particle j in neighborhood
		for (int j = 0; j < grid[cell_map[p1.cell]].neighbors.size(); j++)
		{
#ifdef IGNORE_SELF
			if (p1.id == grid[cell_map[p1.cell]].neighbors[j])
				continue;
#endif

#ifndef SPIKY_GRAD
			const glm::vec3 gradC_j = -(float)grid[cell_map[p1.cell]].neighbors.size() * CalculatePoly6Gradient(p1.pred_com - particles[grid[cell_map[p1.cell]].neighbors[j]].pred_com);
#else
			const glm::vec3 gradC_j = -(float)grid[cell_map[p1.cell]].neighbors.size() * CalculateSpikyGradient(p1.pred_com - particles[grid[cell_map[p1.cell]].neighbors[j]].pred_com);
#endif // !SPIKY_GRAD
			
			sum_grad_C2 += glm::dot(gradC_j / REST_DENSITY, gradC_j / REST_DENSITY);
			gradC_i -= gradC_j / REST_DENSITY;
			//sum_grad_C2 += glm::dot(gradC_j, gradC_j);
			//gradC_i -= gradC_j;
		}

		sum_grad_C2 += glm::dot(gradC_i, gradC_i);

		return -C / (sum_grad_C2 + RELAXATION);
	}

	/// <summary>
	/// Calculate the Position Update dp of a particle
	/// </summary>
	/// <param name="p1">: the particle</param>
	/// <returns>: vector dp</returns>
	inline glm::vec3 CalculatePositionUpdate(const Particle& p1)
	{
		glm::vec3 dp(0.0f);
		for (int i = 0; i < grid[cell_map[p1.cell]].neighbors.size(); i++)
		{
#ifdef IGNORE_SELF
			if (p1.id == grid[cell_map[p1.cell]].neighbors[i])
				continue;
#endif

			//const glm::vec3 distance_vector = p1.com - particles[grid[cell_map[p1.cell]].neighbors[i]].com;
			const glm::vec3 distance_vector = p1.pred_com - particles[grid[cell_map[p1.cell]].neighbors[i]].pred_com;
			const float first_factor = p1.lambda + particles[grid[cell_map[p1.cell]].neighbors[i]].lambda + CalculateArtificialPressure(distance_vector);
			
#ifndef SPIKY_GRAD
			dp += first_factor * CalculatePoly6Gradient(distance_vector);
#else
			dp += first_factor * CalculateSpikyGradient(distance_vector);
#endif // !SPIKY_GRAD

		}

		return dp / REST_DENSITY;
	}

	/// <summary>
	/// Calculate XSPH Viscosity for a particle
	/// </summary>
	/// <param name="p1">: the particle</param>
	/// <returns>: the viscosity vector</returns>
	inline glm::vec3 CalculateXSPHViscosity(const Particle& p1)
	{
		glm::vec3 sum(0.0f);
		for (int i = 0; i < grid[cell_map[p1.cell]].neighbors.size(); i++)
		{
#ifdef IGNORE_SELF
			if (p1.id == grid[cell_map[p1.cell]].neighbors[i])
				continue;
#endif

			//const glm::vec3 distance_vector = p1.com - particles[grid[cell_map[p1.cell]].neighbors[i]].com;
			const glm::vec3 distance_vector = p1.pred_com - particles[grid[cell_map[p1.cell]].neighbors[i]].pred_com;
			sum += (particles[grid[cell_map[p1.cell]].neighbors[i]].prev_velocity - p1.prev_velocity) * CalculatePoly6Kernel(distance_vector);
		}

		return p1.prev_velocity + VISCOSITY_C * sum;
	}

	inline void CalculateVorticityForce()
	{
		std::vector<glm::vec3> omegas;
		omegas.reserve(n_particles);

		// For all particles calculate vorticity
#ifdef PARALLEL
		#pragma omp parallel for
#endif
		for (int i = 0; i < n_particles; i++)
		{
			glm::vec3 omega(0.0f);
			for (int j = 0; j < grid[cell_map[particles[i].cell]].neighbors.size(); j++)
			{
#ifdef IGNORE_SELF
				if (i == grid[cell_map[particles[i].cell]].neighbors[j])
					continue;
#endif

#ifndef SPIKY_GRAD
				const glm::vec3 gradient = CalculatePoly6Gradient(particles[i].pred_com - particles[grid[cell_map[particles[i].cell]].neighbors[j]].pred_com);
#else
				const glm::vec3 gradient = CalculateSpikyGradient(particles[i].pred_com - particles[grid[cell_map[particles[i].cell]].neighbors[j]].pred_com);
#endif // !SPIKY_GRAD

				omega -= (grid[cell_map[particles[i].cell]].neighbors.size() / FLUID_DENSITY) * glm::cross(particles[i].velocity - particles[grid[cell_map[particles[i].cell]].neighbors[j]].velocity, gradient);
			}

			omegas.push_back(omega);
		}

		// For all particles calculate eta
#ifdef PARALLEL
		#pragma omp parallel for
#endif
		for (int i = 0; i < n_particles; i++)
		{
			glm::vec3 eta(0.0f);
			for (int j = 0; j < grid[cell_map[particles[i].cell]].neighbors.size(); j++)
			{
#ifdef IGNORE_SELF
				if (i == grid[cell_map[particles[i].cell]].neighbors[j])
					continue;
#endif

#ifndef SPIKY_GRAD
				const glm::vec3 gradient = CalculatePoly6Gradient(particles[i].pred_com - particles[grid[cell_map[particles[i].cell]].neighbors[j]].pred_com);
#else
				const glm::vec3 gradient = CalculateSpikyGradient(particles[i].pred_com - particles[grid[cell_map[particles[i].cell]].neighbors[j]].pred_com);
#endif // !SPIKY_GRAD

				eta += (grid[cell_map[particles[i].cell]].neighbors.size() / FLUID_DENSITY) * glm::normalize(omegas[i]) * gradient;
			}

			eta = glm::normalize(eta);

			particles[i].vorticity_force = VORTICITY_COEFF * glm::cross(eta, omegas[i]);
		}
	}
};