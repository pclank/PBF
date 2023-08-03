#include <Simulation.hpp>

//#define DEBUG

Simulation::Simulation(unsigned int n_particles, float cell_distance, glm::vec3 particle_generation_location, glm::vec3 grid_generation_location, float floor_border, float width_border, float length_border, float gen_interval, bool distance_gen, Mesh* particle_mesh)
	:
	n_particles(n_particles),
	cell_distance(cell_distance),
	particle_generation_location(particle_generation_location),
	grid_generation_location(grid_generation_location),
	floor_border(floor_border),
	width_border(width_border),
	length_border(length_border),
	generation_distance_interval(gen_interval),
	particle_mesh(particle_mesh)
{
	if (!distance_gen)
		GenerateParticles();
	else
		GenerateParticles2();

	GenerateGrid();
	CalcSphereRadius();

	FindNeighbors();
}

Simulation::~Simulation()
{

}

void Simulation::GenerateParticles()
{
	std::cout << "Generating " << n_particles << " particles..." << std::endl;
	
	const float x_interval = (2 * length_border) / n_particles;
	const float z_interval = (2 * width_border) / n_particles;

	// TODO: Can be done in SIMD for no reason!
	// Generate Particles
	for (int i = 0; i < n_particles; i++)
	{
		Particle new_particle;
		new_particle.com = particle_generation_location + glm::vec3(x_interval * i, particle_generation_location.y, particle_generation_location.z);

		particles.push_back(new_particle);
	}
}

void Simulation::GenerateParticles2()
{
	const glm::vec3 starting_location(-9.5f, 1.0f, -4.5f);

	const float x_limit = length_border / generation_distance_interval;
	const float z_limit = width_border / generation_distance_interval;
	const float y_limit = 4.0f / generation_distance_interval;

	// Length
	int particle_cnt = 0;
	for (int x = 0; x < x_limit; x++)
	{
		// Width
		for (int z = 0; z < z_limit; z++)
		{
			// Height
			for (int y = 0; y < y_limit; y++)
			{
				glm::vec3 new_location = starting_location + glm::vec3(x, y, z) * generation_distance_interval;

				Particle new_particle;
				new_particle.com = new_location;
				new_particle.id = particle_cnt;

				particle_cnt++;

				particles.push_back(new_particle);
			}
		}
	}

	n_particles = particle_cnt;

	std::cout << "Generated " << particle_cnt << " particles..." << std::endl;
}

void Simulation::GenerateGrid()
{
	const int x_limit = 2 * length_border / cell_distance;
	const int z_limit = 2 * width_border / cell_distance;
	const int y_limit = 4.0f / cell_distance;

	// Length
	int cell_cnt = 0;
	for (int x = 0; x < x_limit; x++)
	{
		// Width
		for (int z = 0; z < z_limit; z++)
		{
			// Height
			for (int y = 0; y < y_limit; y++)
			{
				glm::vec3 new_location = grid_generation_location + glm::vec3(x, y, z) * cell_distance;

				Cell new_cell;
				new_cell.pos = new_location;

				const unsigned int x_cell = std::floor((new_location.x - grid_generation_location.x) / cell_distance);
				const unsigned int z_cell = std::floor((new_location.z - grid_generation_location.z) / cell_distance);
				const unsigned int y_cell = std::floor((new_location.y - grid_generation_location.y) / cell_distance);

				const unsigned int cell_id = x_cell + (z_cell << 8) + (y_cell << 16);
				cell_map[cell_id] = cell_cnt;
				grid.push_back(new_cell);

				cell_cnt++;
			}
		}
	}

	n_cells = cell_cnt;

	std::cout << "Generated " << cell_cnt << " cells..." << std::endl;
}

void Simulation::TickSimulation(const float dt)
{
	// Solver loop
	unsigned int iter = 0;
	while (iter < SOLVER_ITER)
	{
		// Calculate Lambda
		for (int i = 0; i < n_particles; i++)
			particles[i].lambda = CalculateLambda(particles[i]);

		// Calculate the Position Update
		for (int i = 0; i < n_particles; i++)
			particles[i].dp = CalculatePositionUpdate(particles[i]);

		// Perform Collision Detection
		//CheckCollisionSimple();
		StupidBorderCollision();

		// Update Positions
		for (int i = 0; i < n_particles; i++)
		{
			particles[i].pred_com += particles[i].dp;
			if (particles[i].pred_com.y == 0xffffffff)
			{
				std::cout << "ERROR!" << std::endl;
			}
		}

		iter++;
	}

	// Update Particle Data
	for (int i = 0; i < n_particles; i++)
	{
		// Update velocity including XSPH Viscosity
		particles[i].velocity = CalculateXSPHViscosity(particles[i]) + (particles[i].pred_com - particles[i].com) / dt;

		// Update position
		particles[i].com = particles[i].pred_com;
		if (particles[i].com.y < 0.0f || particles[i].com.y > 10.0f)
			particles[i].com.y = 1.0f;
		//std::cout << "Particle " << i << ": " << particles[i].com.x << " | " << particles[i].com.y << " | " << particles[i].com.z << std::endl;
	}

	//ParticleCollisionDetection();
}

void Simulation::CheckCollisionSimple()
{
	for (int i = 0; i < n_particles; i++)
	{
		//std::vector<Impulse> impulses;

		// Check floor collision
		if (particles[i].com.y - sphere_radius <= floor_border)
		{
#ifdef DEBUG
			std::cout << "Floor collision!" << std::endl;
#endif // DEBUG
			const float depth = floor_border - particles[i].com.y - sphere_radius;

			static const glm::vec3 norm(0.0f, 1.0f, 0.0f);

			float custom_cor = cor;
			if (std::abs(particles[i].velocity.y) <= MIN_VEL)
			{
				custom_cor = 0.0f;
			}

			const float impulse_magnitude = (-(1 + custom_cor) * glm::dot(particles[i].velocity, norm)) / glm::dot(norm, norm);

			particles[i].velocity += impulse_magnitude * norm;
		}

		// Check length border collision
		if (particles[i].com.z - sphere_radius >= length_border || particles[i].com.z - sphere_radius <= -length_border)
		{
#ifdef DEBUG
			std::cout << "Length collision!" << std::endl;
#endif // DEBUG
			float depth = length_border - particles[i].com.z - sphere_radius;

			// Find side
			glm::vec3 norm;
			if (particles[i].com.z - sphere_radius >= length_border)
				norm = glm::vec3(0.0f, 0.0f, -1.0f);
			else
				norm = glm::vec3(0.0f, 0.0f, 1.0f);

			const float impulse_magnitude = (-(1 + cor) * glm::dot(particles[i].velocity, norm)) / glm::dot(norm, norm);

			particles[i].velocity += impulse_magnitude * norm;
		}

		// Check width border collision
		if (particles[i].com.x - sphere_radius >= width_border || particles[i].com.x - sphere_radius <= -width_border)
		{
#ifdef DEBUG
			std::cout << "Width collision!" << std::endl;
#endif // DEBUG
			float depth = width_border - particles[i].com.x - sphere_radius;

			glm::vec3 norm;
			if (particles[i].com.x - sphere_radius >= width_border)
				norm = glm::vec3(-1.0f, 0.0f, 0.0f);
			else
				norm = glm::vec3(1.0f, 0.0f, 0.0f);

			const float impulse_magnitude = (-(1 + cor) * glm::dot(particles[i].velocity, norm)) / glm::dot(norm, norm);

			particles[i].velocity += impulse_magnitude * norm;
		}
	}
}

void Simulation::StupidBorderCollision()
{
	for (int i = 0; i < n_particles; i++)
	{
		// Check floor collision
		if (particles[i].pred_com.y - sphere_radius < floor_border)
		{
			particles[i].pred_com.y = floor_border + sphere_radius;
		}

		if (particles[i].pred_com.z - sphere_radius > length_border)
		{
			particles[i].pred_com.z = length_border - sphere_radius;
		}
		else if (particles[i].pred_com.z - sphere_radius < -length_border)
		{
			particles[i].pred_com.z = -length_border + sphere_radius;
		}

		if (particles[i].com.x - sphere_radius > width_border)
		{
			particles[i].pred_com.x = width_border - sphere_radius;
		}
		else if (particles[i].com.x - sphere_radius < -width_border)
		{
			particles[i].pred_com.x = -width_border + sphere_radius;
		}
	}
}

void Simulation::ParticleCollisionDetection()
{
	// Check all particles exhaustively
	for (int i = 0; i < n_particles; i++)
	{
		for (int j = 0; j < n_particles; j++)
		{
			// Skip same particles
			if (i == j)
				continue;

			const glm::vec3 s = particles[i].com - particles[j].com;
			const float sq_len = glm::dot(s, s);

			static const float cmp_radius = (2 * sphere_radius) * (2 * sphere_radius);
			
			// Check for collision
			if (sq_len <= cmp_radius)
			{
				glm::vec3 norm = glm::normalize(-s);

				glm::vec3 closure_velocity = particles[i].velocity - particles[j].velocity;

				float custom_cor = cor;
				if (std::abs(closure_velocity.length()) <= 0.0001f)
				{
					custom_cor = 0.0f;
				}

				const float impulse_magnitude = (-(1 + custom_cor) * glm::dot(particles[i].velocity, norm)) / glm::dot(norm, norm * 2.0f);

				particles[i].velocity += impulse_magnitude * norm;
				particles[j].velocity -= impulse_magnitude * norm;
			}
		}
	}
}

void Simulation::RandomWind(float force)
{
	for (int i = 0; i < n_particles; i++)
	{
		if (rand() % 100 < 20)
		{
			particles[i].velocity += force * glm::normalize(glm::vec3(rand() % 100, rand() % 100, rand() % 100));
		}
	}
}

void Simulation::FindNeighbors()
{
	// TODO: Figure this out!
	// Clear neighborhoods
	for (int i = 0; i < n_cells; i++)
		grid[i].neighbors.clear();

	// Assign Particles to Cells
	for (int i = 0; i < n_particles; i++)
	{
		const unsigned int x_cell = std::floor((particles[i].com.x - grid_generation_location.x) / cell_distance);
		const unsigned int z_cell = std::floor((particles[i].com.z - grid_generation_location.z) / cell_distance);
		const unsigned int y_cell = std::floor((particles[i].com.y - grid_generation_location.y) / cell_distance);

		particles[i].cell = x_cell + (z_cell << 8) + (y_cell << 16);

		// Assign to Neighborhood
		grid[cell_map[particles[i].cell]].neighbors.push_back(i);

#ifdef DEBUG
		std::cout << "Particle " << i << " in cell " << particles[i].cell << std::endl;
#endif // DEBUG
	}

#ifdef DEBUG
	// Print Neighborhoods
	for (int i = 0; i < n_cells; i++)
	{
		std::cout << "Cell " << i << ": " << std::endl;
		for (int j = 0; j < grid[i].neighbors.size(); j++)
			std::cout << grid[i].neighbors[j] << " ";
		std::cout << "\n" << std::endl;
	}
#endif // DEBUG

}