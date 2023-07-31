#include <Simulation.hpp>

Simulation::Simulation(unsigned int n_particles, unsigned int n_cells, glm::vec3 particle_generation_location, glm::vec3 grid_generation_location, float floor_border, float width_border, float length_border, Mesh* particle_mesh)
	:
	n_particles(n_particles),
	n_cells(n_cells),
	particle_generation_location(particle_generation_location),
	grid_generation_location(grid_generation_location),
	floor_border(floor_border),
	width_border(width_border),
	length_border(length_border),
	particle_mesh(particle_mesh)
{
	GenerateParticles();
	CalcSphereRadius();
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

void Simulation::GenerateGrid()
{
	std::cout << "Generating " << n_cells << " cells..." << std::endl;

	const float x_interval = (2 * length_border) / n_cells;
	const float y_interval = (2 * width_border) / n_cells;

	// TODO: Can be done in SIMD for no reason!
	// Generate Cells
	for (int i = 0; i < n_cells; i++)
	{
		grid.push_back(glm::vec3(grid_generation_location + glm::vec3(glm::vec3(x_interval * i, particle_generation_location.y, particle_generation_location.z))));
	}
}

void Simulation::CheckCollisionSimple()
{
	for (int i = 0; i < n_particles; i++)
	{
		//std::vector<Impulse> impulses;

		// Check floor collision
		if (particles[i].com.y - sphere_radius <= floor_border)
		{
			std::cout << "Floor collision!" << std::endl;
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
			std::cout << "Length collision!" << std::endl;
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
			std::cout << "Width collision!" << std::endl;
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