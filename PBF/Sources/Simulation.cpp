#include <Simulation.hpp>

Simulation::Simulation(unsigned int n_particles, unsigned int n_cells, glm::vec3 particle_generation_location, glm::vec3 grid_generation_location, float floor_border, float width_border, float length_border)
	:
	n_particles(n_particles),
	n_cells(n_cells),
	particle_generation_location(particle_generation_location),
	grid_generation_location(grid_generation_location),
	floor_border(floor_border),
	width_border(width_border),
	length_border(length_border)
{
	GenerateParticles();
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