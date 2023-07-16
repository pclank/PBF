#pragma once

#include <glm/glm.hpp>

/// <summary>
/// Implements particle objects
/// </summary>
struct Particle
{
	glm::vec3 com;					// Position of COM
	glm::vec3 velocity;				// Velocity of COM
	unsigned int grid;				// Grid the particle belongs to
};

