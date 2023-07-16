#pragma once

#include <glm/glm.hpp>

/// <summary>
/// Implements particle objects
/// </summary>
struct Particle
{
	glm::vec3 com;							// Position of COM
	glm::vec3 velocity = glm::vec3(0.0f);	// Velocity of COM
	int grid = -1;							// Grid the particle belongs to
};
