#pragma once

#include <glm/glm.hpp>

/// <summary>
/// Implements particle objects
/// </summary>
struct Particle
{
	glm::vec3 com;								// Position of COM
	glm::vec3 prev_com;							// Previous COM
	glm::vec3 velocity = glm::vec3(0.0f);		// Velocity of COM
	glm::vec3 prev_velocity = glm::vec3(0.0f);	// Previous Velocity
	int grid = -1;								// Grid the particle belongs to
	bool collided = false;						// Whether particle collided with walls
};
