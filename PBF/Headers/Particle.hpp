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
	unsigned int id = 0xFFFFFFFF;
	unsigned int cell = 0xFFFFFFFF;
	float lambda = 0.0f;
};
