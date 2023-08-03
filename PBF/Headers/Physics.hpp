#pragma once

#include <Particle.hpp>
#include <vector>
#include <glm/glm.hpp>

const float GRAV = -0.980665f;

inline void UpdatePosition(std::vector<Particle>& particles, const float dt)
{
	// TODO: Can be done in SIMD!
	for (int i = 0; i < particles.size(); i++)
	{
		//particles[i].com += particles[i].velocity * dt;
		particles[i].pred_com += particles[i].velocity * dt;
	}
}

inline void UpdateVelocity(std::vector<Particle> &particles, const float dt)
{
	// TODO: Can be done in SIMD!
	for (int i = 0; i < particles.size(); i++)
	{
		particles[i].prev_velocity = particles[i].velocity;
		particles[i].velocity += glm::vec3(0.0f, GRAV, 0.0f) * dt;
	}
}

inline void CheckCollision(std::vector<Particle> particles, float dt)
{
	// TODO: Can be done in SIMD!
	for (int i = 0; i < particles.size(); i++)
	{

	}
}