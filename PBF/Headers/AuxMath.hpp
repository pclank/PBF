#pragma once

#include <glm/glm.hpp>

# define PI 3.14159265358979323846f

// 802.14091318315249227517416739747
//static const float poly6_kernel_const = 802.1409131f;
//// 4812.8454790989149536510450043848
//static const float poly6_gradient_const = 4812.84547909f;
//static const float poly6_radius = 0.5f;
//static const float poly6_radius_squared = 0.25f;

// Kernel Constants
static const float poly6_radius = 5.0f;
static const float poly6_kernel_const = 315 / (64 * PI * powf(poly6_radius, 9));
static const float poly6_gradient_const = -945 / (32 * PI * powf(poly6_radius, 9));
static const float poly6_radius_squared = poly6_radius * poly6_radius;
static const float spiky_gradient_const = -45 / (PI * powf(poly6_radius, 6));

// Artificial Pressure s_corr parameters
//static const glm::vec3 corr_dq(0.06f, 0.04f, 0.05f);
static const glm::vec3 corr_dq(0.0f);
static const float corr_k = 0.001f;
static const unsigned int corr_n = 4;

/// <summary>
/// Calculate Poly6 Kernel of distance vector
/// </summary>
/// <param name="vector">: the distance vector</param>
/// <returns></returns>
inline float CalculatePoly6Kernel(glm::vec3 vector)
{
	const float r = glm::dot(vector, vector);

	// Zero check
	//if (r >= 0.0f && r <= poly6_radius)
	if (r >= 0.0f && r <= poly6_radius_squared)
	{
		return poly6_kernel_const * (poly6_radius_squared - r) * (poly6_radius_squared - r) * (poly6_radius_squared - r);
	}
	else
		return 0.0f;
}

/// <summary>
/// Calculate Poly6 Kernel Gradient (-vector * (945/ 32 * pi * h^9) * (h^2 - r^2)^2)
/// </summary>
/// <param name="vector">: the distance vector</param>
/// <returns></returns>
inline glm::vec3 CalculatePoly6Gradient(glm::vec3 vector)
{
	const float r = glm::dot(vector, vector);

	// Zero check
	//if (r >= 0.0f && r <= poly6_radius)
	if (r >= 0.0f && r <= poly6_radius_squared)
	{
		return vector * poly6_gradient_const * (poly6_radius_squared - r) * (poly6_radius_squared - r);
	}
	else
		return glm::vec3(0.0f);
}

/// <summary>
/// Calculate the Spiky Gradient ( -vector * (45/(pi*h^6) * (h-r)^2) )
/// </summary>
/// <param name="vector">: the distance vector</param>
/// <returns></returns>
inline glm::vec3 CalculateSpikyGradient(glm::vec3 vector)
{
	const float r = glm::dot(vector, vector);

	// Zero check
	//if (r >= 0.0f && r <= poly6_radius)
	if (r >= 0.0f && r <= poly6_radius_squared)
	{
		return vector * spiky_gradient_const * (poly6_radius_squared - r) * (poly6_radius_squared - r);
	}
	else
		return glm::vec3(0.0f);
}

inline float CalculateArtificialPressure(const glm::vec3 vector)
{
	const float ratio = CalculatePoly6Kernel(vector) / CalculatePoly6Kernel(corr_dq);

	// HARDCODED MAGIC!!!
	return -corr_k * (ratio * ratio * ratio * ratio);
}