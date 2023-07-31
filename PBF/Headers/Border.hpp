#pragma once

#include <glm/glm.hpp>
#include <Shader.hpp>

static const float vertices0[] = { 10.0f, 0.1f, -5.0f, 10.0f, 4.0f, -5.0f, 10.0f, 0.1f, 5.0f, 10.0f, 4.0f, 5.0f };
static const float vertices1[] = { -10.0f, 0.1f, -5.0f, -10.0f, 4.0f, -5.0f, -10.0f, 0.1f, 5.0f, -10.0f, 4.0f, 5.0f };
static const float vertices2[] = { -5.0f, 0.1f, 0.0f, 5.0f, 0.1f, 0.0f, -5.0f, 4.0f, 0.0f, 5.0f, 4.0f, 0.0f };
static const float vertices3[] = { -5.0f, 0.1f, 0.0f, 5.0f, 0.1f, 0.0f, -5.0f, 4.0f, 0.0f, 5.0f, 4.0f, 0.0f };

class Border
{
public:
	// Constructor
	Border(const unsigned int index, const Shader shader);

	/// <summary>
	/// Render Border
	/// </summary>
	void Render(glm::mat4 viewMat, glm::mat4 projMat);

private:
	const unsigned int index;
	unsigned int VBO, VAO;
	Shader shader;
};