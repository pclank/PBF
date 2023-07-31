#include <Border.hpp>

Border::Border(const unsigned int index, const Shader shader)
	:
	index(index),
	shader(shader)
{
	glGenVertexArrays(1, &VAO);
	glGenBuffers(1, &VBO);
	glBindVertexArray(VAO);
	glBindBuffer(GL_ARRAY_BUFFER, VBO);
	switch (index)
	{
	case 0:
		glBufferData(GL_ARRAY_BUFFER, sizeof(vertices0), &vertices0, GL_STATIC_DRAW);
		break;
	case 1:
		glBufferData(GL_ARRAY_BUFFER, sizeof(vertices1), &vertices1, GL_STATIC_DRAW);
		break;
	case 2:
		glBufferData(GL_ARRAY_BUFFER, sizeof(vertices2), &vertices2, GL_STATIC_DRAW);
		break;
	case 3:
		glBufferData(GL_ARRAY_BUFFER, sizeof(vertices3), &vertices3, GL_STATIC_DRAW);
		break;
	default:
		glBufferData(GL_ARRAY_BUFFER, sizeof(vertices0), &vertices0, GL_STATIC_DRAW);
		break;
	}
	glEnableVertexAttribArray(0);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
	glBindVertexArray(0);
}

void Border::Render(glm::mat4 viewMat, glm::mat4 projMat)
{
    shader.use();
    shader.setMat4("viewMatrix", viewMat);
    shader.setMat4("projectionMatrix", projMat);

    glBindVertexArray(VAO);
    glDrawArrays(GL_LINES, 0, 12);
}