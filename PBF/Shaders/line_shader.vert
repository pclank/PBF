#version 430

layout(location = 0) in vec3 position;

out vec3 TestColor;

uniform mat4 viewMatrix;
uniform mat4 projectionMatrix;

void main()
{
    gl_Position = projectionMatrix * viewMatrix * vec4(position, 1.0);
    TestColor = vec3(0.1, 1.0, 1.0);
}
