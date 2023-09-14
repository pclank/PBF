#version 430

layout(location = 0) in vec3 position;
layout(location = 1) in vec3 normal;
layout(location = 2) in vec2 texCoords;
layout(location = 3) in vec3 tangent;
layout(location = 7) in vec3 transformation;

out vec3 FragPos;
out vec3 Normal;
out vec3 WorldPos;
out vec2 TexCoords;
out vec3 Tangent;

uniform mat4 viewMatrix;
uniform mat4 modelMatrix;
uniform mat4 projectionMatrix;

void main()
{
    mat4 newModelMatrix = modelMatrix;
//    newModelMatrix[0][0] = 1.0;
//    newModelMatrix[1][1] = 1.0;
//    newModelMatrix[2][2] = 1.0;
//    newModelMatrix[3][3] = 1.0;

//    newModelMatrix[3][0] = modelMatrix[0][0] * transformation[0] + modelMatrix[1][0] * transformation[1] + modelMatrix[2][0] * transformation[2] + modelMatrix[3][0];
//    newModelMatrix[3][1] = modelMatrix[0][1] * transformation[0] + modelMatrix[1][1] * transformation[1] + modelMatrix[2][1] * transformation[2] + modelMatrix[3][1];
//    newModelMatrix[3][2] = modelMatrix[0][2] * transformation[0] + modelMatrix[1][2] * transformation[1] + modelMatrix[2][2] * transformation[2] + modelMatrix[3][2];
//    newModelMatrix[3][3] = modelMatrix[0][3] * transformation[0] + modelMatrix[1][3] * transformation[1] + modelMatrix[2][3] * transformation[2] + modelMatrix[3][3];

//    gl_Position = projectionMatrix * viewMatrix * newModelMatrix * vec4(position, 1.0);
//    FragPos = vec3(newModelMatrix * vec4(position, 1.0));
//    Normal = mat3(transpose(inverse(newModelMatrix))) * normal;
//    Tangent = mat3(transpose(inverse(newModelMatrix))) * tangent;
//    WorldPos = vec3(newModelMatrix);
    gl_Position = projectionMatrix * viewMatrix * modelMatrix * vec4(position + transformation, 1.0);
    FragPos = vec3(modelMatrix * vec4(position + transformation, 1.0));
    Normal = mat3(transpose(inverse(modelMatrix))) * normal;
    Tangent = mat3(transpose(inverse(modelMatrix))) * tangent;
    WorldPos = vec3(modelMatrix);
    TexCoords = texCoords;
}
