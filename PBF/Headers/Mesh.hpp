#pragma once
#include "Vertex.hpp"
#include "Shader.hpp"
#include <vector>
#include <memory>
#include <map>
#include <assimp/Importer.hpp>
#include <assimp/postprocess.h>
#include <assimp/scene.h>

/// <summary>
/// Implements Meshes that were imported using Assimp
/// </summary>
class Mesh
{
public:
	// Delete copy and assignment operators
	Mesh(Mesh const&) = delete;
	Mesh& operator=(Mesh const&) = delete;

	// constructors
	Mesh();
	Mesh(std::string const& filename, Shader* shader, unsigned int instancedVBO = 0);
	~Mesh();

	void Render(glm::mat4, glm::mat4, glm::mat4, glm::vec3, glm::vec3, glm::vec3, glm::vec3, float, float, GLuint, GLuint, GLuint);

	void RenderInstanced(glm::mat4, glm::mat4, glm::mat4, glm::vec3, glm::vec3, glm::vec3, glm::vec3, float, float, GLuint, GLuint, GLuint, unsigned int);
	
	/// <summary>
	/// Change Shader associated with the mesh
	/// </summary>
	/// <param name="new_shader">: the new Shader</param>
	void ChangeShader(Shader* new_shader);

	std::vector<Vertex> m_vertices;												// Vertices of Mesh (Vertex struct)
	unsigned int instancedVBO;

	Shader* getShader();
	
private:
	Mesh(std::vector<Vertex> const& verts, std::vector<unsigned int> const& indices, std::vector<Texture> const& textures, Shader* shader, unsigned int instancedVBO = 0);

	void Parse(const aiMesh* mesh, const aiScene* scene, unsigned int instancedVBO = 0);
	void Parse(const aiNode* node, const aiScene* scene, unsigned int instancedVBO = 0);

	/// <summary>
	/// Loads textures bases on type
	/// </summary>
	/// <param name="mat"></param>
	/// <param name="type"></param>
	/// <param name="typeName"></param>
	/// <returns></returns>
	std::vector<Texture> LoadMaterialTextures(aiMaterial* mat, aiTextureType type, std::string typeName);

	/// <summary>
	/// Loads textures from file
	/// </summary>
	/// <param name="path"></param>
	/// <param name="directory"></param>
	/// <param name="gamma"></param>
	/// <returns></returns>
	unsigned int TextureFromFile(const char* path, const std::string& directory, bool gamma = false);

	/// <summary>
	/// Conversion from aiMatrix4x4 to glm::mat4
	/// </summary>
	/// <param name="from">: source matrix</param>
	/// <returns></returns>
	inline glm::mat4 ConvertMatrixToGLMFormat(const aiMatrix4x4& from);

	/// <summary>
	/// Conversion from aiVector3D to glm::vec3
	/// </summary>
	/// <param name="src">: source vector</param>
	/// <returns></returns>
	inline glm::vec3 ConvertVector3DToGLMFormat(const aiVector3D& src);

	/// <summary>
	/// Conversion from aiQuaternion to glm::quat
	/// </summary>
	/// <param name="src">: source quaternion</param>
	/// <returns></returns>
	inline glm::quat ConvertQuaternionToGLMFormat(const aiQuaternion& src);

	std::vector<unsigned int> m_indices;										// Indices for rendering
	std::vector<Texture> m_textures;											// Textures associated with this mesh
	std::string dir;															// Mesh directory
	Assimp::Importer importer;													// Assimp Importer for the scene (MUST LIVE!!!)
	const aiScene* scene;														// Points to scene of the mesh. Needed to preserve node tree for bone transformation calculations
	Shader* shader;																// Shader used for rendering this mesh (Shader class)
	std::vector<std::unique_ptr<Mesh>> m_subMeshes;								// Who knows at this point

	// Buffer - Array Objects
	unsigned int m_VBO;
	unsigned int m_IBO;
	unsigned int m_VAO;
};