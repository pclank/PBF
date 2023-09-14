#include "Mesh.hpp"

#include <iostream>
#include <glad/glad.h>
#include <glm/glm.hpp>
#include <glm/gtx/matrix_decompose.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtc/type_ptr.hpp>


#define STB_IMAGE_IMPLEMENTATION
//#define SCENE_LOAD_FLAGS (aiProcess_Triangulate | aiProcess_GenNormals | aiProcess_JoinIdenticalVertices | aiProcess_GenUVCoords | aiProcess_FindInvalidData | aiProcess_TransformUVCoords | aiProcess_PreTransformVertices)
#define SCENE_LOAD_FLAGS (aiProcess_Triangulate | aiProcess_GenNormals | aiProcess_GenUVCoords)

// System Headers
#include <stb_image.h>

Mesh::Mesh(std::string const& filename, Shader* shader, unsigned int instancedVBO)
    //:
    //Mesh()
{
    this->shader = shader;
    this->instancedVBO = instancedVBO;

    //Assimp::Importer importer;
    const aiScene* scene = importer.ReadFile(
        filename,
        SCENE_LOAD_FLAGS
    );

    if (!scene)
    {
        throw std::runtime_error(importer.GetErrorString());
    }
    else
    {
        dir = filename.substr(0, filename.find_last_of('/'));
        Parse(scene->mRootNode, scene, instancedVBO);

        // Set root node
        this->scene = scene;
    }
}

Mesh::Mesh(std::vector<Vertex> const& verts, std::vector<unsigned int> const& indices, std::vector<Texture> const& textures, Shader* shader, unsigned int instancedVBO)
    :
    m_vertices(verts),
    m_indices(indices),
    m_textures(textures),
    shader(shader),
    instancedVBO(instancedVBO)
{
    // bind the default vertex array object
    glGenVertexArrays(1, &m_VAO);
    glBindVertexArray(m_VAO);

    // bind & create the vertex buffer
    glGenBuffers(1, &m_VBO);
    glBindBuffer(GL_ARRAY_BUFFER, m_VBO);
    glBufferData(
        GL_ARRAY_BUFFER,
        verts.size() * sizeof(Vertex),
        verts.data(),
        GL_STATIC_DRAW
    );

    // bind & create the index buffer
    glGenBuffers(1, &m_IBO);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_IBO);
    glBufferData(
        GL_ELEMENT_ARRAY_BUFFER,
        indices.size() * sizeof(unsigned int),
        indices.data(),
        GL_STATIC_DRAW
    );

    // Set Shader Attributes
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (GLvoid*)offsetof(Vertex, position));
    glEnableVertexAttribArray(0); // Vertex Positions


    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)offsetof(Vertex, normal));
    glEnableVertexAttribArray(1); // Vertex Normals

    glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)offsetof(Vertex, texCoords));
    glEnableVertexAttribArray(2);  // Vertex texture coords

    glVertexAttribPointer(3, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)offsetof(Vertex, tangent));
    glEnableVertexAttribArray(3);     // Vertex tangent


    glVertexAttribPointer(4, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)offsetof(Vertex, biTangent));
    glEnableVertexAttribArray(4);  // Vertex bitangent

    glVertexAttribIPointer(5, MAXIMUM_BONES, GL_INT, sizeof(Vertex), (void*)offsetof(Vertex, boneIDs));
    glEnableVertexAttribArray(5);   // Bone ids


    glVertexAttribPointer(6, MAXIMUM_BONES, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)offsetof(Vertex, weights));
    glEnableVertexAttribArray(6);  // Bone weights

    // Enable instancing
    if (this->instancedVBO != 0)
    {
        glBindBuffer(GL_ARRAY_BUFFER, instancedVBO);
        glEnableVertexAttribArray(7);
        glVertexAttribPointer(7, 3, GL_FLOAT, GL_FALSE, sizeof(glm::vec3), (void*)0);
        glBindBuffer(GL_ARRAY_BUFFER, 0);
        glVertexAttribDivisor(7, 1);
    }

    glBindVertexArray(0);
}

Mesh::~Mesh()
{
    glDeleteBuffers(1, &m_VBO);
    glDeleteBuffers(1, &m_IBO);
    glDeleteVertexArrays(1, &m_VAO);
}

void Mesh::Render(glm::mat4 view, glm::mat4 model, glm::mat4 projection, glm::vec3 cam_pos, glm::vec3 light_pos, glm::vec3 base_color, glm::vec3 manual_light_color, float manual_metallic, float manual_roughness, GLuint texture_diffuse, GLuint texture_normal, GLuint texture_specular)
{
    // Use shader
    shader->use();

    // Pass uniforms
    shader->setMat4("viewMatrix", view);
    shader->setMat4("modelMatrix", model);
    shader->setMat4("projectionMatrix", projection);
    shader->setVec3("CamPos", cam_pos);
    shader->setVec3("LightPosition", light_pos);
    shader->setVec3("BaseColor", base_color);
    shader->setVec3("ManualLightColor", manual_light_color);
    shader->setFloat("ManualMetallic", manual_metallic);
    shader->setFloat("ManualRoughness", manual_roughness);
    //shader->setInt("DiffuseTexture", 0);
    //shader->setInt("NormalTexture", 1);
    //shader->setInt("SpecularTexture", 2);


    for (auto& mesh : m_subMeshes)
        mesh->Render(view, model, projection, cam_pos, light_pos, base_color, manual_light_color, manual_metallic, manual_roughness, texture_diffuse, texture_normal, texture_specular);

    // bind appropriate textures
    unsigned int diffuseNr = 0;
    unsigned int specularNr = 0;
    unsigned int normalNr = 0;
    unsigned int heightNr = 0;
    for (unsigned int i = 0; i < m_textures.size(); i++)
    {
        glActiveTexture(GL_TEXTURE0 + i); // active proper texture unit before binding
        // retrieve texture number (the N in diffuse_textureN)
        std::string number;
        std::string name = m_textures[i].type;
        if (name == "texture_diffuse")
            number = std::to_string(diffuseNr++);
        else if (name == "texture_specular")
            number = std::to_string(specularNr++); // transfer unsigned int to string
        else if (name == "texture_normal")
            number = std::to_string(normalNr++); // transfer unsigned int to string
        else if (name == "texture_height")
            number = std::to_string(heightNr++); // transfer unsigned int to string

        // now set the sampler to the correct texture unit
        //glUniform1i(glGetUniformLocation(shader->getShaderID(), (name + number).c_str()), i);
        glUniform1i(glGetUniformLocation(shader->getShaderID(), name.c_str()), i);
        // and finally bind the texture
        glBindTexture(GL_TEXTURE_2D, m_textures[i].id);
    }

    glBindVertexArray(m_VAO);
    glDrawElements(GL_TRIANGLES, m_indices.size(), GL_UNSIGNED_INT, 0);
    glActiveTexture(GL_TEXTURE0);
    glBindVertexArray(0);
}

void Mesh::RenderInstanced(glm::mat4 view, glm::mat4 model, glm::mat4 projection, glm::vec3 cam_pos, glm::vec3 light_pos, glm::vec3 base_color, glm::vec3 manual_light_color, float manual_metallic, float manual_roughness, GLuint texture_diffuse, GLuint texture_normal, GLuint texture_specular, unsigned int amount)
{
    // Use shader
    shader->use();

    // Pass uniforms
    shader->setMat4("viewMatrix", view);
    shader->setMat4("modelMatrix", model);
    shader->setMat4("projectionMatrix", projection);
    shader->setVec3("CamPos", cam_pos);
    shader->setVec3("LightPosition", light_pos);
    shader->setVec3("BaseColor", base_color);
    shader->setVec3("ManualLightColor", manual_light_color);
    shader->setFloat("ManualMetallic", manual_metallic);
    shader->setFloat("ManualRoughness", manual_roughness);
    //shader->setInt("DiffuseTexture", 0);
    //shader->setInt("NormalTexture", 1);
    //shader->setInt("SpecularTexture", 2);


    for (auto& mesh : m_subMeshes)
        mesh->RenderInstanced(view, model, projection, cam_pos, light_pos, base_color, manual_light_color, manual_metallic, manual_roughness, texture_diffuse, texture_normal, texture_specular, amount);

    // bind appropriate textures
    unsigned int diffuseNr = 0;
    unsigned int specularNr = 0;
    unsigned int normalNr = 0;
    unsigned int heightNr = 0;
    for (unsigned int i = 0; i < m_textures.size(); i++)
    {
        glActiveTexture(GL_TEXTURE0 + i); // active proper texture unit before binding
        // retrieve texture number (the N in diffuse_textureN)
        std::string number;
        std::string name = m_textures[i].type;
        if (name == "texture_diffuse")
            number = std::to_string(diffuseNr++);
        else if (name == "texture_specular")
            number = std::to_string(specularNr++); // transfer unsigned int to string
        else if (name == "texture_normal")
            number = std::to_string(normalNr++); // transfer unsigned int to string
        else if (name == "texture_height")
            number = std::to_string(heightNr++); // transfer unsigned int to string

        // now set the sampler to the correct texture unit
        //glUniform1i(glGetUniformLocation(shader->getShaderID(), (name + number).c_str()), i);
        glUniform1i(glGetUniformLocation(shader->getShaderID(), name.c_str()), i);
        // and finally bind the texture
        glBindTexture(GL_TEXTURE_2D, m_textures[i].id);
    }

    glBindVertexArray(m_VAO);
    glDrawElementsInstanced(GL_TRIANGLES, m_indices.size(), GL_UNSIGNED_INT, 0, amount);
    glActiveTexture(GL_TEXTURE0);
    glBindVertexArray(0);
}

void Mesh::Parse(const aiNode* node, const aiScene* scene, unsigned int instancedVBO)
{
    for (unsigned int i = 0; i < node->mNumMeshes; i++)
        Parse(scene->mMeshes[node->mMeshes[i]], scene, instancedVBO);

    for (unsigned int i = 0; i < node->mNumChildren; i++)
        Parse(node->mChildren[i], scene, instancedVBO);
}

void Mesh::Parse(const aiMesh* mesh, const aiScene* scene, unsigned int instancedVBO)
{
    // Parse vertices
    std::vector<Vertex> vertices;
    Vertex vert;
    for (unsigned int i = 0; i < mesh->mNumVertices; i++)
    {
        vert = {};
        vert.position = glm::vec3(mesh->mVertices[i].x, mesh->mVertices[i].y, mesh->mVertices[i].z);
        if (mesh->HasNormals())
            vert.normal = glm::vec3(mesh->mNormals[i].x, mesh->mNormals[i].y, mesh->mNormals[i].z);
        if (mesh->mTextureCoords[0])
        {
            vert.texCoords = glm::vec2(mesh->mTextureCoords[0][i].x, mesh->mTextureCoords[0][i].y);
            if (mesh->mTangents)
            {
                vert.tangent = glm::vec3(mesh->mTangents[i].x, mesh->mTangents[i].y, mesh->mTangents[i].z);
                vert.biTangent = glm::vec3(mesh->mBitangents[i].x, mesh->mBitangents[i].y, mesh->mBitangents[i].z);
            }
            else
            {
                vert.tangent = glm::vec3(1.0f);
                vert.biTangent = glm::vec3(1.0f);
            }
        }
        else
            vert.texCoords = glm::vec2(0.0f, 0.0f);
        vertices.push_back(vert);
    }

    // Parse textures
    aiMaterial* mat = scene->mMaterials[mesh->mMaterialIndex];
    std::vector<Texture> textures;
    std::vector<Texture> diffuseMaps = LoadMaterialTextures(mat, aiTextureType_DIFFUSE, "texture_diffuse");
    textures.insert(textures.end(), diffuseMaps.begin(), diffuseMaps.end());
    std::vector<Texture> specularMaps = LoadMaterialTextures(mat, aiTextureType_SPECULAR, "texture_specular");
    textures.insert(textures.end(), specularMaps.begin(), specularMaps.end());
    std::vector<Texture> normalMaps = LoadMaterialTextures(mat, aiTextureType_NORMALS, "texture_normal");
    textures.insert(textures.end(), normalMaps.begin(), normalMaps.end());
    std::vector<Texture> heightMaps = LoadMaterialTextures(mat, aiTextureType_HEIGHT, "texture_height");
    textures.insert(textures.end(), heightMaps.begin(), heightMaps.end());

    // Parse indices
    std::vector<unsigned int> indices;
    for (unsigned int i = 0; i < mesh->mNumFaces; i++)
        for (unsigned int j = 0; j < mesh->mFaces[i].mNumIndices; j++)
            indices.push_back(mesh->mFaces[i].mIndices[j]);

    this->m_vertices = vertices;
    this->m_indices = indices;
    this->m_textures = textures;

    m_subMeshes.push_back(
        std::unique_ptr<Mesh>(new Mesh(this->m_vertices, this->m_indices, this->m_textures, shader, instancedVBO))
    );
}

inline glm::mat4 Mesh::ConvertMatrixToGLMFormat(const aiMatrix4x4& from)
{
    return glm::transpose(glm::make_mat4(&from.a1));
}

// Converts aiVector3D to GLM::vec3
inline glm::vec3 Mesh::ConvertVector3DToGLMFormat(const aiVector3D& src)
{
    return glm::vec3(src.x, src.y, src.z);
}

// Converts aiQuaternion to GLM::quat
inline glm::quat Mesh::ConvertQuaternionToGLMFormat(const aiQuaternion& src)
{
    return glm::quat(src.w, src.x, src.y, src.z);
}

std::vector<Texture> Mesh::LoadMaterialTextures(aiMaterial* mat, aiTextureType type, std::string typeName)
{
    std::vector<Texture> textures;
    for (unsigned int i = 0; i < mat->GetTextureCount(type); i++)
    {
        aiString str;
        mat->GetTexture(type, i, &str);
        // check if texture was loaded before and if so, continue to next iteration: skip loading a new texture
        bool skip = false;
        for (unsigned int j = 0; j < m_textures.size(); j++)
        {
            if (std::strcmp(m_textures[j].path.data(), str.C_Str()) == 0)
            {
                textures.push_back(m_textures[j]);
                skip = true; // a texture with the same filepath has already been loaded, continue to next one. (optimization)
                break;
            }
        }
        if (!skip)
        {   // if texture hasn't been loaded already, load it
            Texture texture;
            texture.id = TextureFromFile(str.C_Str(), this->dir);
            texture.type = typeName;
            texture.path = str.C_Str();
            textures.push_back(texture);
            m_textures.push_back(texture);  // store it as texture loaded for entire model, to ensure we won't unnecessary load duplicate textures.
        }
    }
    return textures;
}


unsigned int Mesh::TextureFromFile(const char* path, const std::string& directory, bool gamma)
{
    std::string filename = std::string(path);
    filename = directory + '/' + filename;

    unsigned int textureID;
    glGenTextures(1, &textureID);

    int width, height, nrComponents;
    unsigned char* data = stbi_load(filename.c_str(), &width, &height, &nrComponents, 0);
    if (data)
    {
        GLenum format;
        if (nrComponents == 1)
            format = GL_RED;
        else if (nrComponents == 3)
            format = GL_RGB;
        else if (nrComponents == 4)
            format = GL_RGBA;

        glBindTexture(GL_TEXTURE_2D, textureID);
        glTexImage2D(GL_TEXTURE_2D, 0, format, width, height, 0, format, GL_UNSIGNED_BYTE, data);
        glGenerateMipmap(GL_TEXTURE_2D);

        if (format == GL_RGBA)
        {
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
        }
        else
        {
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
        }
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

        stbi_image_free(data);
    }
    else
    {
        std::cout << "Texture failed to load at path: " << path << std::endl;
        stbi_image_free(data);
    }

    return textureID;
}

void Mesh::ChangeShader(Shader* new_shader)
{
    shader = new_shader;
    m_subMeshes[0]->shader = new_shader;
}

Shader* Mesh::getShader()
{
    return shader;
}