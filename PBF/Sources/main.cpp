// Local Headers
#include "glitter.hpp"
#include "Shader.hpp"
#include "Camera.hpp"
#include "Mesh.hpp"
#include "Timer.hpp"
#include "AssetLoader.hpp"
#include "GUI.hpp"
#include <Skybox.hpp>
#include <Simulation.hpp>
#include "Application.hpp"
#include <Border.hpp>
#include <Physics.hpp>

// System Headers
#include <glad/glad.h>
#include <GLFW/glfw3.h>

// Standard Headers
#include <cstdio>
#include <cstdlib>
#include <iostream>

// Input Function Declarations
void processKeyboardInput(GLFWwindow* window);
void mouseMovementCallback(GLFWwindow* window, double x_pos, double y_pos);
void mouseScrollCallback(GLFWwindow* window, double x_offset, double y_offset);
void framebufferSizeCallback(GLFWwindow* window, int width, int height);

// Create Render Settings Globals
static SceneSettings g_renderData =
{
    { 1.0f, 0.5f, 0.5f },   // default light pos
    { 1.0f, 1.0f, 1.0f },   // default base color
    { 1.0f, 0.5f, 0.0f },   // default light color
    0.0f,                   // default metallic color
    0.0f,                   // default roughness
    1.0f,                   // default scale
    false,                  // default wireframe mode
    true,                   // default skybox rendering
    false,                  // default wind effect
    nullptr                 // no active asset at first
};

// Create Camera Object
static Camera g_camera(glm::vec3(0.0f, 1.0f, 4.0f));

// Create Timer object
static Timer g_timer;

// First Mouse Movement Hack
bool first_mouse_flag = true;

// Input Tracking Globals
bool spacebar_down = false;
bool p_down = false;
bool r_down = false;

// Track Previous Camera Parameters
float lastX = (float)mWidth / 2.0;
float lastY = (float)mHeight / 2.0;

int main(int argc, char* argv[])
{
    // Load GLFW and Create a Window
    glfwInit();
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
    glfwWindowHint(GLFW_RESIZABLE, GL_TRUE);
    auto mWindow = glfwCreateWindow(mWidth, mHeight, mAppName, nullptr, nullptr);

    // Check for Valid Context
    if (mWindow == nullptr) {
        fprintf(stderr, "Failed to Create OpenGL Context");
        return EXIT_FAILURE;
    }

    // Create Context and Load OpenGL Functions
    glfwMakeContextCurrent(mWindow);
    gladLoadGL();
    fprintf(stderr, "OpenGL %s\n", glGetString(GL_VERSION));

    // Set Callbacks
    glfwSetFramebufferSizeCallback(mWindow, framebufferSizeCallback);
    glfwSetCursorPosCallback(mWindow, mouseMovementCallback);
    glfwSetScrollCallback(mWindow, mouseScrollCallback);

    // Hide Cursor and Capture Mouse
    glfwSetInputMode(mWindow, GLFW_CURSOR, GLFW_CURSOR_DISABLED);

    // Enable Depth Testing
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LESS);

    // create and link simple shader
    Shader defaultShader = Shader();
    defaultShader.init();

    defaultShader
        .registerShader("Shaders/lighting_shader.vert", GL_VERTEX_SHADER)
        .registerShader("Shaders/lighting_shader_simple.frag", GL_FRAGMENT_SHADER)
        .link();

    // create and link line shader
    Shader lineShader = Shader();
    lineShader.init();

    lineShader
        .registerShader("Shaders/line_shader.vert", GL_VERTEX_SHADER)
        .registerShader("Shaders/line_shader.frag", GL_FRAGMENT_SHADER)
        .link();

    // create and link skybox shader
    Shader skyboxShader = Shader();
    skyboxShader.init();

    skyboxShader
        .registerShader("Shaders/skybox.vert", GL_VERTEX_SHADER)
        .registerShader("Shaders/skybox.frag", GL_FRAGMENT_SHADER)
        .link();

    defaultShader.use();

    // Initialize our application and call its init function
    Application app = Application();
    app.init();

    // Create Skybox
    //Skybox skybox("Assets/Yokohama3/", skyboxShader);
    Skybox skybox("Assets/skybox/", skyboxShader);
    
    // Initialize our dynamic asset loader and load fbx files from the asset folder
    AssetLoader assetLoader = AssetLoader();
    assetLoader.Load("Assets/*.fbx", defaultShader);

    // Create Floor Mesh
    Mesh floor("Assets/ca_floor.fbx", &defaultShader);

    // Create Borders
    Border len_0(0, lineShader);
    Border len_1(1, lineShader);

    // Create Particle Mesh
    Mesh p_mesh("Assets/particle_sphere.fbx", &defaultShader);

    // Initialize Simulation
    //Simulation sim(128, 8, glm::vec3(-2.0f, 1.0f, 0.0f), glm::vec3(-2.0f, 1.0f, 0.0f), 0.1f, 10.0f, 5.0f, 0.2f, false, &p_mesh);
    Simulation sim(128, 8, glm::vec3(-2.0f, 1.0f, 0.0f), glm::vec3(-2.0f, 1.0f, 0.0f), 0.1f, 10.0f, 5.0f, 0.5f, true, &p_mesh);

    // Initialize our GUI
    GUI gui = GUI(mWindow, g_camera, g_renderData, g_timer, assetLoader);
    gui.Init();

    // Rendering Loop
    while (glfwWindowShouldClose(mWindow) == false)
    {
        // Update Timer
        g_timer.Tick();

        // Process Keyboard Input
        processKeyboardInput(mWindow);

        // Update Positions and Velocities
        UpdatePosition(sim.particles, g_timer.GetData().DeltaTime);
        UpdateVelocity(sim.particles, g_timer.GetData().DeltaTime);

        if (g_renderData.wind_effect)
            sim.RandomWind(0.001f);

        // Collision Detection
        sim.CheckCollisionSimple();

        // Background Fill Color
        glClearColor(0.25f, 0.25f, 0.25f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        // Normal/Wireframe Rendering
        if (g_renderData.wireframe_mode)
            glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
        else
            glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

        // Get View and Projection Matrics from Camera
        glm::mat4 view = g_camera.GetCurrentViewMatrix();
        glm::mat4 projection = g_camera.GetCurrentProjectionMatrix(mWidth, mHeight);
  
        GLuint texture_diffuseID = 0;
        GLuint texture_normalID = 1;
        GLuint texture_specularID = 2;

        // Render Skybox
        if (g_renderData.show_skybox)
            skybox.Render(view, projection);
      
        // Render floor
        floor.Render(
            view,
            glm::mat4(1.0f),
            projection,
            g_camera.position,
            glm::vec3(g_renderData.light_position[0], g_renderData.light_position[1], g_renderData.light_position[2]),
            glm::vec3(g_renderData.base_color[0], g_renderData.base_color[1], g_renderData.base_color[2]),
            glm::vec3(g_renderData.light_color[0], g_renderData.light_color[1], g_renderData.light_color[2]),
            g_renderData.manual_metallic,
            g_renderData.manual_roughness,
            texture_diffuseID,
            texture_normalID,
            texture_specularID
        );

        // Draw borders
        len_0.Render(view, projection);
        len_1.Render(view, projection);

        // Render Mesh
        if (g_renderData.active_asset)
        {
            Mesh* pActiveMesh = g_renderData.active_asset->m_mesh.get();
            // TODO: Render Obstacle!
        }

        // Render Particles
        for (int i = 0; i < sim.n_particles; i++)
        {
            p_mesh.Render(
                view,
                glm::scale(glm::translate(glm::mat4(1.0f), sim.particles[i].com), glm::vec3(g_renderData.scale)),
                projection,
                g_camera.position,
                glm::vec3(g_renderData.light_position[0], g_renderData.light_position[1], g_renderData.light_position[2]),
                glm::vec3(g_renderData.base_color[0], g_renderData.base_color[1], g_renderData.base_color[2]),
                glm::vec3(g_renderData.light_color[0], g_renderData.light_color[1], g_renderData.light_color[2]),
                g_renderData.manual_metallic,
                g_renderData.manual_roughness,
                texture_diffuseID,
                texture_normalID,
                texture_specularID
            );
        }
        
        // Render GUI
        gui.Render();

        // Flip Buffers and Draw
        glfwSwapBuffers(mWindow);
        glfwPollEvents();
    }

    // Cleanup GUI
    gui.Cleanup();

    defaultShader.cleanup();
    skyboxShader.cleanup();

    glfwTerminate();

    return EXIT_SUCCESS;
}

// Process Keyboard Input
void processKeyboardInput(GLFWwindow* window)
{
    // Exit on ESC Key Press
    if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
        glfwSetWindowShouldClose(window, true);

    // Enable/Disable Camera
    if (spacebar_down && glfwGetKey(window, GLFW_KEY_SPACE) == GLFW_RELEASE)
    {
        g_camera.enabled = !g_camera.enabled;

        // Enable/Disable Cursor
        if (g_camera.enabled)
            glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);
        else
            glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_NORMAL);

        spacebar_down = false;
    }

    // Start/Pause Simulation
    if (p_down && glfwGetKey(window, GLFW_KEY_P) == GLFW_RELEASE)
    {
        // TODO: Start/Pause Simulation

        p_down = false;
    }

    // Reset Simulation
    if (r_down && glfwGetKey(window, GLFW_KEY_R) == GLFW_RELEASE)
    {
        // TODO: Reset Simulation!

        r_down = false;
    }

    if (!spacebar_down && glfwGetKey(window, GLFW_KEY_SPACE) == GLFW_PRESS)
        spacebar_down = true;

    if (!p_down && glfwGetKey(window, GLFW_KEY_P) == GLFW_PRESS)
        p_down = true;

    if (!r_down && glfwGetKey(window, GLFW_KEY_R) == GLFW_PRESS)
        r_down = true;

    // Ignore Keyboard Inputs for Camera Movement if arcball_mode == true
    if (g_camera.arcball_mode)
        return;

    TimeData time = g_timer.GetData();

    if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS)
        g_camera.MoveCamera(FWD, time.DeltaTime);
    if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS)
        g_camera.MoveCamera(AFT, time.DeltaTime);
    if (glfwGetKey(window, GLFW_KEY_Q) == GLFW_PRESS)
        g_camera.MoveCamera(UPWARD, time.DeltaTime);
    if (glfwGetKey(window, GLFW_KEY_E) == GLFW_PRESS)
        g_camera.MoveCamera(DOWNWARD, time.DeltaTime);
    if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS)
        g_camera.MoveCamera(LEFT, time.DeltaTime);
    if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS)
        g_camera.MoveCamera(RIGHT, time.DeltaTime);
}

void mouseMovementCallback(GLFWwindow * window, double x_pos, double y_pos)
{
    float xpos = static_cast<float>(x_pos);
    float ypos = static_cast<float>(y_pos);

    if (first_mouse_flag)
    {
        lastX = xpos;
        lastY = ypos;
        first_mouse_flag = false;
    }

    float xoffset = xpos - lastX;
    float yoffset = lastY - ypos;

    lastX = xpos;
    lastY = ypos;

    TimeData time = g_timer.GetData();
    if (g_camera.arcball_mode)
        g_camera.RotateArcballCamera(xoffset, yoffset, mWidth, mHeight, time.DeltaTime);
    else
        g_camera.RotateCamera(xoffset, yoffset);
}

void mouseScrollCallback(GLFWwindow* window, double x_offset, double y_offset)
{
    TimeData time = g_timer.GetData();
    g_camera.MoveArcballCamera(y_offset, time.DeltaTime);
}

void framebufferSizeCallback(GLFWwindow* window, int width, int height)
{
    glViewport(0, 0, width, height);
}