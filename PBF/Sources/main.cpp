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

// System Headers
#include <glad/glad.h>
#include <GLFW/glfw3.h>

// Standard Headers
#include <cstdio>
#include <cstdlib>
#include <iostream>

//#define COLOR_NEIGHBORS
#define INSTANCED_RENDERING

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
    0.5f,                   // default wind effect force
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
bool g_down = false;

// Track Previous Camera Parameters
float lastX = (float)mWidth / 2.0;
float lastY = (float)mHeight / 2.0;

Simulation* sim_p;

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

    // create and link instanced shader
    Shader instancedShader = Shader();
    instancedShader.init();

    instancedShader
        .registerShader("Shaders/instanced_shader.vert", GL_VERTEX_SHADER)
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

    // VBOS for instancing
    unsigned int instancedVBO, instancedCellVBO;
    glGenBuffers(1, &instancedVBO);
    glGenBuffers(1, &instancedCellVBO);

    // Create Particle Mesh
#ifdef INSTANCED_RENDERING
    Mesh p_mesh("Assets/particle_sphere.fbx", &instancedShader, instancedVBO);
    Mesh c_mesh("Assets/particle_sphere.fbx", &instancedShader, instancedCellVBO);
#else
    Mesh p_mesh("Assets/particle_sphere.fbx", &defaultShader);
#endif

    // Initialize Simulation
    Simulation sim(128, 1.0f, glm::vec3(-2.0f, 1.0f, 0.0f), glm::vec3(0.0f), 0.1f, 5.0f, 4.0f, 0.25f, true, &p_mesh);
    //Simulation sim(128, 1.0f, glm::vec3(-2.0f, 1.0f, 0.0f), glm::vec3(0.0f), 0.1f, 5.0f, 4.0f, 1.0f, true, &p_mesh);
    sim_p = &sim;

#ifdef INSTANCED_RENDERING
    // Set up instancing
    glBindBuffer(GL_ARRAY_BUFFER, instancedVBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(glm::vec3) * sim.n_particles, &sim.particle_coms[0], GL_DYNAMIC_DRAW);
    glBindBuffer(GL_ARRAY_BUFFER, instancedCellVBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(glm::vec3)* sim.n_cells, &sim.cell_coms[0], GL_STATIC_DRAW);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
#endif

    // Initialize our GUI
    GUI gui = GUI(mWindow, g_camera, g_renderData, g_timer, assetLoader);
    gui.Init();

    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    // Rendering Loop
    while (glfwWindowShouldClose(mWindow) == false)
    {
        // Update Timer
        g_timer.Tick();

        // Process Keyboard Input
        processKeyboardInput(mWindow);

        // Tick Simulation
        if (sim.sim_running)
        {
            if (sim.step_run && g_down)
            {
                if (g_renderData.wind_effect)
                    sim.RandomWind(g_renderData.wind_force);
                sim.TickSimulation(g_timer.GetData().DeltaTime);
            }
            else if (!sim.step_run)
            {
                if (g_renderData.wind_effect)
                    sim.RandomWind(g_renderData.wind_force);
                sim.TickSimulation(g_timer.GetData().DeltaTime);
            }
        }

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
        glDisable(GL_BLEND);
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
        glEnable(GL_BLEND);
#ifndef INSTANCED_RENDERING
        for (int i = 0; i < sim.n_particles; i++)
        {
            p_mesh.Render(
                view,
                //glm::scale(glm::translate(glm::mat4(1.0f), sim.particles[i].com), glm::vec3(g_renderData.scale)),
                glm::translate(glm::mat4(1.0f), sim.particles[i].com),
                projection,
                g_camera.position,
                glm::vec3(g_renderData.light_position[0], g_renderData.light_position[1], g_renderData.light_position[2]),
#ifdef COLOR_NEIGHBORS
                glm::vec3(ColorTable[sim.cell_map[sim.particles[i].cell] * 3], ColorTable[sim.cell_map[sim.particles[i].cell] * 3 + 1], ColorTable[sim.cell_map[sim.particles[i].cell] * 3 + 2]),
#else
                glm::vec3(0.83f, 0.94f, 0.97f),
#endif
                glm::vec3(g_renderData.light_color[0], g_renderData.light_color[1], g_renderData.light_color[2]),
                g_renderData.manual_metallic,
                g_renderData.manual_roughness,
                texture_diffuseID,
                texture_normalID,
                texture_specularID
            );
        }
#endif
        
#ifdef INSTANCED_RENDERING
        // Update Buffer
        glBindBuffer(GL_ARRAY_BUFFER, instancedVBO);
        glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(glm::vec3) * sim.n_particles, &sim.particle_coms[0]);
        glBindBuffer(GL_ARRAY_BUFFER, 0);

        p_mesh.RenderInstanced(
            view,
            glm::mat4(1.0f),
            projection,
            g_camera.position,
            glm::vec3(g_renderData.light_position[0], g_renderData.light_position[1], g_renderData.light_position[2]),
#ifdef COLOR_NEIGHBORS
            glm::vec3(ColorTable[sim.cell_map[sim.particles[i].cell] * 3], ColorTable[sim.cell_map[sim.particles[i].cell] * 3 + 1], ColorTable[sim.cell_map[sim.particles[i].cell] * 3 + 2]),
#else
            glm::vec3(0.83f, 0.94f, 0.97f),
#endif
            glm::vec3(g_renderData.light_color[0], g_renderData.light_color[1], g_renderData.light_color[2]),
            g_renderData.manual_metallic,
            g_renderData.manual_roughness,
            texture_diffuseID,
            texture_normalID,
            texture_specularID,
            sim.n_particles
        );
#endif

#ifndef INSTANCED_RENDERING
        // Render Cells
        for (int i = 0; i < sim.n_cells; i++)
        {
            p_mesh.Render(
                view,
                glm::scale(glm::translate(glm::mat4(1.0f), sim.grid[i].pos), glm::vec3(g_renderData.scale)),
                projection,
                g_camera.position,
                glm::vec3(g_renderData.light_position[0], g_renderData.light_position[1], g_renderData.light_position[2]),
                glm::vec3(1.0f, 1.0f, 1.0f),
                //glm::vec3(g_renderData.light_color[0], g_renderData.light_color[1], g_renderData.light_color[2]),
                glm::vec3(1.0f, 1.0f, 1.0f),
                g_renderData.manual_metallic,
                g_renderData.manual_roughness,
                texture_diffuseID,
                texture_normalID,
                texture_specularID
            );
        }
#endif

#ifdef INSTANCED_RENDERING
        // Render Cells
        c_mesh.RenderInstanced(
            view,
            glm::scale(glm::mat4(1.0f), glm::vec3(g_renderData.scale)),
            projection,
            g_camera.position,
            glm::vec3(g_renderData.light_position[0], g_renderData.light_position[1], g_renderData.light_position[2]),
            glm::vec3(1.0f, 1.0f, 1.0f),
            //glm::vec3(g_renderData.light_color[0], g_renderData.light_color[1], g_renderData.light_color[2]),
            glm::vec3(1.0f, 1.0f, 1.0f),
            g_renderData.manual_metallic,
            g_renderData.manual_roughness,
            texture_diffuseID,
            texture_normalID,
            texture_specularID,
            sim.n_cells
        );
#endif
        
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
        sim_p->sim_running = !sim_p->sim_running;

        p_down = false;
    }

    // Toggle Step Simulation
    if (r_down && glfwGetKey(window, GLFW_KEY_R) == GLFW_RELEASE)
    {
        sim_p->step_run = !sim_p->step_run;

        r_down = false;
    }

    // Run Step Simulation
    if (g_down && glfwGetKey(window, GLFW_KEY_G) == GLFW_RELEASE)
    {
        g_down = false;
    }

    if (!spacebar_down && glfwGetKey(window, GLFW_KEY_SPACE) == GLFW_PRESS)
        spacebar_down = true;

    if (!p_down && glfwGetKey(window, GLFW_KEY_P) == GLFW_PRESS)
        p_down = true;

    if (!r_down && glfwGetKey(window, GLFW_KEY_R) == GLFW_PRESS)
        r_down = true;

    if (!g_down && sim_p->step_run && glfwGetKey(window, GLFW_KEY_G) == GLFW_PRESS)
        g_down = true;

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