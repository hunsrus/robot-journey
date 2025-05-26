#include <raylib.h>
#include <raymath.h>
#include <iostream>
#include <vector>

int main() {
    // Initialize the window
    const int screenWidth = 128;
    const int screenHeight = 128;
    InitWindow(screenWidth, screenHeight, "robot journey");

    // Set the target FPS
    SetTargetFPS(60);

    Camera camera = { {0.0f, 3.0f, 5.0f}, Vector3Zero(), { 0.0f, 1.0f, 0.0f }, 45.0f, 0 };
    camera.fovy = 80.0f;
    camera.projection = CAMERA_PERSPECTIVE;

    //SetCameraMode(camera, CAMERA_THIRD_PERSON);
	SetCameraMode(camera, CAMERA_ORBITAL);
    // SetCameraMode(camera, CAMERA_CUSTOM);

    float modelScale = 1.0f/1000.0f;
    Model* robotModel = new Model(LoadModel(std::string("src/mod/GP180.obj").c_str()));

    // Main loop
    while (!WindowShouldClose()) {
        // Update logic here
        UpdateCamera(&camera);      // Actualizar camara 3D

        // Start drawing
        BeginDrawing();
        ClearBackground(RAYWHITE);

        BeginMode3D(camera);
            DrawGrid(10, 1.0f);
            DrawModel(*robotModel, Vector3Zero(), modelScale, BLUE);
        EndMode3D();


        // End drawing
        EndDrawing();
    }

    // Close the window and clean up resources
    CloseWindow();

    return 0;
}