#include <raylib.h>
#include <raymath.h>
#include <iostream>
#include <vector>

typedef struct Point {
    Vector3 position;
    Vector3 rotation;
} Point;

typedef struct Trajectory {
    std::vector<Point> points;
} Trajectory;

Trajectory generateTrajectory(Point start, Point end, int steps) {
    Trajectory trajectory;
    trajectory.points.reserve(steps + 1);
    
    float stepX = (end.position.x - start.position.x) / steps;
    float stepY = (end.position.y - start.position.y) / steps;
    float stepZ = (end.position.z - start.position.z) / steps;

    for (int i = 0; i <= steps; ++i) {
        Point point;
        point.position.x = start.position.x + stepX * i;
        point.position.y = start.position.y + stepY * i;
        point.position.z = start.position.z + stepZ * i;

        // For simplicity, we keep rotation constant
        point.rotation = start.rotation;

        trajectory.points.push_back(point);
    }

    return trajectory;
}

// generate trajectory with a circular segment from start to end
Trajectory generateCircularTrajectory(Vector3 center, Point start, Point end, int steps) {
    Trajectory trajectory;
    trajectory.points.reserve(steps + 1);

    // calculate circle segment between start and end points
    Vector3 startToCenter = Vector3Subtract(center, start.position);
    Vector3 endToCenter = Vector3Subtract(center, end.position);
    float radius = Vector3Length(startToCenter);
    float angleStart = atan2f(startToCenter.z, startToCenter.x);
    float angleEnd = atan2f(endToCenter.z, endToCenter.x);
    float angleStep = (angleEnd - angleStart) / steps;
    if (angleStep < 0) angleStep += 2 * PI; // Ensure positive angle step
    if (angleStep > 2 * PI) angleStep -= 2 * PI; // Ensure angle step is within a full circle
    for (int i = 0; i <= steps; ++i) {
        Point point;
        float angle = angleStart + angleStep * i;
        point.position.x = center.x + radius * cosf(angle);
        point.position.z = center.z + radius * sinf(angle);
        point.position.y = start.position.y; // Keep the height constant

        // For simplicity, we keep rotation constant
        point.rotation = start.rotation;

        trajectory.points.push_back(point);
    }

    return trajectory;
}

Trajectory interpolateCircularSegment(
    const Point& start,
    const Point& end,
    Vector3 center,
    int numPoints = 5,
    double radius = 0.0
) {
    Trajectory trajectory;
    const double epsilon = 5e-7;
    Point auxPoint;

    // Calcular nuevo centro si se especifica radio
    if (radius != 0.0) {
        Vector3 V = Vector3Subtract(end.position, start.position);
        double d = Vector3Length(V);
        Vector3 M = Vector3Scale(Vector3Add(start.position, end.position), 0.5f);
        
        if (d > 2.0 * abs(radius)) {
            // Radio inválido, interpolar linealmente
            for (int i = 0; i < numPoints; ++i) {
                double t = static_cast<double>(i) / (numPoints - 1);
                Point p = {Vector3Subtract(end.position, start.position), Vector3Zero()};
                p.position = Vector3Scale(p.position, t);
                p.position = Vector3Add(start.position, p.position);
                trajectory.points.push_back(p);
            }
            return trajectory;
        }

        // Calcular dirección perpendicular desde el centro original
        Vector3 C_dir = Vector3Subtract(center, M);
        double proj = Vector3DotProduct(C_dir, V) / Vector3DotProduct(V, V);
        Vector3 Proj_dir = Vector3Scale(V, proj);
        Vector3 Perp_dir = Vector3Subtract(C_dir, Proj_dir);

        // Manejar caso colineal
        if (Vector3Length(Perp_dir) < epsilon) {
            // Generar dirección perpendicular arbitraria
            Vector3 temp = {1.0f, 0.0f, 0.0f};
            Perp_dir = Vector3CrossProduct(V, temp);
            if (Vector3Length(Perp_dir) < epsilon) {
                temp = (Vector3){0, 1, 0};
                Perp_dir = Vector3CrossProduct(V, temp);
            }
        }

        Vector3 unit_perp = Vector3Normalize(Perp_dir);
        double h = sqrt(radius*radius - (d/2)*(d/2));
        center = Vector3Add(M, Vector3Scale(unit_perp, h));
    }

    Vector3 v1 = Vector3Subtract(start.position, center);
    Vector3 v2 = Vector3Subtract(end.position, center);

    // Manejar casos especiales
    if (Vector3Length(v1) < epsilon || Vector3Length(v2) < epsilon) {
        trajectory.points.push_back(start);
        trajectory.points.push_back(end);
        return trajectory;
    }

    double r1 = Vector3Length(v1);
    double r2 = Vector3Length(v2);

    // Verificar radio constante
    if (std::abs(r1 - r2) > epsilon) {
        for (int i = 0; i < numPoints; ++i) {
            double t = static_cast<double>(i) / (numPoints - 1);
            Point p = {Vector3Subtract(end.position, start.position), Vector3Zero()};
            p.position = Vector3Scale(p.position, t);
            p.position = Vector3Add(start.position, p.position);
            trajectory.points.push_back(p);
        }
        std::cout << "diferencia máxima entre radios excedida: " << r1-r2 << std::endl;
        return trajectory;
    }

    // Calcular eje de rotación
    Vector3 normal_axis = Vector3CrossProduct(v1, v2);
    double axis_len = Vector3Length(normal_axis);

    // Caso colineal (segmento recto)
    if (axis_len < epsilon) {
        for (int i = 0; i < numPoints; ++i) {
            double t = static_cast<double>(i) / (numPoints - 1);
            Point p = {Vector3Subtract(end.position, start.position), Vector3Zero()};
            p.position = Vector3Scale(p.position, t);
            p.position = Vector3Add(start.position, p.position);
            trajectory.points.push_back(p);
        }
        return trajectory;
    }

    normal_axis = Vector3Normalize(normal_axis);

    // Calcular ángulo entre vectores
    double cos_theta = Vector3DotProduct(v1, v2) / (r1 * r2);
    double sin_theta = axis_len / (r1 * r2);
    double theta = std::atan2(sin_theta, cos_theta);
    if(radius < 0.0) theta -= 360*DEG2RAD;

    // Generar puntos interpolados
    for (int i = 0; i < numPoints; ++i) {
        double t = static_cast<double>(i) / (numPoints - 1);
        double angle = t * theta;

        // Aplicar fórmula de rotación de Rodrigues
        Vector3 term1 = Vector3Scale(v1, std::cos(angle));
        Vector3 term2 = Vector3Scale(Vector3CrossProduct(normal_axis, v1), std::sin(angle));
        Vector3 term3 = Vector3Scale(normal_axis, Vector3DotProduct(normal_axis, v1));
        term3 = Vector3Scale(term3, (1 - std::cos(angle)));
        Vector3 rotated = Vector3Add(term1, term2);
        rotated = Vector3Add(rotated, term3);

        auxPoint = {Vector3Add(center, rotated), Vector3Zero()};
        trajectory.points.push_back(auxPoint);
    }

    return trajectory;
}

int main() {
    // Initialize the window
    const int screenWidth = 480;
    const int screenHeight = 480;
    InitWindow(screenWidth, screenHeight, "robot journey");

    // Set the target FPS
    SetTargetFPS(60);

    Camera camera = { {0.0f, 3.0f, 5.0f}, Vector3Zero(), { 0.0f, 1.0f, 0.0f }, 45.0f, 0 };
    camera.fovy = 80.0f;
    camera.projection = CAMERA_PERSPECTIVE;

    //SetCameraMode(camera, CAMERA_THIRD_PERSON);
	SetCameraMode(camera, CAMERA_ORBITAL);
    // SetCameraMode(camera, CAMERA_CUSTOM);

    float modelScale = 1.0f;
    Model* robotModel = new Model(LoadModel(std::string("src/mod/gp180/GP180.obj").c_str()));
    robotModel->transform = MatrixScale(1.0f/1000,1.0f/1000,1.0f/1000);
    Model* palletModel = new Model(LoadModel(std::string("src/mod/pallet/pallet1000x1200.obj").c_str()));
    palletModel->transform = MatrixRotate((Vector3){1,0,0},-90*DEG2RAD);

    Point infeederA = { {2.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 0.0f} };
    Point outfeederA = { {0.0f, 0.0f, 2.0f}, {0.0f, 0.0f, 0.0f} };
    Point infeederB = { {0.0f, 2.0f, -2.0f}, {0.0f, 0.0f, 0.0f} };
    Point outfeederB = { {-2.0f, 0.0f, 2.0f}, {0.0f, 0.0f, 0.0f} };

    Vector3 offset = {0.0f, 1.0f, 0.0f};
    Point auxPoint;
    double radius = 0.0;

    offset = {0.0f, 0.5f, 0.0f};
    auxPoint = outfeederA;
    auxPoint.position = Vector3Add(auxPoint.position, offset);
    radius = (Vector3Distance(auxPoint.position,infeederB.position)*0.5+0.001f);
    Trajectory trajectoryOAIA = interpolateCircularSegment(
        auxPoint,
        infeederA,
        (Vector3){0.0f,(auxPoint.position.y+infeederA.position.y)/2.0f,0.0f},
        5,
        radius
    );

    offset = {0.0f, 1.5f, 0.0f};
    auxPoint = outfeederB;
    auxPoint.position = Vector3Add(auxPoint.position, offset);
    radius = -(Vector3Distance(auxPoint.position,infeederB.position)*0.5+0.001f);
    std::cout << "radius: " << radius << std::endl;
    Trajectory trajectoryOBIB = interpolateCircularSegment(
        auxPoint,
        infeederB,
        (Vector3){0.0f,(auxPoint.position.y+infeederB.position.y)/2.0f,0.0f},
        5,
        radius
    );

    // Main loop
    while (!WindowShouldClose()) {
        // Update logic here
        UpdateCamera(&camera);

        if(IsKeyDown(KEY_UP)) {
            radius += 0.01;
            offset = {0.0f, 1.5f, 0.0f};
            auxPoint = outfeederB;
            auxPoint.position = Vector3Add(auxPoint.position, offset);
            trajectoryOBIB = interpolateCircularSegment(
                auxPoint,
                infeederB,
                (Vector3){0.0f,(auxPoint.position.y+infeederB.position.y)/2.0f,0.0f},
                5,
                radius
            );
        }else if (IsKeyDown(KEY_DOWN)) {
            radius -= 0.01;
            offset = {0.0f, 1.5f, 0.0f};
            auxPoint = outfeederB;
            auxPoint.position = Vector3Add(auxPoint.position, offset);
            trajectoryOBIB = interpolateCircularSegment(
                auxPoint,
                infeederB,
                (Vector3){0.0f,(auxPoint.position.y+infeederB.position.y)/2.0f,0.0f},
                5,
                radius
            );
        }
        

        // Start drawing
        BeginDrawing();
        ClearBackground(RAYWHITE);

        BeginMode3D(camera);
            DrawGrid(10, 1.0f);
            DrawModel(*robotModel, Vector3Zero(), modelScale, BLUE);
            DrawModel(*palletModel, outfeederA.position, modelScale, WHITE);
            DrawModel(*palletModel, outfeederB.position, modelScale, WHITE);
            for (const auto& point : trajectoryOAIA.points) {
                DrawSphere(point.position, 0.05f, RED);
                if(&point != &trajectoryOAIA.points.back())
                    DrawLine3D(point.position, trajectoryOAIA.points[&point - &trajectoryOAIA.points[0] + 1].position, GREEN);
            }
            for (const auto& point : trajectoryOBIB.points) {
                DrawSphere(point.position, 0.05f, RED);
                if(&point != &trajectoryOBIB.points.back())
                    DrawLine3D(point.position, trajectoryOBIB.points[&point - &trajectoryOBIB.points[0] + 1].position, GREEN);
            }
        EndMode3D();

        std::string radiusText = "Radius: " + std::to_string(radius);
        DrawText(radiusText.c_str(), 10, 10, 8, DARKGRAY);

        // End drawing
        EndDrawing();
    }

    // Close the window and clean up resources
    CloseWindow();

    UnloadModel(*robotModel);
    UnloadModel(*palletModel);

    return 0;
}