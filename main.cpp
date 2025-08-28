#define RLIGHTS_IMPLEMENTATION      //Importante para que defina las funciones de rlights y eso
#define PLATFORM_DESKTOP

#include <raylib.h>
#include <raymath.h>
#define RAYGUI_IMPLEMENTATION
#include "raygui.h"
#include <iostream>
#include <iomanip>
#include <fstream>
#include <vector>
#include <algorithm>
#include <string>

#include "YMConnect.h" // Include the YMConnect header file

#include "rlights.h"
#include "rlgl.h"

#if defined(PLATFORM_DESKTOP)
    #define GLSL_VERSION            330
#else   // PLATFORM_RPI, PLATFORM_ANDROID, PLATFORM_WEB
    #define GLSL_VERSION            100
#endif

#define FLT_MAX 340282346638528859811704183484516925440.0f     // Maximum value of a float, from bit pattern 01111111011111111111111111111111
#define COORD_PRECISION 3

static bool DRAW_ZONES = false;
static bool DRAW_WIRED = false;
static bool DRAW_TRAJECTORIES = false;

static Color COLOR_FG = {100,100,100,255};
static Color COLOR_BG = {215,215,215,255};
static Color COLOR_HL2 = {254, 119, 67,255};
static Color COLOR_HL = {68, 125, 155,255};
// static Color COLOR_FG = {102,155,188,255};
// static Color COLOR_BG = {253,240,213,255};
// static Color COLOR_HL = {0,48,73,255};
// static Color COLOR_HL2 = {193,18,31,255};

#include <rshapes.c>
void DrawRectangleRoundedRadius(Rectangle rec, float radius, int segments, Color color);


Shader initShader(void);

typedef struct Link
{
    float length;
    Vector3 origin;
    Model* model;
    Link* parent = nullptr;
}Link;

typedef struct Point {
    Vector3 position;
    Vector3 rotation;
    Vector3 offset = Vector3Zero();
    bool selected = false;
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

struct CubeZone {
    int id;
    std::string name;
    float P1, P2, P3;
    Vector3 maxCoords;
    Vector3 minCoords;
    Vector3 maxCoordsAux; // adaptadas a las coordenadas de raylib
    Vector3 minCoordsAux; // adaptadas a las coordenadas de raylib
    Vector3 position; // solo para graficos
    Vector3 size; // solo para gráficos
    Color color = COLOR_HL;
};

bool isZoneDefined(const CubeZone &z) {
    bool allZero = (z.P1 == 0 && z.P2 == 0 && z.P3 == 0 &&
                    z.maxCoords.x == 0 && z.maxCoords.y == 0 && z.maxCoords.z == 0 &&
                    z.minCoords.x == 0 && z.minCoords.y == 0 && z.minCoords.z == 0);
    bool hasName = !z.name.empty();
    return hasName || !allZero; // Está definida si tiene nombre o valores diferentes de cero
}

std::vector<CubeZone> parseCubeFile(const std::string &filename) {
    std::ifstream file(filename);
    std::vector<CubeZone> zones;
    std::string line;

    if (!file.is_open()) {
        std::cerr << "Error: no se pudo abrir el archivo " << filename << "\n";
        return zones;
    }

    CubeZone zone;
    int lineCounter = 0;
    bool readingZone = false;

    while (std::getline(file, line)) {
        if (!line.empty() && line.back() == '\r') line.pop_back();

        // Detecta inicio de zona
        if (line.rfind("//CUBEINTF", 0) == 0) {
            if (readingZone && isZoneDefined(zone)) { 
                zones.push_back(zone); 
            }
            readingZone = true;
            lineCounter = 0;
            zone = {}; // reiniciar zona

            // Extrae ID
            std::istringstream iss(line.substr(10));
            iss >> zone.id;
        }
        else if (readingZone) {
            lineCounter++;

            switch (lineCounter) {
                case 1: // Nombre
                    if (line.rfind("///NAME", 0) == 0) {
                        zone.name = line.substr(7);
                        zone.name.erase(0, zone.name.find_first_not_of(" ")); // Quita espacios iniciales
                    }
                    break;
                case 2: { // P1, P2, P3
                    std::replace(line.begin(), line.end(), ',', ' ');
                    std::istringstream iss(line);
                    iss >> zone.P1 >> zone.P2 >> zone.P3;
                    break;
                }
                case 3: { // Coordenadas máximas
                    std::replace(line.begin(), line.end(), ',', ' ');
                    std::istringstream iss(line);
                    iss >> zone.maxCoords.x >> zone.maxCoords.y >> zone.maxCoords.z;
                    break;
                }
                case 4: { // Coordenadas mínimas
                    std::replace(line.begin(), line.end(), ',', ' ');
                    std::istringstream iss(line);
                    iss >> zone.minCoords.x >> zone.minCoords.y >> zone.minCoords.z;
                    break;
                }
                case 7: // Fin de bloque útil
                    if (isZoneDefined(zone)) { 
                        zones.push_back(zone); 
                    }
                    readingZone = false;
                    break;
            }
        }
    }

    // Guarda última zona si está definida
    if (readingZone && isZoneDefined(zone)) {
        zones.push_back(zone);
    }

    return zones;
}

int main() {
    // Initialize the window
    const int screenWidth = 800;
    const int screenHeight = 600;
    float MARGIN = screenHeight/32.0f;

    SetConfigFlags(FLAG_MSAA_4X_HINT);
    InitWindow(screenWidth, screenHeight, "aeea");

    float fontSize = 24*screenHeight/600;
    Font font = LoadFontEx("src/fonts/JetBrainsMono/JetBrainsMono-Bold.ttf", fontSize, 0, 250);
    // float fontSize = font.baseSize;//DISPLAY_HEIGHT/20;

    Shader shader = initShader();

    // Set the target FPS
    SetTargetFPS(60);

    Camera camera = { {0.0f, 4.0f, 5.0f}, Vector3Zero(), { 0.0f, 1.0f, 0.0f }, 45.0f, 0 };
    camera.fovy = 80.0f;
    camera.projection = CAMERA_PERSPECTIVE;
    float cameraAngle = 0.0f;
    CameraMode cameraMode = CAMERA_FREE;

    // SetCameraMode(camera, CAMERA_THIRD_PERSON);
	// SetCameraMode(camera, CAMERA_ORBITAL);
    //  SetCameraMode(camera, CAMERA_CUSTOM);
    //  SetCameraMode(camera, CAMERA_FREE);

    float modelScale = 1.0f;
    // Model* robotBaseModel = new Model(LoadModel(std::string("src/mot/motoman_gp12_support/meshes/visual/gp12_base_link.obj").c_str()));
    // robotBaseModel->transform = MatrixRotate((Vector3){1,0,0},-90*DEG2RAD);
    // Model* palletModel = new Model(LoadModel(std::string("src/mod/pallet/pallet1000x1200.obj").c_str()));
    // palletModel->transform = MatrixRotate((Vector3){1,0,0},-90*DEG2RAD);

    Link robotLink[AXIS_NUMBER];
    robotLink[0].origin = (Vector3){0.0f, 0.0f, 0.0f};
    robotLink[1].origin = (Vector3){0.0f, 0.450f, 0.0f};
    robotLink[2].origin = (Vector3){0.155f, 0.0f, 0.0f};
    robotLink[3].origin = (Vector3){0.0f, 0.614f, 0.0f};
    robotLink[4].origin = (Vector3){0.640f, 0.2f, 0.0f};
    robotLink[5].origin = (Vector3){0.0f, 0.0f, 0.0f};
    robotLink[6].origin = (Vector3){0.0f, 0.0f, 0.0f};
    
    for(int i = 0; i < AXIS_NUMBER; i++) {
        std::string objPath = "src/mot/motoman_gp12_support/meshes/visual/gp12_link_" + std::to_string(i) + ".obj";
        robotLink[i].model = new Model(LoadModel(objPath.c_str()));
        robotLink[i].model->materials[0].maps[MATERIAL_MAP_DIFFUSE].color = BLUE;
        robotLink[i].model->materials[0].shader = shader;
        if(i > 0) {
            robotLink[i].parent = &robotLink[i-1];
            robotLink[i].model->transform = robotLink[i].parent->model->transform;
        }else{
            robotLink[i].model->transform = MatrixMultiply(robotLink[i].model->transform, MatrixRotate((Vector3){1,0,0},-90*DEG2RAD));
        }
        robotLink[i].model->transform = MatrixMultiply(robotLink[i].model->transform, MatrixTranslate(robotLink[i].origin.x,robotLink[i].origin.y,robotLink[i].origin.z));
    }
    
    for(int i = 0; i < AXIS_NUMBER; i++) {
        robotLink[i].model->transform = MatrixMultiply(robotLink[i].model->transform, MatrixRotate((Vector3){0,1,0},180*DEG2RAD));
        robotLink[i].model->transform = MatrixMultiply(robotLink[i].model->transform, MatrixRotate((Vector3){0,0,1},30*DEG2RAD));
    }

    // robotBaseModel->materials[0].maps[MATERIAL_MAP_DIFFUSE].color = BLUE;
    // robotBaseModel->materials[0].maps[MATERIAL_MAP_NORMAL].color = WHITE;
    // robotBaseModel->materials[0].maps[MATERIAL_MAP_NORMAL].value = 0.0f;
    // robotBaseModel->materials[0].maps[MATERIAL_MAP_SPECULAR].color = WHITE;
    // robotBaseModel->materials[0].shader = shader;
    // palletModel->materials[0].shader = shader;

    Light lights[MAX_LIGHTS] = { 0 };
    lights[0] = CreateLight(LIGHT_POINT, (Vector3){ 100, 100, 100 }, Vector3Zero(), WHITE, shader);
    lights[0].position = (Vector3){100.0f, 100.0f, 100.0f};

    // Create a RenderTexture2D to be used for render to texture
    RenderTexture2D target = LoadRenderTexture(screenWidth, screenHeight);

    float coordScale = 1.0f/1000.0f;

    // conexión al robot
    StatusInfo status;
    MotomanController* c = YMConnect::OpenConnection("192.168.23.15", status); // Open a connection to the robot controller

    if (status.StatusCode != 0)
    {
        std::cout << status << std::endl;
        return status.StatusCode;
    }

    status = c->Files->SaveFromControllerToFile("UFRAME.CND", "dat/UFRAME.CND", true);
    status = c->Files->SaveFromControllerToFile("CUBEINTF.CND", "dat/CUBEINTF.CND", true);

    auto zones = parseCubeFile("dat/CUBEINTF.CND");

    // Mostrar resultados
    for (auto &z : zones) {
        if(!z.name.empty())
        {
            std::cout << "Zona ID: " << z.id << "\n";
            std::cout << "Nombre: " << z.name << "\n";
            std::cout << "P1,P2,P3: " << z.P1 << ", " << z.P2 << ", " << z.P3 << "\n";
            std::cout << "Max coords: " << z.maxCoords.x << ", " << z.maxCoords.y << ", " << z.maxCoords.z << "\n";
            std::cout << "Min coords: " << z.minCoords.x << ", " << z.minCoords.y << ", " << z.minCoords.z << "\n";
            std::cout << "-----------------------------------\n";

            z.maxCoordsAux.x = -z.minCoords.x;
            z.minCoordsAux.x = -z.maxCoords.x;
            z.maxCoordsAux.y = z.maxCoords.z;
            z.minCoordsAux.y = z.minCoords.z;
            z.maxCoordsAux.z = z.maxCoords.y;
            z.minCoordsAux.z = z.minCoords.y;
            z.minCoordsAux = Vector3Scale(z.minCoordsAux, coordScale/1000.0f);
            z.maxCoordsAux = Vector3Scale(z.maxCoordsAux, coordScale/1000.0f);

            z.position = Vector3Add(z.minCoordsAux,Vector3Scale(Vector3Subtract(z.maxCoordsAux,z.minCoordsAux),0.5));
            z.size = Vector3Subtract(z.maxCoordsAux,z.minCoordsAux);
        }
    }

    zones[0].color = GREEN;
    zones[1].color = YELLOW;
    zones[2].color = ORANGE;
    zones[3].color = RED;

    Point infeederA = { {219.0f, -1309.0f, 1915.0f}, {0.0f, 180.0f, 0.0f} };
    Point outfeederA = { {1756.0f, -1483.0f, 1235.0f}, {0.0f, 90.0f, 0.0f} };
    Point infeederB = { {-1397.0f, 903.0f, 882.0f}, {0.0f, 0.0f, 0.0f} };
    Point outfeederB = { {1749.0f, -1474.0f, -536.0f}, {0.0f, 90.0f, 0.0f} };

    outfeederA.position.x *= -1.0f;
    outfeederA.position = Vector3Scale(outfeederA.position, coordScale);
    outfeederB.position.x *= -1.0f;
    outfeederB.position = Vector3Scale(outfeederB.position, coordScale);
    infeederA.position.x *= -1.0f;
    infeederA.position = Vector3Scale(infeederA.position, coordScale);
    infeederB.position.x *= -1.0f;
    infeederB.position = Vector3Scale(infeederB.position, coordScale);

    Vector3 offset = {0.0f, 1.0f, 0.0f};
    Point auxPoint;
    double radius = 0.0;
    int pointsNum = 5;
    int pointCount = 0;

    offset = {0.0f, 0.5f, 0.0f};
    auxPoint = outfeederA;
    auxPoint.position = Vector3Add(auxPoint.position, offset);
    radius = (Vector3Distance(auxPoint.position,infeederB.position)*0.5+0.001f);
    Trajectory trajectoryOAIA = interpolateCircularSegment(
        auxPoint,
        infeederA,
        (Vector3){0.0f,(auxPoint.position.y+infeederA.position.y)/2.0f,0.0f},
        pointsNum,
        radius
    );

    offset = {0.0f, 0.5f, 0.0f};
    auxPoint = outfeederB;
    auxPoint.position = Vector3Add(auxPoint.position, offset);
    radius = (Vector3Distance(auxPoint.position,infeederB.position)*0.5+0.001f);
    std::cout << "radius: " << radius << std::endl;
    Trajectory trajectoryOBIB = interpolateCircularSegment(
        auxPoint,
        infeederB,
        (Vector3){0.0f,(auxPoint.position.y+infeederB.position.y)/2.0f,0.0f},
        pointsNum,
        radius
    );

    // selección de puntos
    Ray ray = { 0 };        // Picking ray

    // test de botones
    bool *btnWiredState = new bool(DRAW_WIRED);
    btnWiredState = &DRAW_WIRED;
    bool *btnTransparencyState = new bool(false);

    // Main loop
    while (!WindowShouldClose()) {
        // Update logic here
        UpdateCamera(&camera, cameraMode);
        
        SetShaderValue(shader, shader.locs[SHADER_LOC_VECTOR_VIEW], &camera.position.x, SHADER_UNIFORM_VEC3);

        UpdateLightValues(shader, lights[0]);
        
        // Actualiza shader de luz con la posicion de vista de la camara
        float cameraPos[3] = { camera.position.x, camera.position.y, camera.position.z };
        SetShaderValue(shader, shader.locs[SHADER_LOC_VECTOR_VIEW], cameraPos, SHADER_UNIFORM_VEC3);

        if(IsKeyPressed(KEY_O)) {
            cameraMode == CAMERA_FREE ? cameraMode = CAMERA_ORBITAL : cameraMode = CAMERA_FREE;
        }

        if(IsKeyPressed(KEY_LEFT)) {
            radius += 0.05;
            // offset = {0.0f, 1.5f, 0.0f};
            auxPoint = outfeederB;
            auxPoint.position = Vector3Add(auxPoint.position, offset);
            trajectoryOBIB = interpolateCircularSegment(
                auxPoint,
                infeederB,
                (Vector3){0.0f,(auxPoint.position.y+infeederB.position.y)/2.0f,0.0f},
                pointsNum,
                radius
            );
            // offset = {0.0f, 1.5f, 0.0f};
            auxPoint = outfeederA;
            auxPoint.position = Vector3Add(auxPoint.position, offset);
            trajectoryOAIA = interpolateCircularSegment(
                auxPoint,
                infeederA,
                (Vector3){0.0f,(auxPoint.position.y+infeederA.position.y)/2.0f,0.0f},
                pointsNum,
                radius
            );
        }else if (IsKeyPressed(KEY_RIGHT)) {
            radius -= 0.05;
            // offset = {0.0f, 1.5f, 0.0f};
            auxPoint = outfeederB;
            auxPoint.position = Vector3Add(auxPoint.position, offset);
            trajectoryOBIB = interpolateCircularSegment(
                auxPoint,
                infeederB,
                (Vector3){0.0f,(auxPoint.position.y+infeederB.position.y)/2.0f,0.0f},
                pointsNum,
                radius
            );
            // offset = {0.0f, 1.5f, 0.0f};
            auxPoint = outfeederA;
            auxPoint.position = Vector3Add(auxPoint.position, offset);
            trajectoryOAIA = interpolateCircularSegment(
                auxPoint,
                infeederA,
                (Vector3){0.0f,(auxPoint.position.y+infeederA.position.y)/2.0f,0.0f},
                pointsNum,
                radius
            );
        }

        if(IsKeyPressed(KEY_UP)) {
            pointsNum++;
            // offset = {0.0f, 1.5f, 0.0f};
            auxPoint = outfeederB;
            auxPoint.position = Vector3Add(auxPoint.position, offset);
            trajectoryOBIB = interpolateCircularSegment(
                auxPoint,
                infeederB,
                (Vector3){0.0f,(auxPoint.position.y+infeederB.position.y)/2.0f,0.0f},
                pointsNum,
                radius
            );
            // offset = {0.0f, 1.5f, 0.0f};
            auxPoint = outfeederA;
            auxPoint.position = Vector3Add(auxPoint.position, offset);
            trajectoryOAIA = interpolateCircularSegment(
                auxPoint,
                infeederA,
                (Vector3){0.0f,(auxPoint.position.y+infeederA.position.y)/2.0f,0.0f},
                pointsNum,
                radius
            );
        }else if (IsKeyPressed(KEY_DOWN)) {
            pointsNum--;
            // offset = {0.0f, 1.5f, 0.0f};
            auxPoint = outfeederB;
            auxPoint.position = Vector3Add(auxPoint.position, offset);
            trajectoryOBIB = interpolateCircularSegment(
                auxPoint,
                infeederB,
                (Vector3){0.0f,(auxPoint.position.y+infeederB.position.y)/2.0f,0.0f},
                pointsNum,
                radius
            );
            // offset = {0.0f, 1.5f, 0.0f};
            auxPoint = outfeederA;
            auxPoint.position = Vector3Add(auxPoint.position, offset);
            trajectoryOAIA = interpolateCircularSegment(
                auxPoint,
                infeederA,
                (Vector3){0.0f,(auxPoint.position.y+infeederA.position.y)/2.0f,0.0f},
                pointsNum,
                radius
            );
        }

        if(IsKeyPressed(KEY_T)){
            for (auto &z : zones) {
                z.color.a == 255 ? z.color.a = 100 : z.color.a = 255;
            }
        }

        if(IsKeyPressed(KEY_Z)) DRAW_ZONES = !DRAW_ZONES;
        if(IsKeyPressed(KEY_W)) DRAW_WIRED = !DRAW_WIRED;
        if(IsKeyPressed(KEY_TAB)) DRAW_TRAJECTORIES = !DRAW_TRAJECTORIES;

        if(IsKeyPressed(KEY_E)) {
            std::ofstream output_file("dat/100-TRAYECTORIA.JBI");
    
            if (!output_file.is_open()) {
                std::cerr << "Error al abrir archivo para escritura: " << "dat/TRAYECTORIA.JBI" << std::endl;
                return EXIT_FAILURE;
            }

            output_file <<  "/JOB" << "\n"
                            "//NAME 100-TRAYECTORIA" << "\n"
                            "///FOLDERNAME TRAYECTORIAS" << "\n"
                            "//POS" << "\n"
                            "///NPOS 0,0,0,46,0,0" << "\n"
                            "///TOOL 1" << "\n"
                            "///POSTYPE ROBOT" << "\n"
                            "///RECTAN" << "\n"
                            "///RCONF 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0" << "\n";
            
            pointCount = 200;
            for (const auto& point : trajectoryOBIB.points) {
                output_file << std::fixed << std::setprecision(COORD_PRECISION)
                            <<  "P" << pointCount << "="
                            << -point.position.x/coordScale << "," << point.position.z/coordScale << "," << point.position.y/coordScale << ","
                            << point.rotation.x << "," << point.rotation.y << "," << point.rotation.z << "\n";
                pointCount++;
            }

            output_file <<  "//INST" << "\n"
                            "///DATE 2000/01/18 10:00" << "\n"
                            "///ATTR SC,RW" << "\n"
                            "///GROUP1 RB1" << "\n"
                            "///LVARS 0,0,0,0,0,0,0,0" << "\n"
                            "NOP" << "\n"
                            "SET I003 10000" << "\n"
                            "SET I004 30000" << "\n"
                            "SET I005 359" << "\n"
                            "SPEED VJ=I003 V=I004 VR=I005" << "\n";

            pointCount = 200;
            for (const auto& point : trajectoryOBIB.points) {
                output_file << "MOVJ P" << pointCount << "\n";
                pointCount++;
            }

            output_file <<  "END" << "\n" << "";

            output_file.close();
        }

         // Display information about closest hit
        RayCollision collision = { 0 };
        collision.distance = FLT_MAX;
        collision.hit = false;

        // Get ray and test against objects
        ray = GetMouseRay(GetMousePosition(), camera);

        for (auto& point : trajectoryOBIB.points) {
            // Check ray collision against points
            RayCollision sphereHitInfo = GetRayCollisionSphere(ray, point.position, 0.1f);
            if ((sphereHitInfo.hit) && (sphereHitInfo.distance < collision.distance))
            {
                collision = sphereHitInfo;
                point.selected = true;
            }else{
                point.selected = false;
            }
        }

        for (auto& point : trajectoryOAIA.points) {
            // Check ray collision against points
            RayCollision sphereHitInfo = GetRayCollisionSphere(ray, point.position, 0.1f);
            if ((sphereHitInfo.hit) && (sphereHitInfo.distance < collision.distance))
            {
                collision = sphereHitInfo;
                point.selected = true;
            }else{
                point.selected = false;
            }
        }
        

        BeginTextureMode(target);
            ClearBackground(COLOR_BG);
            BeginMode3D(camera);
                DrawGrid(10, 1.0f);
                if(DRAW_TRAJECTORIES)
                {
                    BeginShaderMode(shader);
                    for(int i = 0; i < AXIS_NUMBER; i++) {
                        DrawModel(*robotLink[i].model, Vector3Zero(), modelScale, WHITE);
                    }
                    // DrawModel(*palletModel, outfeederA.position, modelScale, WHITE);
                    // DrawModel(*palletModel, outfeederB.position, modelScale, WHITE);
                    EndShaderMode();
                    for (const auto& point : trajectoryOAIA.points) {
                        if(point.selected)
                            DrawSphere(point.position, 0.1f, COLOR_HL);
                        else
                            DrawSphere(point.position, 0.05f, COLOR_HL2);
                        if(&point != &trajectoryOAIA.points.back())
                            DrawLine3D(point.position, trajectoryOAIA.points[&point - &trajectoryOAIA.points[0] + 1].position, COLOR_HL2);
                    }
                    for (const auto& point : trajectoryOBIB.points) {
                        if(point.selected)
                            DrawSphere(point.position, 0.1f, COLOR_HL);
                        else
                            DrawSphere(point.position, 0.05f, COLOR_HL2);
                        if(&point != &trajectoryOBIB.points.back())
                            DrawLine3D(point.position, trajectoryOBIB.points[&point - &trajectoryOBIB.points[0] + 1].position, COLOR_HL2);
                    }
                }
                if(DRAW_ZONES)
                {
                    if(DRAW_WIRED)
                    {
                        for (auto &z : zones) {
                            DrawCubeWiresV(z.position,z.size,z.color);
                        }
                    }else{
                        for (auto &z : zones) {
                            DrawCubeV(z.position, z.size, z.color);
                        }
                    }
                }
            EndMode3D();
        EndTextureMode();
        
        // Start drawing
        BeginDrawing();
        ClearBackground(RAYWHITE);

            // NOTE: Render texture must be y-flipped due to default OpenGL coordinates (left-bottom)
            DrawTextureRec(target.texture, (Rectangle){ 0, 0, (float)target.texture.width, (float)-target.texture.height }, (Vector2){ 0, 0 }, WHITE);
            
            // DRAW_WIRED = GuiButton((Rectangle){ 10, 10, 120, 30 }, "Wired");
            // GuiToggle((Rectangle){ 10, 10, 120, 30 }, "Wired", btnWiredState);
            // GuiButton((Rectangle){ 10, 50, 120, 30 }, "Exportar Trayectoria");

            // std::string radiusText = "Radius: " + std::to_string(radius);
            // DrawText(radiusText.c_str(), 10, 10, 8, DARKGRAY);
            if(DRAW_ZONES){
                DrawRectangleRoundedRadius((Rectangle){MARGIN,MARGIN,15*fontSize,MARGIN*2+fontSize*zones.size()}, fontSize/2.0f, 5, (Color{0,0,0,28}));
                // DrawRectangle(5,5,15*fontSize,10+fontSize*7+20,(Color{0,0,0,28}));
                for (auto &z : zones) {
                    std::string zoneText = "Zona " + std::to_string(z.id) + ": " + z.name;
                    DrawTextEx(font, zoneText.c_str(), (Vector2){MARGIN*2, MARGIN*2+fontSize*(z.id-1)}, fontSize, 1, z.color);
                }
            }else{
                // DrawTextEx(font, TextFormat("Hit Object: %s", hitObjectName), (Vector2){10, 10}, fontSize, 1, DARKGRAY);
                // DrawTextEx(font, "E: Exportar trayectoria", (Vector2){10, 10}, fontSize, 1, COLOR_FG);
                // DrawTextEx(font, "Z: Dibujar zonas", (Vector2){10, 10+fontSize*1}, fontSize, 1, COLOR_FG);
                // DrawTextEx(font, "W: Modo Wired", (Vector2){10, 10+fontSize*2}, fontSize, 1, COLOR_FG);
                // DrawTextEx(font, "T: Modo transparente", (Vector2){10, 10+fontSize*3}, fontSize, 1, COLOR_FG);
                // DrawTextEx(font, "Arr/Aba: Puntos", (Vector2){10, 10+fontSize*4}, fontSize, 1, COLOR_FG);
                // DrawTextEx(font, "Izq/Der: Radio", (Vector2){10, 10+fontSize*5}, fontSize, 1, COLOR_FG);
            }

        // End drawing
        EndDrawing();
    }

    for(int i = 0; i < AXIS_NUMBER; i++) {
        UnloadModel(*robotLink[i].model);
    }
    // UnloadModel(*palletModel);

    UnloadShader(shader);

    // Close the window and clean up resources
    CloseWindow();

    return 0;
}

Shader initShader(void)
{
    // Load shader and set up some uniforms--------------------------------------------------------------
    Shader shader = LoadShader("../src/sha/basic_lighting.vs", 
                               "../src/sha/lighting.fs");
    shader.locs[SHADER_LOC_MATRIX_MODEL] = GetShaderLocation(shader, "matModel");
    shader.locs[SHADER_LOC_VECTOR_VIEW] = GetShaderLocation(shader, "viewPos");

    int ambientLoc = GetShaderLocation(shader, "ambient");
    float aux[4] = {1.0f, 1.0f, 1.0f, 1.0f};
    SetShaderValue(shader, ambientLoc, aux, SHADER_UNIFORM_VEC4);

    return shader;
}

// Draw rectangle with rounded edges
void DrawRectangleRoundedRadius(Rectangle rec, float radius, int segments, Color color)
{
    rec.width -= radius*0.4f;
    rec.height -= radius*0.4f;
    rec.x += radius*0.2f;
    rec.y += radius*0.2f;

    // Not a rounded rectangle
    if ((radius <= 0.0f) || (rec.width < 1) || (rec.height < 1 ))
    {
        DrawRectangleRec(rec, color);
        return;
    }

    if (radius <= 0.0f) return;

    // Calculate number of segments to use for the corners
    if (segments < 4)
    {
        // Calculate the maximum angle between segments based on the error rate (usually 0.5f)
        float th = acosf(2*powf(1 - SMOOTH_CIRCLE_ERROR_RATE/radius, 2) - 1);
        segments = (int)(ceilf(2*PI/th)/4.0f);
        if (segments <= 0) segments = 4;
    }

    float stepLength = 90.0f/(float)segments;

    /*
    Quick sketch to make sense of all of this,
    there are 9 parts to draw, also mark the 12 points we'll use

          P0____________________P1
          /|                    |\
         /1|          2         |3\
     P7 /__|____________________|__\ P2
       |   |P8                P9|   |
       | 8 |          9         | 4 |
       | __|____________________|__ |
     P6 \  |P11              P10|  / P3
         \7|          6         |5/
          \|____________________|/
          P5                    P4
    */
    // Coordinates of the 12 points that define the rounded rect
    const Vector2 point[12] = {
        {(float)rec.x + radius, rec.y}, {(float)(rec.x + rec.width) - radius, rec.y}, { rec.x + rec.width, (float)rec.y + radius },     // PO, P1, P2
        {rec.x + rec.width, (float)(rec.y + rec.height) - radius}, {(float)(rec.x + rec.width) - radius, rec.y + rec.height},           // P3, P4
        {(float)rec.x + radius, rec.y + rec.height}, { rec.x, (float)(rec.y + rec.height) - radius}, {rec.x, (float)rec.y + radius},    // P5, P6, P7
        {(float)rec.x + radius, (float)rec.y + radius}, {(float)(rec.x + rec.width) - radius, (float)rec.y + radius},                   // P8, P9
        {(float)(rec.x + rec.width) - radius, (float)(rec.y + rec.height) - radius}, {(float)rec.x + radius, (float)(rec.y + rec.height) - radius} // P10, P11
    };

    const Vector2 centers[4] = { point[8], point[9], point[10], point[11] };
    const float angles[4] = { 180.0f, 90.0f, 0.0f, 270.0f };

#if defined(SUPPORT_QUADS_DRAW_MODE)
    rlCheckRenderBatchLimit(16*segments/2 + 5*4);

    rlSetTexture(texShapes.id);

    rlBegin(RL_QUADS);
        // Draw all of the 4 corners: [1] Upper Left Corner, [3] Upper Right Corner, [5] Lower Right Corner, [7] Lower Left Corner
        for (int k = 0; k < 4; ++k) // Hope the compiler is smart enough to unroll this loop
        {
            float angle = angles[k];
            const Vector2 center = centers[k];

            // NOTE: Every QUAD actually represents two segments
            for (int i = 0; i < segments/2; i++)
            {
                rlColor4ub(color.r, color.g, color.b, color.a);
                rlTexCoord2f(texShapesRec.x/texShapes.width, texShapesRec.y/texShapes.height);
                rlVertex2f(center.x, center.y);
                rlTexCoord2f(texShapesRec.x/texShapes.width, (texShapesRec.y + texShapesRec.height)/texShapes.height);
                rlVertex2f(center.x + sinf(DEG2RAD*angle)*radius, center.y + cosf(DEG2RAD*angle)*radius);
                rlTexCoord2f((texShapesRec.x + texShapesRec.width)/texShapes.width, (texShapesRec.y + texShapesRec.height)/texShapes.height);
                rlVertex2f(center.x + sinf(DEG2RAD*(angle + stepLength))*radius, center.y + cosf(DEG2RAD*(angle + stepLength))*radius);
                rlTexCoord2f((texShapesRec.x + texShapesRec.width)/texShapes.width, texShapesRec.y/texShapes.height);
                rlVertex2f(center.x + sinf(DEG2RAD*(angle + stepLength*2))*radius, center.y + cosf(DEG2RAD*(angle + stepLength*2))*radius);
                angle += (stepLength*2);
            }

            // NOTE: In case number of segments is odd, we add one last piece to the cake
            if (segments%2)
            {
                rlColor4ub(color.r, color.g, color.b, color.a);
                rlTexCoord2f(texShapesRec.x/texShapes.width, texShapesRec.y/texShapes.height);
                rlVertex2f(center.x, center.y);
                rlTexCoord2f(texShapesRec.x/texShapes.width, (texShapesRec.y + texShapesRec.height)/texShapes.height);
                rlVertex2f(center.x + sinf(DEG2RAD*angle)*radius, center.y + cosf(DEG2RAD*angle)*radius);
                rlTexCoord2f((texShapesRec.x + texShapesRec.width)/texShapes.width, (texShapesRec.y + texShapesRec.height)/texShapes.height);
                rlVertex2f(center.x + sinf(DEG2RAD*(angle + stepLength))*radius, center.y + cosf(DEG2RAD*(angle + stepLength))*radius);
                rlTexCoord2f((texShapesRec.x + texShapesRec.width)/texShapes.width, texShapesRec.y/texShapes.height);
                rlVertex2f(center.x, center.y);
            }
        }

        // [2] Upper Rectangle
        rlColor4ub(color.r, color.g, color.b, color.a);
        rlTexCoord2f(texShapesRec.x/texShapes.width, texShapesRec.y/texShapes.height);
        rlVertex2f(point[0].x, point[0].y);
        rlTexCoord2f(texShapesRec.x/texShapes.width, (texShapesRec.y + texShapesRec.height)/texShapes.height);
        rlVertex2f(point[8].x, point[8].y);
        rlTexCoord2f((texShapesRec.x + texShapesRec.width)/texShapes.width, (texShapesRec.y + texShapesRec.height)/texShapes.height);
        rlVertex2f(point[9].x, point[9].y);
        rlTexCoord2f((texShapesRec.x + texShapesRec.width)/texShapes.width, texShapesRec.y/texShapes.height);
        rlVertex2f(point[1].x, point[1].y);

        // [4] Right Rectangle
        rlColor4ub(color.r, color.g, color.b, color.a);
        rlTexCoord2f(texShapesRec.x/texShapes.width, texShapesRec.y/texShapes.height);
        rlVertex2f(point[2].x, point[2].y);
        rlTexCoord2f(texShapesRec.x/texShapes.width, (texShapesRec.y + texShapesRec.height)/texShapes.height);
        rlVertex2f(point[9].x, point[9].y);
        rlTexCoord2f((texShapesRec.x + texShapesRec.width)/texShapes.width, (texShapesRec.y + texShapesRec.height)/texShapes.height);
        rlVertex2f(point[10].x, point[10].y);
        rlTexCoord2f((texShapesRec.x + texShapesRec.width)/texShapes.width, texShapesRec.y/texShapes.height);
        rlVertex2f(point[3].x, point[3].y);

        // [6] Bottom Rectangle
        rlColor4ub(color.r, color.g, color.b, color.a);
        rlTexCoord2f(texShapesRec.x/texShapes.width, texShapesRec.y/texShapes.height);
        rlVertex2f(point[11].x, point[11].y);
        rlTexCoord2f(texShapesRec.x/texShapes.width, (texShapesRec.y + texShapesRec.height)/texShapes.height);
        rlVertex2f(point[5].x, point[5].y);
        rlTexCoord2f((texShapesRec.x + texShapesRec.width)/texShapes.width, (texShapesRec.y + texShapesRec.height)/texShapes.height);
        rlVertex2f(point[4].x, point[4].y);
        rlTexCoord2f((texShapesRec.x + texShapesRec.width)/texShapes.width, texShapesRec.y/texShapes.height);
        rlVertex2f(point[10].x, point[10].y);

        // [8] Left Rectangle
        rlColor4ub(color.r, color.g, color.b, color.a);
        rlTexCoord2f(texShapesRec.x/texShapes.width, texShapesRec.y/texShapes.height);
        rlVertex2f(point[7].x, point[7].y);
        rlTexCoord2f(texShapesRec.x/texShapes.width, (texShapesRec.y + texShapesRec.height)/texShapes.height);
        rlVertex2f(point[6].x, point[6].y);
        rlTexCoord2f((texShapesRec.x + texShapesRec.width)/texShapes.width, (texShapesRec.y + texShapesRec.height)/texShapes.height);
        rlVertex2f(point[11].x, point[11].y);
        rlTexCoord2f((texShapesRec.x + texShapesRec.width)/texShapes.width, texShapesRec.y/texShapes.height);
        rlVertex2f(point[8].x, point[8].y);

        // [9] Middle Rectangle
        rlColor4ub(color.r, color.g, color.b, color.a);
        rlTexCoord2f(texShapesRec.x/texShapes.width, texShapesRec.y/texShapes.height);
        rlVertex2f(point[8].x, point[8].y);
        rlTexCoord2f(texShapesRec.x/texShapes.width, (texShapesRec.y + texShapesRec.height)/texShapes.height);
        rlVertex2f(point[11].x, point[11].y);
        rlTexCoord2f((texShapesRec.x + texShapesRec.width)/texShapes.width, (texShapesRec.y + texShapesRec.height)/texShapes.height);
        rlVertex2f(point[10].x, point[10].y);
        rlTexCoord2f((texShapesRec.x + texShapesRec.width)/texShapes.width, texShapesRec.y/texShapes.height);
        rlVertex2f(point[9].x, point[9].y);

    rlEnd();
    rlSetTexture(0);
#else
    rlCheckRenderBatchLimit(12*segments + 5*6); // 4 corners with 3 vertices per segment + 5 rectangles with 6 vertices each

    rlBegin(RL_TRIANGLES);

        // Draw all of the 4 corners: [1] Upper Left Corner, [3] Upper Right Corner, [5] Lower Right Corner, [7] Lower Left Corner
        for (int k = 0; k < 4; ++k) // Hope the compiler is smart enough to unroll this loop
        {
            float angle = angles[k];
            const Vector2 center = centers[k];
            for (int i = 0; i < segments; i++)
            {
                rlColor4ub(color.r, color.g, color.b, color.a);
                rlVertex2f(center.x, center.y);
                rlVertex2f(center.x + sinf(DEG2RAD*angle)*radius, center.y + cosf(DEG2RAD*angle)*radius);
                rlVertex2f(center.x + sinf(DEG2RAD*(angle + stepLength))*radius, center.y + cosf(DEG2RAD*(angle + stepLength))*radius);
                angle += stepLength;
            }
        }

        // [2] Upper Rectangle
        rlColor4ub(color.r, color.g, color.b, color.a);
        rlVertex2f(point[0].x, point[0].y);
        rlVertex2f(point[8].x, point[8].y);
        rlVertex2f(point[9].x, point[9].y);
        rlVertex2f(point[1].x, point[1].y);
        rlVertex2f(point[0].x, point[0].y);
        rlVertex2f(point[9].x, point[9].y);

        // [4] Right Rectangle
        rlColor4ub(color.r, color.g, color.b, color.a);
        rlVertex2f(point[9].x, point[9].y);
        rlVertex2f(point[10].x, point[10].y);
        rlVertex2f(point[3].x, point[3].y);
        rlVertex2f(point[2].x, point[2].y);
        rlVertex2f(point[9].x, point[9].y);
        rlVertex2f(point[3].x, point[3].y);

        // [6] Bottom Rectangle
        rlColor4ub(color.r, color.g, color.b, color.a);
        rlVertex2f(point[11].x, point[11].y);
        rlVertex2f(point[5].x, point[5].y);
        rlVertex2f(point[4].x, point[4].y);
        rlVertex2f(point[10].x, point[10].y);
        rlVertex2f(point[11].x, point[11].y);
        rlVertex2f(point[4].x, point[4].y);

        // [8] Left Rectangle
        rlColor4ub(color.r, color.g, color.b, color.a);
        rlVertex2f(point[7].x, point[7].y);
        rlVertex2f(point[6].x, point[6].y);
        rlVertex2f(point[11].x, point[11].y);
        rlVertex2f(point[8].x, point[8].y);
        rlVertex2f(point[7].x, point[7].y);
        rlVertex2f(point[11].x, point[11].y);

        // [9] Middle Rectangle
        rlColor4ub(color.r, color.g, color.b, color.a);
        rlVertex2f(point[8].x, point[8].y);
        rlVertex2f(point[11].x, point[11].y);
        rlVertex2f(point[10].x, point[10].y);
        rlVertex2f(point[9].x, point[9].y);
        rlVertex2f(point[8].x, point[8].y);
        rlVertex2f(point[10].x, point[10].y);
    rlEnd();
#endif
}