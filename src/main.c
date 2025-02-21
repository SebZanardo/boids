// https://www.red3d.com/cwr/boids/
#include "raylib.h"
#include "raymath.h"
#include <stdbool.h>


#define WINDOW_WIDTH 800
#define WINDOW_HEIGHT 800
#define MAX_FPS 360

#define MAX_AREA_RADIUS 256

#define NUM_BOIDS 1024
#define BOID_SIZE 4
#define VIEW_DISTANCE 64
#define VIEW_DISTANCE_SQR VIEW_DISTANCE * VIEW_DISTANCE
#define AVOID_DISTANCE 16
#define AVOID_DISTANCE_SQR AVOID_DISTANCE * AVOID_DISTANCE
#define VIEW_DOT_PRODUCT -0.6
#define SEPARATION_CONSTANT 0.05
#define ALIGNMENT_CONSTANT 0.02
#define COHESION_CONSTANT 0.03
#define AVOIDANCE_CONSTANT 5
#define MOVE_SPEED 0.3


typedef struct {
    Vector2 position;
    Vector2 direction;
} Boid;


int main(void)
{
    InitWindow(WINDOW_WIDTH, WINDOW_HEIGHT, "boids");
    SetTargetFPS(MAX_FPS);
    SetRandomSeed(0);

    int area_radius = MAX_AREA_RADIUS / 2;

    Boid boids[NUM_BOIDS] = {};

    int i = 0;
    for (i = 0; i < NUM_BOIDS; i++) {
        Vector2 position = (Vector2) {
            GetRandomValue(0, WINDOW_WIDTH),
            GetRandomValue(0, WINDOW_HEIGHT)
        };
        Vector2 direction = Vector2Normalize((Vector2) {
            GetRandomValue(-64, 64),
            GetRandomValue(-64, 64)
        });
        boids[i] = (Boid) {position, direction};
    }

    Vector2 offscreen = (Vector2) {-WINDOW_WIDTH, -WINDOW_HEIGHT};
    Vector2 area_position = offscreen;
    bool is_area_attract = false;

    while (!WindowShouldClose()) {
        // Update area
        if (IsCursorOnScreen()) {
            area_position = GetMousePosition();
        } else {
            area_position = offscreen;
        }

        float mouse_wheel = GetMouseWheelMove();
        if (mouse_wheel) {
            area_radius += mouse_wheel * 5;
            area_radius = Clamp(area_radius, 0, MAX_AREA_RADIUS);
        }
        if (IsMouseButtonPressed(0)) {
            is_area_attract = is_area_attract ? false : true;
        }

        // Update Boids
        Vector2 average_positions[NUM_BOIDS] = {};
        Vector2 average_directions[NUM_BOIDS] = {};
        Vector2 average_separations[NUM_BOIDS] = {};
        for (i = 0; i < NUM_BOIDS; i++) {
            int count = 0;
            int separation_count = 0;
            Vector2 average_position = Vector2Zero();
            Vector2 average_direction = Vector2Zero();
            Vector2 average_separation = Vector2Zero();
            for (int j = 0; j < NUM_BOIDS; j++) {
                if (i == j) continue;
                float distance_sqr = Vector2DistanceSqr(boids[i].position, boids[j].position);
                if (distance_sqr > VIEW_DISTANCE_SQR) continue;
                if (Vector2DotProduct(boids[i].position, boids[j].position) < VIEW_DOT_PRODUCT) continue;
                average_position = Vector2Add(average_position, boids[j].position);
                average_direction = Vector2Add(average_direction, boids[j].direction);
                count++;
                if (distance_sqr > VIEW_DISTANCE_SQR) continue;
                average_separation = Vector2Subtract(average_separation, Vector2Scale(Vector2Subtract(boids[i].position, boids[j].position), 1.0f / distance_sqr));
                separation_count++;
            }
            average_positions[i] = Vector2Scale(average_position, 1.0f / count);
            average_directions[i] = Vector2Normalize(Vector2Scale(average_direction, 1.0f / count));
            average_separations[i] = Vector2Normalize(Vector2Scale(average_separation, 1.0f / separation_count));
        }

        for (i = 0; i < NUM_BOIDS; i++) {
            // Separation
            Vector2 separation = Vector2Scale(Vector2Normalize(Vector2Subtract(average_separations[i], boids[i].direction)), -SEPARATION_CONSTANT);

            // Alignment
            Vector2 alignment = Vector2Scale(Vector2Normalize(Vector2Subtract(average_directions[i], boids[i].direction)), ALIGNMENT_CONSTANT);

            // Cohension
            Vector2 cohesion = Vector2Scale(Vector2Normalize(Vector2Subtract(average_positions[i], boids[i].position)), COHESION_CONSTANT);

            // Avoidance
            float distance_sqr = Vector2DistanceSqr(boids[i].position, area_position);
            Vector2 avoidance = Vector2Zero();
            if (distance_sqr < area_radius * area_radius + AVOID_DISTANCE_SQR) {
                avoidance = Vector2Scale(Vector2Subtract(boids[i].position, area_position), 1.0f / distance_sqr * AVOIDANCE_CONSTANT);
                if (is_area_attract) avoidance = Vector2Negate(avoidance);
            }

            boids[i].direction = Vector2Normalize(Vector2Add(boids[i].direction, separation));
            boids[i].direction = Vector2Normalize(Vector2Add(boids[i].direction, alignment));
            boids[i].direction = Vector2Normalize(Vector2Add(boids[i].direction, cohesion));
            boids[i].direction = Vector2Normalize(Vector2Add(boids[i].direction, avoidance));

            // Update position
            boids[i].position = Vector2Add(boids[i].position, Vector2Scale(boids[i].direction, MOVE_SPEED));

            boids[i].position.x = Wrap(boids[i].position.x, 0, WINDOW_WIDTH);
            boids[i].position.y = Wrap(boids[i].position.y, 0, WINDOW_HEIGHT);
        }

        BeginDrawing();
        ClearBackground(BLACK);
        for (i = 0; i < NUM_BOIDS; i++) {
            /*DrawCircleLinesV(boids[i].position, VIEW_DISTANCE, GREEN);*/
            /*DrawCircleLinesV(boids[i].position, AVOID_DISTANCE, RED);*/
            /*DrawLineV(boids[i].position, Vector2Add(boids[i].position, Vector2Scale(boids[i].direction, 32)), BLUE);*/
            /*DrawCircleV(boids[i].position, BOID_SIZE, MAGENTA);*/
            DrawTriangleLines((Vector2) {boids[i].position.x - 8, boids[i].position.y + 4}, (Vector2) {boids[i].position.x + 8, boids[i].position.y + 4}, Vector2Add(boids[i].position, Vector2Scale(boids[i].direction, AVOID_DISTANCE)), MAGENTA);
        }

        DrawCircleLinesV(area_position, area_radius, is_area_attract ? BLUE : RED);

        EndDrawing();

        DrawFPS(0, 0);
    }

    CloseWindow();

    return 0;
}
