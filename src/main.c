// https://www.red3d.com/cwr/boids/
#include "raylib.h"
#include "raymath.h"
#include <stdbool.h>


#define WINDOW_WIDTH 2048
#define WINDOW_HEIGHT 1024
#define MAX_FPS 0

#define NUM_BOIDS 1024
#define BOID_SIZE 4
#define VIEW_DISTANCE 64
#define VIEW_DISTANCE_SQR (VIEW_DISTANCE * VIEW_DISTANCE)
#define AVOID_DISTANCE 16
#define AVOID_DISTANCE_SQR (AVOID_DISTANCE * AVOID_DISTANCE)
#define VIEW_DOT_PRODUCT -0.6
#define SEPARATION_CONSTANT 0.03  //0.03
#define ALIGNMENT_CONSTANT 0.01  //0.01
#define COHESION_CONSTANT 0.05  //0.05
#define AVOIDANCE_CONSTANT 10
#define MOVE_SPEED 0.3

#define MAX_AREA_RADIUS 256
#define AREA_SIZE_CHANGE 10

#define GRID_HALF_SIZE VIEW_DISTANCE
#define GRID_SIZE (GRID_HALF_SIZE * 2)
#define GRID_WIDTH (WINDOW_WIDTH / GRID_SIZE)
#define GRID_HEIGHT (WINDOW_HEIGHT / GRID_SIZE)
#define GRID_CELLS (GRID_WIDTH * GRID_HEIGHT)


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

    // Using a linked list would make using SIMD impossible but, hopefully,
    //  should heavily reduce number of collision checks between boids from.
    //  time: O(n^2), space: O(n) -->
    //  time: O(n), space: O(n) but n's of higher magnitudes
    // An alternative or addition to this would be multithreading but I would
    //  like to keep processing to one thread for compatability and simplicity.
    int links[NUM_BOIDS] = {};
    int link_heads[GRID_CELLS] = {};

    for (int i = 0; i < GRID_CELLS; i++) {
        link_heads[i] = -1;
    }

    for (int i = 0; i < NUM_BOIDS; i++) {
        Vector2 position = (Vector2) {
            GetRandomValue(0, WINDOW_WIDTH),
            GetRandomValue(0, WINDOW_HEIGHT)
        };
        Vector2 direction = Vector2Normalize((Vector2) {
            GetRandomValue(-64, 64),
            GetRandomValue(-64, 64)
        });
        boids[i] = (Boid) {position, direction};

        int x_grid = (int) position.x / GRID_SIZE;
        int y_grid = (int) position.y / GRID_SIZE;
        int i_grid = y_grid * GRID_WIDTH + x_grid;

        if (link_heads[i_grid] == -1) {
            link_heads[i_grid] = i;
            links[i] = -1;
        } else {
            // Insert at head because easier
            links[i] = link_heads[i_grid];
            link_heads[i_grid] = i;
        }
    }
    /*for (int i = 0; i < GRID_CELLS; i++) {*/
    /*    printf("%d, ", link_heads[i]);*/
    /*}*/
    /*printf("\n");*/
    /*for (int i = 0; i < NUM_BOIDS; i++) {*/
    /*    printf("%d, ", links[i]);*/
    /*}*/
    /*printf("\n");*/

    Vector2 offscreen = (Vector2) {-WINDOW_WIDTH, -WINDOW_HEIGHT};
    Vector2 area_position = offscreen;
    bool is_area_attract = false;

    Vector2 average_positions[NUM_BOIDS] = {};
    Vector2 average_directions[NUM_BOIDS] = {};
    Vector2 average_separations[NUM_BOIDS] = {};

    while (!WindowShouldClose()) {
        // Update area
        if (IsCursorOnScreen()) {
            area_position = GetMousePosition();
        } else {
            area_position = offscreen;
        }

        float mouse_wheel = GetMouseWheelMove();
        if (mouse_wheel) {
            area_radius += mouse_wheel * AREA_SIZE_CHANGE;
            area_radius = Clamp(area_radius, 0, MAX_AREA_RADIUS);
        }
        if (IsMouseButtonPressed(0)) {
            is_area_attract = is_area_attract ? false : true;
        }

        // Update Boids
        for (int i = 0; i < GRID_CELLS; i++) {
            if (link_heads[i] == -1) continue;

            int current = link_heads[i];
            while (current != -1) {
                // TODO: For accuracy check surrounding grid cells and perform
                //  calculations. Currently just checking withing cell.

                // Calculate surrounding grid cells
                int x_grid = (int) boids[current].position.x / GRID_SIZE;
                int y_grid = (int) boids[current].position.y / GRID_SIZE;

                int horizontal = 0;  // -1, 0 , 1
                int vertical = 0;  // -1, 0 , 1
                int remaining_x = (int) boids[current].position.x - x_grid * GRID_SIZE;
                int remaining_y = (int) boids[current].position.y - y_grid * GRID_SIZE;

                if (remaining_x > GRID_HALF_SIZE) horizontal++;
                else if (remaining_x < GRID_HALF_SIZE) horizontal--;
                if (remaining_y > GRID_HALF_SIZE) vertical++;
                else if (remaining_y < GRID_HALF_SIZE) vertical--;

                int count = 0;
                int separation_count = 0;
                Vector2 average_position = Vector2Zero();
                Vector2 average_direction = Vector2Zero();
                Vector2 average_separation = Vector2Zero();

                int inside = link_heads[i];
                while ((inside != -1)) {
                    if (inside == current) {
                        inside = links[inside];
                        continue;
                    }
                    float distance_sqr = Vector2DistanceSqr(boids[current].position, boids[inside].position);
                    if (distance_sqr > VIEW_DISTANCE_SQR) {
                        inside = links[inside];
                        continue;
                    }
                    if (Vector2DotProduct(boids[current].position, boids[inside].position) < VIEW_DOT_PRODUCT) {
                        inside = links[inside];
                        continue;
                    }
                    average_position = Vector2Add(average_position, boids[inside].position);
                    average_direction = Vector2Add(average_direction, boids[inside].direction);
                    count++;
                    average_separation = Vector2Subtract(average_separation, Vector2Scale(Vector2Subtract(boids[current].position, boids[inside].position), 1.0f / distance_sqr));
                    separation_count++;

                    inside = links[inside];
                }

                average_positions[current] = Vector2Scale(average_position, 1.0f / count);
                average_directions[current] = Vector2Normalize(Vector2Scale(average_direction, 1.0f / count));
                average_separations[current] = Vector2Normalize(Vector2Scale(average_separation, 1.0f / separation_count));

                current = links[current];
            }
        }

        // Move the boids
        for (int i = 0; i < NUM_BOIDS; i++) {
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

        // Update boids linked list
        for (int i = 0; i < GRID_CELLS; i++) {
            if (link_heads[i] == -1) continue;

            int current = link_heads[i];
            while (current != -1) {
                int x_grid = (int) boids[current].position.x / GRID_SIZE;
                int y_grid = (int) boids[current].position.y / GRID_SIZE;
                int i_grid = y_grid * GRID_WIDTH + x_grid;

                // Boid still in valid grid cell
                if (i == i_grid) {
                    current = links[current];
                    continue;
                }

                // We gotta move the boid from this linked list to another
                // Move to head of other linked list for simplicity

                // Move head
                if (current == link_heads[i]) {
                    link_heads[i] = links[current];

                    // Current points to new head
                    links[current] = link_heads[i_grid];
                    // Head is now current of correct grid cell
                    link_heads[i_grid] = current;

                    current = link_heads[i];
                } else {
                    int old = links[current];
                    // Patch up link, set current to be next
                    links[current] = links[old];

                    // Redundant link points to new head, -1 is fine.
                    links[old] = link_heads[i_grid];
                    // Head is now current of correct grid cell
                    link_heads[i_grid] = old;

                    current = links[current];
                }
            }
        }

        // RENDERING
        BeginDrawing();
        ClearBackground(BLACK);
        // Draw grid
        for (int y = 0; y < GRID_HEIGHT; y++) {
            for (int x = 0; x < GRID_WIDTH; x++) {
                Rectangle rect = (Rectangle) {
                    x * GRID_SIZE,
                    y * GRID_SIZE,
                    GRID_SIZE,
                    GRID_SIZE
                };
                DrawRectangleLinesEx(rect, 1, WHITE);
            }
        }

        // Draw boids
        for (int i = 0; i < NUM_BOIDS; i++) {
            /*DrawCircleLinesV(boids[i].position, VIEW_DISTANCE, GREEN);*/
            /*DrawCircleLinesV(boids[i].position, AVOID_DISTANCE, BLUE);*/
            /*DrawLineV(boids[i].position, Vector2Add(boids[i].position, Vector2Scale(boids[i].direction, AVOID_DISTANCE)), BLUE);*/
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
