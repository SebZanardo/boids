// https://www.red3d.com/cwr/boids/
#include <stdlib.h>
#include <stdbool.h>
#include "raylib.h"
#include "raymath.h"


const int WINDOW_WIDTH = 2049;
const int WINDOW_HEIGHT = 1024;
const int MAX_FPS = 120;
const float FIXED_DT = 1.0f / MAX_FPS;

const int NUM_BOIDS = 4096;
const int BOID_SIZE = 4;
const int VIEW_DISTANCE = 32;
const int VIEW_DISTANCE_SQR = VIEW_DISTANCE * VIEW_DISTANCE;
const int AVOID_DISTANCE = 8;
const int AVOID_DISTANCE_SQR = AVOID_DISTANCE * AVOID_DISTANCE;
const float VIEW_DOT_PRODUCT = -0.6f;
const float SEPARATION_CONSTANT = 0.1f;
const float ALIGNMENT_CONSTANT = 0.01f;
const float COHESION_CONSTANT = 0.02f;
const float ESCAPE_FACTOR = 10.0f;
const float MOVE_SPEED = 100.0f;

// Stop updating boids if cells after depth reached
const int MAX_CELL_DEPTH = 32;
// Stop comparing to neighbours in surrounding cell after depth reached
const int MAX_DEPTH = 32;

const int MAX_AREA_RADIUS = 512;
const int AREA_SIZE_CHANGE = 10;

const int GRID_HALF_SIZE = VIEW_DISTANCE;
const int GRID_SIZE = GRID_HALF_SIZE * 2;
const int GRID_WIDTH = WINDOW_WIDTH / GRID_SIZE;
const int GRID_HEIGHT = WINDOW_HEIGHT / GRID_SIZE;
const int GRID_CELLS = GRID_WIDTH * GRID_HEIGHT;


typedef struct {
    Vector2 position;
    Vector2 direction;
    int next;
} Boid;


int main(void)
{
    InitWindow(WINDOW_WIDTH, WINDOW_HEIGHT, "boids");
    SetTargetFPS(MAX_FPS);
    SetRandomSeed(0);

    int area_radius = MAX_AREA_RADIUS / 2;

    Boid *boids = (Boid *)malloc(sizeof(Boid) * NUM_BOIDS);

    // Using a linked list would make using SIMD impossible but, hopefully,
    //  should heavily reduce number of collision checks between boids from.
    //  time: O(n^2), space: O(n) -->
    //  time: O(n), space: O(n) but n's of higher magnitudes
    // An alternative or addition to this would be multithreading but I would
    //  like to keep processing to one thread for compatability and simplicity.
    int *link_heads = (int *)malloc(sizeof(int) * GRID_CELLS);

    // Setting up boids lists
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
        boids[i] = (Boid) {position, direction, -1};

        int x_grid = (int) position.x / GRID_SIZE;
        x_grid = Wrap(x_grid, 0, GRID_WIDTH);

        int y_grid = (int) position.y / GRID_SIZE;
        y_grid = Wrap(y_grid, 0, GRID_HEIGHT);

        int i_grid = y_grid * GRID_WIDTH + x_grid;

        if (link_heads[i_grid] == -1) {
            // There is no head of the list for this grid cell
            link_heads[i_grid] = i;
        } else {
            // Insert node as new head
            boids[i].next = link_heads[i_grid];
            link_heads[i_grid] = i;
        }
    }

    Vector2 offscreen = (Vector2) {-WINDOW_WIDTH, -WINDOW_HEIGHT};
    Vector2 area_position = offscreen;
    bool is_area_attract = false;

    Vector2 *average_positions = (Vector2 *)malloc(sizeof(Vector2) * NUM_BOIDS);
    Vector2 *average_directions = (Vector2 *)malloc(sizeof(Vector2) * NUM_BOIDS);
    Vector2 *average_separations = (Vector2 *)malloc(sizeof(Vector2) * NUM_BOIDS);

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
            int cell_depth = 0;
            while (current != -1) {
                cell_depth++;
                if (cell_depth > MAX_CELL_DEPTH) break;

                // Calculate surrounding grid cells
                int x_grid = (int) boids[current].position.x / GRID_SIZE;
                x_grid = Wrap(x_grid, 0, GRID_WIDTH);
                int y_grid = (int) boids[current].position.y / GRID_SIZE;
                y_grid = Wrap(y_grid, 0, GRID_HEIGHT);

                int remaining_x = (int) boids[current].position.x - x_grid * GRID_SIZE;
                int remaining_y = (int) boids[current].position.y - y_grid * GRID_SIZE;

                int horizontal = 0;  // -1 or 1
                int vertical = 0;  // -1 or 1

                if (remaining_x >= GRID_HALF_SIZE) horizontal++;
                else if (remaining_x < GRID_HALF_SIZE) horizontal--;
                if (remaining_y >= GRID_HALF_SIZE) vertical++;
                else if (remaining_y < GRID_HALF_SIZE) vertical--;

                int count = 0;
                int separation_count = 0;
                Vector2 average_position = Vector2Zero();
                Vector2 average_direction = Vector2Zero();
                Vector2 average_separation = Vector2Zero();

                // Check 2x2 around boid for accurate movement
                int cell_to_check = i;
                for (int y = 0; y != vertical * 2; y += vertical) {
                    cell_to_check = i + GRID_WIDTH * y;
                    for (int x = 0; x != horizontal * 2; x += horizontal) {
                        cell_to_check += x;

                        if (cell_to_check < 0 || cell_to_check >= GRID_CELLS) continue;

                        int inside = link_heads[cell_to_check];
                        int depth = 0;
                        while ((inside != -1)) {
                            depth++;
                            if (depth > MAX_DEPTH) break;
                            if (inside == current) {
                                inside = boids[inside].next;
                                continue;
                            }

                            float distance_sqr = Vector2DistanceSqr(boids[current].position, boids[inside].position);
                            if (distance_sqr > VIEW_DISTANCE_SQR) {
                                inside = boids[inside].next;
                                continue;
                            }
                            if (Vector2DotProduct(boids[current].position, boids[inside].position) < VIEW_DOT_PRODUCT) {
                                inside = boids[inside].next;
                                continue;
                            }
                            average_position = Vector2Add(average_position, boids[inside].position);
                            average_direction = Vector2Add(average_direction, boids[inside].direction);
                            count++;

                            if (distance_sqr > AVOID_DISTANCE_SQR) {
                                inside = boids[inside].next;
                                continue;
                            }
                            average_separation = Vector2Subtract(average_separation, Vector2Scale(Vector2Subtract(boids[current].position, boids[inside].position), 1.0f / distance_sqr));
                            separation_count++;

                            inside = boids[inside].next;
                        }
                    }
                }

                average_positions[current] = Vector2Scale(average_position, 1.0f / count);
                average_directions[current] = Vector2Normalize(Vector2Scale(average_direction, 1.0f / count));
                average_separations[current] = Vector2Normalize(Vector2Scale(average_separation, 1.0f / separation_count));

                current = boids[current].next;
            }
        }


        int escape_distance_sqr = area_radius * area_radius + AVOID_DISTANCE_SQR;

        // Update boids linked list
        for (int cell = 0; cell < GRID_CELLS; cell++) {
            if (link_heads[cell] == -1) continue;

            int last = -1;
            int i = link_heads[cell];
            while (i != -1) {
                // Move boids
                // Separation
                Vector2 separation = Vector2Scale(Vector2Normalize(Vector2Subtract(average_separations[i], boids[i].direction)), -SEPARATION_CONSTANT);

                // Alignment
                Vector2 alignment = Vector2Scale(Vector2Normalize(Vector2Subtract(average_directions[i], boids[i].direction)), ALIGNMENT_CONSTANT);

                // Cohension
                Vector2 cohesion = Vector2Scale(Vector2Normalize(Vector2Subtract(average_positions[i], boids[i].position)), COHESION_CONSTANT);

                // Avoidance
                Vector2 avoidance = Vector2Zero();
                float distance_sqr = Vector2DistanceSqr(boids[i].position, area_position);
                if (distance_sqr < escape_distance_sqr) {
                    avoidance = Vector2Scale(Vector2Subtract(boids[i].position, area_position), 1.0f / distance_sqr * ESCAPE_FACTOR);
                    if (is_area_attract) avoidance = Vector2Negate(avoidance);
                }

                // Update position & direction
                boids[i].direction = Vector2Normalize(Vector2Add(boids[i].direction, separation));
                boids[i].direction = Vector2Normalize(Vector2Add(boids[i].direction, alignment));
                boids[i].direction = Vector2Normalize(Vector2Add(boids[i].direction, cohesion));
                boids[i].direction = Vector2Normalize(Vector2Add(boids[i].direction, avoidance));
                boids[i].position = Vector2Add(boids[i].position, Vector2Scale(boids[i].direction, MOVE_SPEED * FIXED_DT));

                boids[i].position.x = Wrap(boids[i].position.x, 0, WINDOW_WIDTH);
                boids[i].position.y = Wrap(boids[i].position.y, 0, WINDOW_HEIGHT);

                // Does this boid need to move cells?
                int x_grid = (int) boids[i].position.x / GRID_SIZE;
                x_grid = Wrap(x_grid, 0, GRID_WIDTH);
                int y_grid = (int) boids[i].position.y / GRID_SIZE;
                y_grid = Wrap(y_grid, 0, GRID_HEIGHT);
                int i_grid = y_grid * GRID_WIDTH + x_grid;

                // Boid still in valid grid cell
                if (cell == i_grid) {
                    last = i;
                    i = boids[i].next;
                    continue;
                }

                // We gotta move the boid from this linked list to another
                // Move to head of other linked list for simplicity

                int old_next = boids[i].next;

                // Move head
                if (i == link_heads[cell]) {
                    // Head now points to next element
                    link_heads[cell] = boids[i].next;
                } else {
                    // Last node point to current's next node
                    boids[last].next = boids[i].next;
                }

                // Assign removed node to be new head of correct cell
                boids[i].next = link_heads[i_grid];
                link_heads[i_grid] = i;

                i = old_next;
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
                DrawRectangleLinesEx(rect, 1, DARKGRAY);
            }
        }

        // Draw boids
        for (int i = 0; i < NUM_BOIDS; i++) {
            /*DrawCircleLinesV(boids[i].position, VIEW_DISTANCE, GREEN);*/
            /*DrawCircleLinesV(boids[i].position, AVOID_DISTANCE, BLUE);*/
            /*DrawLineV(boids[i].position, Vector2Add(boids[i].position, Vector2Scale(boids[i].direction, AVOID_DISTANCE)), BLUE);*/
            /*DrawCircleV(boids[i].position, BOID_SIZE, MAGENTA);*/
            /*DrawPixelV(boids[i].position, MAGENTA);*/
            DrawTriangleLines((Vector2) {boids[i].position.x - BOID_SIZE, boids[i].position.y + BOID_SIZE}, (Vector2) {boids[i].position.x + BOID_SIZE, boids[i].position.y + BOID_SIZE}, Vector2Add(boids[i].position, Vector2Scale(boids[i].direction, AVOID_DISTANCE)), MAGENTA);
        }
        DrawCircleLinesV(area_position, area_radius, is_area_attract ? BLUE : RED);
        EndDrawing();

        DrawFPS(4, 4);
    }

    free(boids);
    free(link_heads);
    free(average_positions);
    free(average_directions);
    free(average_separations);

    CloseWindow();

    return 0;
}
