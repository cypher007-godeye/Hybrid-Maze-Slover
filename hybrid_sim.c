#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "API.h"

#define N 16
#define GOAL_SIZE 2 // The goal is a 2x2 area

// --- Global Declarations ---
// Direction vectors: 0:North, 1:East, 2:South, 3:West
int dx[] = {0, 1, 0, -1};
int dy[] = {1, 0, -1, 0};

// --- Data Structures ---
// Node for A* search
typedef struct Node {
    int x, y, g, h, f;
    struct Node* parent;
} Node;

// Cell coordinates for stacks and queues
typedef struct {
    int x, y;
} Point;

// --- Function Prototypes ---
void logit(char *text);
void storeWalls(int x, int y, int turnR, unsigned char walls[N][N]);
void move(int *x, int *y, int turnR);
int a_star_search(int startX, int startY, int goalX, int goalY, unsigned char walls[N][N], Node path[N*N]);
void speed_run(Node path[], int path_len, int* x, int* y, int* turnR);
void go_to_cell(int targetX, int targetY, int* x, int* y, int* turnR, unsigned char walls[N][N]);


// --- Depth-First Search (DFS) Exploration ---

// The main exploration logic to fully map the maze using DFS
void explore_dfs(int* x, int* y, int* turnR, unsigned char walls[N][N], int visited[N][N]) {
    Point stack[N * N];
    int stack_top = 0;

    // Start at the beginning
    stack[stack_top++] = (Point){*x, *y};
    visited[*x][*y] = 1;
    API_setColor(*x, *y, 'B');

    while (stack_top > 0) {
        Point current = stack[stack_top - 1]; // Peek at the top of the stack

        // Ensure the physical bot is at the 'current' cell location
        if (*x != current.x || *y != current.y) {
            go_to_cell(current.x, current.y, x, y, turnR, walls);
        }

        storeWalls(*x, *y, *turnR, walls);

        // --- Find an unvisited, accessible neighbor ---
        int found_neighbor = 0;
        for (int i = 0; i < 4; i++) {
            // Check neighbors in a consistent order (e.g., N, E, S, W relative to current orientation)
            int dir = (*turnR + i) % 4; 
            if (!(walls[current.x][current.y] & (1 << dir))) { // If no wall
                int nx = current.x + dx[dir];
                int ny = current.y + dy[dir];

                if (nx >= 0 && nx < N && ny >= 0 && ny < N && !visited[nx][ny]) {
                    // Found a new cell to explore
                    visited[nx][ny] = 1;
                    API_setColor(nx, ny, 'B');
                    stack[stack_top++] = (Point){nx, ny}; // Push new cell to stack
                    found_neighbor = 1;
                    break; // Explore this path first
                }
            }
        }

        if (!found_neighbor) {
            // If no unvisited neighbors, this path is fully explored. Backtrack.
            stack_top--; // Pop from stack
        }
    }
    // After loop, maze is fully explored. Go back to the start cell.
    go_to_cell(0, 0, x, y, turnR, walls);
}


// --- Goal Finding ---

// Analyzes the map to find the single entrance to the goal area
Point find_goal_entrance(unsigned char walls[N][N], int goal_area_X, int goal_area_Y) {
    for (int i = 0; i < GOAL_SIZE; i++) {
        for (int j = 0; j < GOAL_SIZE; j++) {
            int gx = goal_area_X + i;
            int gy = goal_area_Y + j;
            for (int k = 0; k < 4; k++) {
                if (!(walls[gx][gy] & (1 << k))) { // If there's an opening
                    int nx = gx + dx[k], ny = gy + dy[k];
                    if (!(nx >= goal_area_X && nx < goal_area_X + GOAL_SIZE &&
                          ny >= goal_area_Y && ny < goal_area_Y + GOAL_SIZE)) {
                        char buffer[32];
                        sprintf(buffer, "Entrance found at (%d, %d)", nx, ny);
                        logit(buffer);
                        return (Point){nx, ny};
                    }
                }
            }
        }
    }
    return (Point){-1, -1};
}


// --- Main Program ---

int main() {
    // --- PHASE 0: SETUP ---
    int startX = 0, startY = 0;
    // Define the top-left corner of the 2x2 goal area.
    int goal_area_X = 7, goal_area_Y = 7;
    int x = startX, y = startY, turnR = 0;
    unsigned char walls[N][N] = {0};
    int visited[N][N] = {0};
    Node path[N * N];
    int path_length;

    // --- PHASE 1: FULL EXPLORATION ---
    logit("Phase 1: Starting full DFS exploration...");
    API_setColor(startX, startY, 'G');
    explore_dfs(&x, &y, &turnR, walls, visited);
    logit("Full exploration complete. Bot returned to start.");

    // --- PHASE 2: CALCULATE OPTIMAL PATH ---
    logit("Phase 2: Finding goal entrance...");
    Point entrance = find_goal_entrance(walls, goal_area_X, goal_area_Y);
    if (entrance.x == -1) {
        logit("Error: Could not find goal entrance!");
        return 1;
    }
    
    // --- PHASE 3: SPEED RUN ---
    logit("Phase 3: Executing speed run to goal entrance...");
    API_clearAllColor();
    API_setColor(startX, startY, 'G');
    API_setColor(entrance.x, entrance.y, 'Y');

    path_length = a_star_search(startX, startY, entrance.x, entrance.y, walls, path);
    if (path_length > 0) {
        speed_run(path, path_length, &x, &y, &turnR);
        
        // --- FINAL MOVE INTO GOAL ---
        // After reaching the entrance, perform one last move to enter the goal area.
        logit("At entrance. Making final move into goal.");
        int final_dir = -1;
        // Find which direction leads from the entrance into the goal area
        for (int i = 0; i < 4; i++) {
            if (!(walls[x][y] & (1 << i))) { // If no wall
                int nx = x + dx[i];
                int ny = y + dy[i];
                if (nx >= goal_area_X && nx < goal_area_X + GOAL_SIZE &&
                    ny >= goal_area_Y && ny < goal_area_Y + GOAL_SIZE) {
                    final_dir = i;
                    break;
                }
            }
        }

        if (final_dir != -1) {
            while (turnR != final_dir) {
                int diff = final_dir - turnR;
                if (diff == 1 || diff == -3) { API_turnRight(); turnR = (turnR + 1) % 4; }
                else { API_turnLeft(); turnR = (turnR + 3) % 4; }
            }
            move(&x, &y, turnR);
            API_setColor(x, y, 'R'); // Color the final cell
        }

    } else {
        logit("Could not find path for speed run!");
    }

    logit("Simulation finished.");
    return 0;
}


// --- Utility and Movement Functions ---

void logit(char *text) {
    fprintf(stderr, "%s\n", text);
    fflush(stderr);
}

void storeWalls(int x, int y, int turnR, unsigned char walls[N][N]) {
    if (API_wallFront()) walls[x][y] |= (1 << turnR);
    if (API_wallRight()) walls[x][y] |= (1 << ((turnR + 1) % 4));
    if (API_wallLeft())  walls[x][y] |= (1 << ((turnR + 3) % 4));
}

void move(int *x, int *y, int turnR) {
    if (API_moveForward()) {
        *x += dx[turnR];
        *y += dy[turnR];
    }
}

// Helper function to move bot to a specific adjacent or distant cell
void go_to_cell(int targetX, int targetY, int* x, int* y, int* turnR, unsigned char walls[N][N]) {
    Node path[N*N];
    int path_len = a_star_search(*x, *y, targetX, targetY, walls, path);
    if (path_len > 0) {
        speed_run(path, path_len, x, y, turnR);
    }
}

void speed_run(Node path[], int path_len, int* x, int* y, int* turnR) {
    for (int i = 1; i < path_len; i++) {
        int target_x = path[i].x, target_y = path[i].y;
        int required_dir = -1;
        if (target_x == *x && target_y == *y + 1) required_dir = 0;
        else if (target_x == *x + 1 && target_y == *y) required_dir = 1;
        else if (target_x == *x && target_y == *y - 1) required_dir = 2;
        else if (target_x == *x - 1 && target_y == *y) required_dir = 3;

        while (*turnR != required_dir) {
            int diff = required_dir - *turnR;
            if (diff == 1 || diff == -3) { API_turnRight(); *turnR = (*turnR + 1) % 4; }
            else { API_turnLeft(); *turnR = (*turnR + 3) % 4; }
        }
        move(x, y, *turnR);
        API_setColor(*x, *y, 'R');
    }
}

// A* implementation (unchanged)
int a_star_search(int startX, int startY, int goalX, int goalY, unsigned char walls[N][N], Node path[N*N]) {
    Node* openList[N*N]; int openCount = 0;
    Node* closedList[N*N]; int closedCount = 0;
    Node all_nodes[N][N];
    Node* startNode = &all_nodes[startX][startY];
    startNode->x = startX; startNode->y = startY; startNode->g = 0;
    startNode->h = abs(startX - goalX) + abs(startY - goalY);
    startNode->f = startNode->g + startNode->h; startNode->parent = NULL;
    openList[openCount++] = startNode;
    while (openCount > 0) {
        int best_idx = 0;
        for (int i = 1; i < openCount; i++) if (openList[i]->f < openList[best_idx]->f) best_idx = i;
        Node* current = openList[best_idx];
        for (int i = best_idx; i < openCount - 1; i++) openList[i] = openList[i+1];
        openCount--;
        closedList[closedCount++] = current;
        if (current->x == goalX && current->y == goalY) {
            Node* path_node = current; int len = 0;
            while(path_node != NULL) { path[len++] = *path_node; path_node = path_node->parent; }
            for(int i = 0; i < len/2; i++){ Node temp = path[i]; path[i] = path[len-1-i]; path[len-1-i] = temp; }
            return len;
        }
        for (int i = 0; i < 4; i++) {
            if (walls[current->x][current->y] & (1 << i)) continue;
            int nx = current->x + dx[i], ny = current->y + dy[i];
            if (nx < 0 || nx >= N || ny < 0 || ny >= N) continue;
            int in_closed = 0;
            for(int j=0; j<closedCount; j++) if(closedList[j]->x == nx && closedList[j]->y == ny) { in_closed = 1; break; }
            if (in_closed) continue;
            int g_cost = current->g + 1;
            int in_open_idx = -1;
            for(int j=0; j<openCount; j++) if(openList[j]->x == nx && openList[j]->y == ny) { in_open_idx = j; break; }
            if (in_open_idx != -1) {
                if (g_cost < openList[in_open_idx]->g) {
                    openList[in_open_idx]->g = g_cost;
                    openList[in_open_idx]->f = g_cost + openList[in_open_idx]->h;
                    openList[in_open_idx]->parent = current;
                }
            } else {
                Node* neighbor = &all_nodes[nx][ny];
                neighbor->x = nx; neighbor->y = ny; neighbor->g = g_cost;
                neighbor->h = abs(nx - goalX) + abs(ny - goalY);
                neighbor->f = neighbor->g + neighbor->h; neighbor->parent = current;
                openList[openCount++] = neighbor;
            }
        }
    }
    return 0;
}
