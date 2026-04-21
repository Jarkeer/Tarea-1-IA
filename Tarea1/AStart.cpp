#include <iostream>
#include <vector>
#include <queue>
#include <cmath>
#include <algorithm>
#include <fstream>
#include <chrono>

using namespace std;

struct Node {
    int x, y;
};  

struct NodeAStar {
    int x, y;
    double f; 
    
    bool operator>(const NodeAStar& other) const {
        return f > other.f;
    }  
};

int dx_dir[8] = {0, 0, -1, 1, -1, 1, -1, 1};
int dy_dir[8] = {-1, 1, 0, 0, -1, -1, 1, 1};

double calculateHeuristic(int x1, int y1, int x2, int y2) {   
    int dx = abs(x1 - x2);
    int dy = abs(y1 - y2);
    return sqrt(dx * dx + dy * dy);
}

void runAStar(vector<vector<int>>& mapGrid, Node start, Node end, double weight, bool isHeightMap, bool silent = false) {
    int height = mapGrid.size();
    int width = mapGrid[0].size();
    int expandedNodes = 0;

    priority_queue<NodeAStar, vector<NodeAStar>, greater<NodeAStar>> pq;
    // A* necesita recordar cual fue el costo G mas barato para llegar a cada nodo
    // Lo inicializamos con un numero muy grande (infinito)
    vector<vector<double>> g_cost(height, vector<double>(width, 1e9));
    vector<vector<Node>> parent(height, vector<Node>(width, {-1, -1}));

    g_cost[start.y][start.x] = 0.0;
    double h_start = calculateHeuristic(start.x, start.y, end.x, end.y);

    pq.push({start.x, start.y, 0.0 + (weight * h_start)}); 

    bool reachedGoal = false;

    while(!pq.empty()) {
        NodeAStar current = pq.top();
        pq.pop();

        if(current.f > g_cost[current.y][current.x] + weight * calculateHeuristic(current.x, current.y, end.x, end.y)) {
            continue;
        }

        expandedNodes++;

        if(current.x == end.x && current.y == end.y) {
            reachedGoal = true;
            break;
        }

        for(int i = 0; i < 8; i++) {
            int newX = current.x + dx_dir[i];
            int newY = current.y + dy_dir[i];

            if(newX >= 0 && newX < width && newY >= 0 && newY < height) {
                
                bool isWalkable = false;
                if (isHeightMap) {
                    isWalkable = true; // Todos los nodos son transitables en los mapas de altura
                } else {
                    if (mapGrid[newY][newX] == 0) isWalkable = true; // 0 es accesible a pie en mapas binarios
                }

                if(isWalkable) {
                    double base_step_cost = (abs(dx_dir[i]) == 1 && abs(dy_dir[i]) == 1) ? 1.41 : 1.0;
                    double actual_step_cost = 0.0;

                    if (isHeightMap) {
                        int height_diff = abs(mapGrid[current.y][current.x] - mapGrid[newY][newX]);
                        actual_step_cost = (height_diff == 0) ? base_step_cost : height_diff;
                    } else {
                        actual_step_cost = base_step_cost;
                    }

                    double new_G = g_cost[current.y][current.x] + actual_step_cost;

                    if(new_G < g_cost[newY][newX]) {
                        g_cost[newY][newX] = new_G;
                        double h_next = calculateHeuristic(newX, newY, end.x, end.y);
                        double new_F = new_G + (weight * h_next);
                        
                        pq.push({newX, newY, new_F});
                        parent[newY][newX] = {current.x, current.y};
                    }
                }
            }
        }
    } 
    
    if(!silent) {
        if(reachedGoal) {
            cout << "Nodos expandidos: " << expandedNodes << endl;
            cout << "Longitud del camino (Costo G Final): " << g_cost[end.y][end.x] << endl;
        } else {
            cout << "Camino no encontrado." << endl;
        }
    }
}

int main() {
    string fileName = "perling_small_map.txt"; // Modificar según el mapa que quieras probar
    ifstream file(fileName);
    if(!file.is_open()) {
        cerr << "Error al abrir el archivo: " << fileName << endl;
        return 1;
    }

    int rows, columns;
    file >> rows >> columns;
    vector<vector<int>> mapGrid(rows, vector<int>(columns));
    string line;

    for(int i = 0; i < rows; i++) {
       file >> line;
       for(int j = 0; j < columns; j++) {
           mapGrid[i][j] = line[j] - '0';
       }
    }
    file.close();

    cout << "Mapa'" << fileName << "' cargado (" << columns << "x" << rows << ")" << endl;

    Node start = {0, 0}; 
    Node end = {columns - 1, rows - 1}; 

    int replays = 30;
    bool isHeightMap = true; // Establezca este valor en false si utiliza el laberinto binario

    // --- A* Normal ---
    cout << "\n--- Corriendo A* Normal (w = 1.0) ---" << endl;
    runAStar(mapGrid, start, end, 1.0, isHeightMap, false); 

    double total_time_astar = 0.0;
    for(int i = 0; i < replays; i++) {
        auto strt_time = std::chrono::high_resolution_clock::now();
        runAStar(mapGrid, start, end, 1.0, isHeightMap, true); 
        auto end_time = std::chrono::high_resolution_clock::now();
        chrono::duration<double, std::milli> time_taken = end_time - strt_time;
        total_time_astar += time_taken.count();
    }
    cout << ">> Tiempo Promedio A*: " << (total_time_astar / replays) << " ms\n" << endl;

    // --- Weighted A* ---
    cout << "--- Corriendo Weighted A* (w = 2.0) ---" << endl;
    runAStar(mapGrid, start, end, 2.0, isHeightMap, false);

    double total_time_wa = 0.0;
    for(int i = 0; i < replays; i++) {
        auto strt_time = std::chrono::high_resolution_clock::now();
        runAStar(mapGrid, start, end, 2.0, isHeightMap, true);
        auto end_time = std::chrono::high_resolution_clock::now();
        chrono::duration<double, std::milli> time_taken = end_time - strt_time;
        total_time_wa += time_taken.count();
    }
    cout << ">> Tiempo Promedio WA*: " << (total_time_wa / replays) << " ms\n" << endl;

    return 0;
}