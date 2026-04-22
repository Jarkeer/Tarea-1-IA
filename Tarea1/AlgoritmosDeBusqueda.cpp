#include <iostream>
#include <vector>
#include <queue>
#include <cmath>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <string>
#include <limits>
#include <iomanip>
#include <functional>
 
using namespace std;
 
 
struct Node {
    int x, y;
};
 
struct NodeAStar {
    int x, y;
    double f;

    // Sobrecarga para que la cola de prioridad ordene de menor a mayor F
    bool operator>(const NodeAStar& other) const {
         return f > other.f;
         }
};
 
struct NodeGreedy {
    int x, y;
    double h;

    // Sobrecarga para que la cola de prioridad ordene de menor a mayor H
    bool operator>(const NodeGreedy& other) const {
         return h > other.h;
         }
};
 
// Guarda los 3 resultados que queremos comparar
struct Result {
    bool   found;
    int    nodes;       // nodos revisados
    double pathLength;  // largo del camino
};
 

 // Direcciones para movimiento 8-conectado
int dx_dir[8] = {0, 0, -1, 1, -1, 1, -1, 1};
int dy_dir[8] = {-1, 1, 0, 0, -1, -1, 1, 1};
 
// Cálculo de la distancia Euclidiana
double calculateHeuristic(int x1, int y1, int x2, int y2) 
{
    int dx = abs(x1 - x2), dy = abs(y1 - y2);
    return sqrt(dx * dx + dy * dy);
}
 
// Reconstruye el camino desde parent[][] y calcula su largo
double calcPathLength(vector<vector<Node>>& parent, vector<vector<int>>&  mapGrid, Node start, Node end, bool isHeightMap) 
{
    vector<Node> path;
    Node step = end;

    while (!(step.x == start.x && step.y == start.y))
     {
        path.push_back(step);
        step = parent[step.y][step.x];
    }
    path.push_back(start);
    reverse(path.begin(), path.end());
 
    double length = 0.0;
    for (size_t i = 0; i < path.size() - 1; i++) {
        Node a = path[i], b = path[i+1];
        if (isHeightMap) {
            length += abs(mapGrid[a.y][a.x] - mapGrid[b.y][b.x]);
        } else {
            int dx = abs(b.x - a.x), dy = abs(b.y - a.y);
            length += (dx == 1 && dy == 1) ? 1.41 : 1.0;
        }
    }
    return length;
}
 
// Imprime los 3 resultados mas el tiempo promedio
void printResults(const Result& r, double timeMs) {
    cout << fixed << setprecision(4);
    cout << "   -> Nodos revisados  : " << r.nodes      << endl;
    cout << "   -> Largo del camino : " << r.pathLength  << endl;
    cout << "   -> Tiempo promedio  : " << timeMs        << " ms" << endl;
} 

// 1. BFS
Result runBFS(vector<vector<int>>& mapGrid, Node start, Node end, bool isHeightMap)
 {
    int height = mapGrid.size();
    int width = mapGrid[0].size();
    int expandedNodes = 0;
 
    queue<Node> q;
    vector<vector<bool>> visited(height, vector<bool>(width, false));
    vector<vector<Node>> parent(height, vector<Node>(width, {-1, -1}));
 
    q.push(start);
    visited[start.y][start.x] = true;
 
    while (!q.empty()) {
        Node current = q.front(); q.pop();
        expandedNodes++;
 
        if (current.x == end.x && current.y == end.y)
            return {true, expandedNodes,
                    calcPathLength(parent, mapGrid, start, end, isHeightMap)};
 
        for (int i = 0; i < 8; i++) {
            int newX = current.x + dx_dir[i], newY = current.y + dy_dir[i];
            if (newX >= 0 && newX < width && newY >= 0 && newY < height)
             {
                bool walkable = isHeightMap ? true : (mapGrid[newY][newX] == 0);
                if (walkable && !visited[newY][newX]) {
                    visited[newY][newX] = true;
                    q.push({newX, newY});
                    parent[newY][newX] = current;
                }
            }
        }
    }
    return {false, expandedNodes, 0.0};
}
 
// 2. GREEDY BFS
Result runGreedyBFS(vector<vector<int>>& mapGrid, Node start, Node end, bool isHeightMap)
 {
    int height = mapGrid.size();
    int width = mapGrid[0].size();
    int expandedNodes = 0;
 
    priority_queue<NodeGreedy, vector<NodeGreedy>, greater<NodeGreedy>> pq;
    vector<vector<bool>> visited(height, vector<bool>(width, false));
    vector<vector<Node>> parent(height, vector<Node>(width, {-1, -1}));
 
    pq.push({start.x, start.y, calculateHeuristic(start.x, start.y, end.x, end.y)});
    visited[start.y][start.x] = true;
 
    while (!pq.empty()) {
        NodeGreedy current = pq.top(); pq.pop();
        expandedNodes++;
 
        if (current.x == end.x && current.y == end.y)
            return {
                true, expandedNodes,
                    calcPathLength(parent, mapGrid, start, end, isHeightMap)};
 
        for (int i = 0; i < 8; i++) 
        {
            int newX = current.x + dx_dir[i], newY = current.y + dy_dir[i];
            if (newX >= 0 && newX < width && newY >= 0 && newY < height)
             {
                bool walkable = isHeightMap ? true : (mapGrid[newY][newX] == 0);
                if (walkable && !visited[newY][newX])
                 {
                    visited[newY][newX] = true;
                    pq.push({newX, newY, calculateHeuristic(newX, newY, end.x, end.y)});
                    parent[newY][newX] = {current.x, current.y};
                }
            }
        }
    }
    return {false, expandedNodes, 0.0};
}
 
// 3. A* Y WEIGHTED A* 
Result runAStar(vector<vector<int>>& mapGrid, Node start, Node end, double weight, bool isHeightMap)
 {
    int height = mapGrid.size();
    int width = mapGrid[0].size();
    int expandedNodes = 0;
 
    priority_queue<NodeAStar, vector<NodeAStar>, greater<NodeAStar>> pq;
    vector<vector<double>> g_cost(height, vector<double>(width, 1e9));
    vector<vector<bool>>   expanded(height, vector<bool>(width, false));
    vector<vector<Node>>   parent(height, vector<Node>(width, {-1, -1}));
 
    g_cost[start.y][start.x] = 0.0;
    pq.push({start.x, start.y,
             weight * calculateHeuristic(start.x, start.y, end.x, end.y)});
 
    while (!pq.empty()) 
    {
        NodeAStar current = pq.top(); pq.pop();
 
        if (expanded[current.y][current.x]) continue;
        expanded[current.y][current.x] = true;
        expandedNodes++;
 
        if (current.x == end.x && current.y == end.y)
            return {true, expandedNodes,
                    calcPathLength(parent, mapGrid, start, end, isHeightMap)};
 
        for (int i = 0; i < 8; i++) 
        {
            int newX = current.x + dx_dir[i], newY = current.y + dy_dir[i];
            if (newX >= 0 && newX < width && newY >= 0 && newY < height) 
            {
                bool walkable = isHeightMap ? true : (mapGrid[newY][newX] == 0);
                if (walkable && !expanded[newY][newX]) {
                    double base = (abs(dx_dir[i]) == 1 && abs(dy_dir[i]) == 1) ? 1.41 : 1.0;
                    int    hdiff = abs(mapGrid[current.y][current.x] - mapGrid[newY][newX]);
                    double step  = isHeightMap ? (hdiff == 0 ? base : hdiff) : base;
 
                    double newG = g_cost[current.y][current.x] + step;
                    if (newG < g_cost[newY][newX]) {
                        g_cost[newY][newX] = newG;
                        double newF = newG + weight * calculateHeuristic(newX, newY, end.x, end.y);
                        pq.push({newX, newY, newF});
                        parent[newY][newX] = {current.x, current.y};
                    }
                }
            }
        }
    }
    return {false, expandedNodes, 0.0};
}
 
// Funcion central: corre N repeticiones,
// mide tiempo y muestra las 3 metricas juntas
 
void runExperiment(const string& name,function<Result()> algo, int replays) 
{
    cout << "\n[*] " << name << ":" << endl;
 
    Result last  = {false, 0, 0.0};
    double total = 0.0;
 
    for (int i = 0; i < replays; i++) {
        auto t1 = chrono::high_resolution_clock::now();
        last    = algo();
        auto t2 = chrono::high_resolution_clock::now();
        total  += chrono::duration<double, milli>(t2 - t1).count();
    }
 
    if (last.found) {
        printResults(last, total / replays);
    } else {
        cout << "-> Camino no encontrado." << endl;
    }
}
 
// MENU PRINCIPAL
 
int main() {
    string fileName;
 
    cout << " TAREA 1 IA - BUSQUEDA DE CAMINOS" << endl;
    cout << "========================================" << endl;
 
    cout << "Ingrese el nombre del archivo del mapa (ej. perling_small_map.txt): ";
    cin >> fileName;
 
    ifstream file(fileName);
    if (!file.is_open()) 
    {
        cerr << "Error: No se pudo abrir el archivo '" << fileName << "'." << endl;
        return 1;
    }
 
    int rows, columns;
    file >> rows >> columns;
    vector<vector<int>> mapGrid(rows, vector<int>(columns));
 
    
    string firstDataLine;
    file.ignore(numeric_limits<streamsize>::max(), '\n');
    getline(file, firstDataLine);
    if (!firstDataLine.empty() && firstDataLine.back() == '\r')
        firstDataLine.pop_back();
 
    bool hasSpaces = (firstDataLine.find(' ') != string::npos);
 
    if (hasSpaces) 
    {
        size_t pos = 0;
        for (int j = 0; j < columns; j++) 
        {
            mapGrid[0][j] = stoi(firstDataLine, &pos);
            pos = firstDataLine.find_first_not_of(' ', firstDataLine.find_first_of(' ', pos));
            if (pos == string::npos) pos = firstDataLine.size();
        }
        for (int i = 1; i < rows; i++)
            for (int j = 0; j < columns; j++)
                file >> mapGrid[i][j];
    } else
     {
        for (int j = 0; j < columns; j++)
            mapGrid[0][j] = firstDataLine[j] - '0';
        for (int i = 1; i < rows; i++)
         {
            string line;
            getline(file, line);
            if (!line.empty() && line.back() == '\r') line.pop_back();
            for (int j = 0; j < columns; j++)
                mapGrid[i][j] = line[j] - '0';
        }
    }
    file.close();
 
    cout << "\nMapa cargado! Dimensiones: " << columns << "x" << rows << endl;
 
    bool isHeightMap = false;
    for (int i = 0; i < rows && !isHeightMap; i++)
        for (int j = 0; j < columns && !isHeightMap; j++)
            if (mapGrid[i][j] > 1) isHeightMap = true;
 
    cout << "Tipo detectado: "
         << (isHeightMap ? "Mapa de Alturas" : "Mapa Binario") << endl;
 
    Node start = {0, 0};
    Node end   = {columns - 1, rows - 1};
    const int replays = 30;
 
    int option = -1;
    while (option != 0) {
        cout << "\n=== MENU ====" << endl;
        cout << "1. BFS             (30 repeticiones)" << endl;
        cout << "2. Greedy BFS      (30 repeticiones)" << endl;
        cout << "3. A*              (30 repeticiones)" << endl;
        cout << "4. Weighted A*     (30 repeticiones, peso personalizado)" << endl;
        cout << "5. EXPERIMENTO COMPLETO (todos, 30 repeticiones)" << endl;
        cout << "0. Salir" << endl;
        cout << "==============================" << endl;
        cout << "Seleccione: ";
        cin >> option;
        cout << "\n";
 
        switch (option) {
            case 1:
                runExperiment("BFS",
                    [&]() { return runBFS(mapGrid, start, end, isHeightMap); },
                    replays);
                break;
 
            case 2:
                runExperiment("Greedy BFS",
                    [&]() { return runGreedyBFS(mapGrid, start, end, isHeightMap); },
                    replays);
                break;
 
            case 3:
                runExperiment("A* (w=1.0)",
                    [&]() { return runAStar(mapGrid, start, end, 1.0, isHeightMap); },
                    replays);
                break;
 
            case 4: {
                double w;
                cout << "Ingrese el peso (ej. 1.5, 2.0, 5.0): ";
                cin >> w;
                runExperiment("Weighted A* (w=" + to_string(w) + ")",
                    [&]() { return runAStar(mapGrid, start, end, w, isHeightMap); },
                    replays);
                break;
            }
 
            case 5:
                cout << "EXPERIMENTO COMPLETO (" << replays << " repeticiones)" << endl;
                runExperiment("BFS",
                    [&]() { return runBFS(mapGrid, start, end, isHeightMap); }, replays);
                runExperiment("Greedy BFS",
                    [&]() { return runGreedyBFS(mapGrid, start, end, isHeightMap); }, replays);
                runExperiment("A* (w=1.0)",
                    [&]() { return runAStar(mapGrid, start, end, 1.0, isHeightMap); }, replays);
                runExperiment("Weighted A* (w=2.0)",
                    [&]() { return runAStar(mapGrid, start, end, 2.0, isHeightMap); }, replays);
                break;
 
            case 0:
                cout << "Saliendo." << endl;
                break;
 
            default:
                cout << "Opcion invalida." << endl;
                break;
        }
    }
    return 0;
}