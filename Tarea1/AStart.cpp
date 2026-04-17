#include <iostream>
#include <vector>
#include <queue>
#include <cmath>
#include <algorithm>
#include <fstream>
#include <chrono>
using namespace std;

struct Nodo {
    int x, y;
};  

struct NodoAStar
{
    int x, y;
    double f; 

    bool operator>(const NodoAStar& other) const{
        return f > other.f;
    }  
};

   int movX[8] = {0, 0, -1, 1, -1, 1, -1, 1};
   int movY[8] = {-1, 1, 0, 0, -1, -1, 1, 1};

    double calculateHeuristica(int x1,int y1,int x2, int y2)
    {   
        int dx = abs(x1-x2);
        int dy = abs(y1-y2);
        return sqrt(dx * dx + dy * dy);
    }

    void runAStart(vector<vector<int>>& mapa, Nodo start, Nodo end, double w = 1.0)
    {
        int height = mapa.size();
        int width = mapa[0].size();
        int expandedNodes = 0;

        priority_queue<NodoAStar, vector<NodoAStar>, greater<NodoAStar>> pq;

        // A* necesita recordar cual fue el costo G mas barato para llegar a cada nodo
        // Lo inicializamos con un numero muy grande (infinito)
        vector<vector<double>> cost_G(height, vector<double>(width, 1e9));
        vector<vector<Nodo>> parent(height, vector<Nodo>(width, {-1, -1}));

        cost_G[start.y][start.x] = 0.0;
        double h_start = calculateHeuristica(start.x, start.y, end.x, end.y);

        pq.push({start.x, start.y, 0.0 + (w * h_start)}); // Meter el nodo del inicio a la colo para que arranque

        bool crossTheFinish = false;

        while(!pq.empty())
        {
            NodoAStar actual = pq.top();
            pq.pop();

            if(actual.f > cost_G[actual.y][actual.x] + w * calculateHeuristica(actual.x, actual.y, end.x, end.y))
            {
                continue;
            }

            expandedNodes++;

            if(actual.x == end.x && actual.y == end.y) 
            {
                crossTheFinish = true;
                break;
            }

            for(int i = 0; i < 8; i++)
            {
                int newX = actual.x + movX[i];
                int newY = actual.y + movY[i];

                if(newX >= 0 && newX < width && newY >= 0 && newY < height)
                {
                    // Si es mapa binario
                    if(mapa[newY][newX] == 0)
                    {
                        double cost_per_step = (abs(movX[i]) == 1 && abs(movY[i]) == 1) ? 1.41 : 1.0;

                        double new_G = cost_G[actual.y][actual.x] + cost_per_step;

                        //Si se encuentra un nodo mas barato hacia el vecino, lo actualizamos
                        if(new_G < cost_G[newY][newX])
                        {
                            cost_G[newX][newY] = new_G;

                            double h_next = calculateHeuristica(newX, newY, end.x, end.y);
                            double new_F = new_G + (w * h_next);
                            
                            pq.push({newX, newY, new_F});
                            parent[newY][newX] = {actual.x, actual.y};

                        }
                    }
                }
            }
        }
        
        if(crossTheFinish)
        {
            cout << "Nodos revisados: " << expandedNodes << endl;
        // El largo del camino en A* es literalmente el costo G acumulado en la meta
        cout << "Longitud del camino (Costo final G): " << cost_G[end.y][end.x] << endl;
        }else{
            cout << "No se encontro un camino." << endl;
        }
    }
int main()
{
    ifstream file("mapa8x8.txt");
    if(!file.is_open()  )
    {
        cerr << "Error al abrir el archivo mapa.txt." << endl;
        return 1;
    }

    int rows, columns;
    file >> rows >> columns;

    vector<vector<int>> mapa(rows, vector<int>(columns));
    string line;

    for(int i = 0; i < rows; i++)
    {
       file >> line;
        for(int j = 0; j < columns; j++) 
        {
            mapa[i][j] = line[j] - '0';
        }
    }
    file.close();

    cout << "Mapa cargado exitosamente (" << columns << "x" << rows << ")" << endl;

    Nodo start = {0, 0}; 
    Nodo end = {9, 9}; 

    int replays = 30;

    // A* Normal
    cout << "\n--- Ejecutando A* Normal (w = 1.0) ---" << endl;

    runAStart(mapa, start, end, 1.0);

    double  total_time_astart = 0.0;
    for(int i = 0; i < replays; i++)
    {
        auto strt_time = std::chrono::high_resolution_clock::now();

        runAStart(mapa, start, end, 1.0);
        auto end_time = std::chrono::high_resolution_clock::now();
        chrono::duration<double, std::milli> time_taken = end_time - strt_time;
        total_time_astart += time_taken.count();
    }
    cout << ">> Tiempo Promedio A*: " << (total_time_astart / replays) << " ms\n" << endl;

    //  Wighted A* Peso w = 2.0
    cout << "--- Ejecutando Weighted A* (w = 2.0) ---" << endl;

    runAStart(mapa, start, end, 2.0);

    double total_time_wa = 0.0;

    for(int i = 0; i < replays; i++)
    {
        auto strt_time = std::chrono::high_resolution_clock::now();

        runAStart(mapa, start, end, 2.0);
         auto end_time = std::chrono::high_resolution_clock::now();
        chrono::duration<double, std::milli> time_taken = end_time - strt_time;
        total_time_wa += time_taken.count();
    }
    cout << ">> Tiempo Promedio WA*: " << (total_time_wa / replays) << " ms\n" << endl;

    return 0;
}
