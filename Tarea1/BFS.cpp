#include <iostream>
#include <vector>
#include <queue>  
#include <map>
#include <cmath>
#include <algorithm>
#include <fstream>
#include <string>
#include <chrono>
using namespace std;

struct Nodo {
    int x, y;
};

int movX[8] = {0, 0, -1, 1, -1, 1, -1, 1};
int movY[8] = {-1, 1, 0, 0, -1, -1, 1, 1};


void BFS(vector<vector<int>>& mapa, Nodo start, Nodo end)
{
    int height = mapa.size();
    int width = mapa[0].size();

    int expandedNodes = 0;
    queue<Nodo> q;

    vector<vector<bool>> visited(height, vector<bool>(width, false));
    vector<vector<Nodo>> parent(height, vector<Nodo>(width, {-1, -1}));
    
    q.push(start);
    
    visited[start.y][start.x] = true;

    bool crossTheFinish = false;

    while(!q.empty())
    {
        Nodo actual = q.front();
        q.pop();

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

            // Revisar límites usando width (ancho = X) y height (alto = Y)
            if(newX >= 0 && newX < width && newY >= 0 && newY < height)
            {
                // Consultamos matriz en orden [newY][newX]
                if(mapa[newY][newX] == 0 && !visited[newY][newX]) // 0 es camino libre
                {
                    visited[newY][newX] = true;
                    
                    q.push({newX, newY});
                    
                    parent[newY][newX] = actual; 
                }
            }
        }
    }

    if(crossTheFinish)
    {
       
        std::cout << "BFS encontro la salida!" << std::endl;
        std::cout << "Nodos revisados: " << expandedNodes << std::endl;

        vector<Nodo> finalPath;
        Nodo current = end;
        while(!(current.x == start.x && current.y == start.y))
        {
            finalPath.push_back(current);
            
            current = parent[current.y][current.x];
        }
        finalPath.push_back(start);
        
        reverse(finalPath.begin(), finalPath.end());

        double pathLength = 0.0;
        
        for(size_t i = 0; i < finalPath.size() - 1; i++) 
        {
            // Usar abs() para evitar que movimientos negativos arruinen el cálculo
            int dx = abs(finalPath[i+1].x - finalPath[i].x);
            int dy = abs(finalPath[i+1].y - finalPath[i].y);
            
            if(dx == 1 && dy == 1)
            {
                pathLength += 1.41; 
            }
            else 
            {
                pathLength += 1.0; 
            }
        }
        std::cout << "Longitud del camino: " << pathLength << std::endl;
    } else
    {
        std::cout << "No se encontro un camino." << std::endl;
    }
}
void BFS_Alturas(vector<vector<int>>& mapa, Nodo start, Nodo end)
{
    int height = mapa.size();
    int width = mapa[0].size();

    int expandedNodes = 0;
    queue<Nodo> q;

    vector<vector<bool>> visited(height, vector<bool>(width, false));
    vector<vector<Nodo>> parent(height, vector<Nodo>(width, {-1, -1}));
    
    q.push(start);
    visited[start.y][start.x] = true;

    bool crossTheFinish = false;

    while(!q.empty())
    {
        Nodo actual = q.front();
        q.pop();

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
             
                if(!visited[newY][newX]) 
                {
                    visited[newY][newX] = true;
                    q.push({newX, newY});
                    parent[newY][newX] = actual; 
                }
            }
        }
    }

    if(crossTheFinish)
    {
        cout << "BFS encontro la salida!" << endl;
        cout << "Nodos revisados: " << expandedNodes << endl;

        vector<Nodo> finalPath;
        Nodo current = end;
        while(!(current.x == start.x && current.y == start.y))
        {
            finalPath.push_back(current);
            current = parent[current.y][current.x];
        }
        finalPath.push_back(start);
        
        reverse(finalPath.begin(), finalPath.end());

        double pathLength = 0.0;
        
        
        for(size_t i = 0; i < finalPath.size() - 1; i++) 
        {
            Nodo actual = finalPath[i];
            Nodo siguiente = finalPath[i+1];
            
            // Calculamos la diferencia de altura en valor absoluto
            int altura_actual = mapa[actual.y][actual.x];
            int altura_siguiente = mapa[siguiente.y][siguiente.x];
            int diferencia_altura = abs(altura_actual - altura_siguiente);

            
            pathLength += diferencia_altura; 
        }
        cout << "Costo total del camino (diferencia de alturas): " << pathLength << endl;
    } else
    {
        cout << "No se encontro un camino." << endl;
    }
}
int main()
{
    ifstream file("mapa.txt");
    if(!file.is_open())
    {
        cerr << "Error al abrir el archivo." << endl;
        return 1;
    }

    int rows, columns;
    file >> rows >> columns;

    // Cambiamos nombre de variable aquí también
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

    std::cout << "Mapa cargado exitosamente (" << columns << "x" << rows << ")" << std::endl;

    Nodo start = {1, 1}; 
    Nodo end = {3,3}; 

    std::cout << "Iniciando BFS...(" << start.x << "," << start.y << ") -> (" << end.x << "," << end.y << ")" << std::endl;
    
    // Y se la pasamos aquí
    //BFS(mapa, start, end);

    int replays = 30;
    double total_time_ms = 0.0;

    
    cout << "INICIANDO EXPERIMENTO: BFS" << endl;
    cout << "Mapa: " << columns << "x" << rows << endl;
    cout << "Repeticiones: " << replays << endl;
    cout << "=========================================" << endl;


    for(int i = 0; i < replays; i++)
    {
        auto strt_time = std::chrono::high_resolution_clock::now();
        BFS(mapa, start, end);

        auto end_time = std::chrono::high_resolution_clock::now();
        chrono::duration<double, std::milli> tiempo_tomado = end_time - strt_time;
        total_time_ms += tiempo_tomado.count();
    }

    double tiempo_promedio = total_time_ms / replays;
    
    cout << "\n--- RESULTADOS FINALES DEL EXPERIMENTO ---" << endl;
    cout << "Tiempo Promedio de Ejecución: " << tiempo_promedio << " ms" << endl;

    return 0;
}