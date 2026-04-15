#include <iostream>
#include <vector>
#include <queue>  
#include <map>
#include <cmath>
#include <algorithm>
#include <fstream>
#include <string>

using namespace std;

struct Nodo {
    int x, y;
};

int movX[8] = {0, 0, -1, 1, -1, 1, -1, 1};
int movY[8] = {-1, 1, 0, 0, -1, -1, 1, 1};

// Cambiamos "map" a "mapa" para evitar colisiones con std::map
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
        // Uso std::cout por si tu editor sigue con el bug visual de ambigüedad
        std::cout << "BFS encontro la salida!" << std::endl;
        std::cout << "Nodos revisados: " << expandedNodes << std::endl;

        vector<Nodo> finalPath;
        Nodo current = end;
        while(!(current.x == start.x && current.y == start.y))
        {
            finalPath.push_back(current);
            // Recuperamos usando [y][x]
            current = parent[current.y][current.x];
        }
        finalPath.push_back(start);
        
        reverse(finalPath.begin(), finalPath.end());

        double pathLength = 0.0;
        // El for debe empezar en 0 para contar el primer paso
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
    BFS(mapa, start, end);

    return 0;
}