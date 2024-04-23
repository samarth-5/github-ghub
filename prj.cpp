#include <iostream>
#include <vector>
#include <queue>
#include <climits>
#include <cmath>
#include <iomanip>

using namespace std;

struct Edge 
{
    int destination;
    int weight;
};

struct Vertex 
{
    vector<Edge> edges;
};

void inputData(int& num_vertices, int& num_edges, vector<vector<int>>& edge_data) 
{    
    cout << "--------------------------------------------------------------\n"; 
    cout << "REAL TIME DISASTER RESPONSE PLANNING USING GRAPH-BASED ALGOS!!\n";
    cout << "--------------------------------------------------------------\n\n"; 
    cout << "Enter the number of vertices: ";
    cin >> num_vertices;
    cout << "Enter the number of edges: ";
    cin >> num_edges;
    
    edge_data.resize(num_edges, vector<int>(3));
    cout << "--------------------------------------------------------------\n"; 

    cout << "Enter edge data in the format (source destination weight):" << endl;
    for (int i = 0; i < num_edges; ++i) 
    {
        cout << "Edge " << i + 1 << ": ";
        cin >> edge_data[i][0] >> edge_data[i][1] >> edge_data[i][2];
    }
    cout << "--------------------------------------------------------------\n"; 
}


// Function to construct the graph from the input data
void constructGraph(vector<Vertex>& graph, const vector<vector<int>>& edge_data) 
{
    for (const auto& edge : edge_data) 
    {
        int source = edge[0];
        int destination = edge[1];
        int weight = edge[2];

        graph[source].edges.push_back({destination, weight});
    }
}


// Function to calculate the shortest path using Dijkstra's algorithm
void dijkstra(vector<Vertex>& graph, int source, vector<int>& distance, vector<int>& parent) 
{
    priority_queue<pair<int, int>, vector<pair<int, int>>, greater<pair<int, int>>> pq;
    pq.push({0, source});
    distance[source] = 0;

    while (!pq.empty()) 
    {
        int u = pq.top().second;
        pq.pop();

        for (const Edge& e : graph[u].edges) 
        {
            int v = e.destination;
            int weight = e.weight;

            if (distance[v] > distance[u] + weight) {
                distance[v] = distance[u] + weight;
                parent[v] = u;
                pq.push({distance[v], v});
            }
        }
    }
}


// Function to find and display the shortest path from source to destination
void printShortestPath(int source, int destination, const vector<int>& parent) 
{
    vector<int> path;
    for (int v = destination; v != source; v = parent[v]) 
    {
        path.push_back(v);
    }
    path.push_back(source);

    cout << "Shortest path from vertex " << source << " to vertex " << destination << ": ";
    for (int i = path.size() - 1; i >= 0; --i) 
    {
        cout << path[i];
        if (i != 0) cout << " -> ";
    }
    cout << endl;
}


int dijkstraShortestDistance(vector<Vertex>& graph, int source, int destination) 
{
    vector<int> distance(graph.size(), INT_MAX);
    priority_queue<pair<int, int>, vector<pair<int, int>>, greater<pair<int, int>>> pq;
    pq.push({0, source});
    distance[source] = 0;

    while (!pq.empty()) 
    {
        int u = pq.top().second;
        pq.pop();

        if (u == destination) // If we reach the destination, return its distance
            return distance[u];

        for (const Edge& e : graph[u].edges) 
        {
            int v = e.destination;
            int weight = e.weight;

            if (distance[v] > distance[u] + weight) 
            {
                distance[v] = distance[u] + weight;
                pq.push({distance[v], v});
            }
        }
    }

    return INT_MAX; // Destination not reachable
}


// Updated minimum time function
double minimumTime(vector<Vertex>& graph, int source, int destination, double speed) 
{
    // Calculate shortest distance from source to destination
    int shortest_distance = dijkstraShortestDistance(graph, source, destination);

    // Calculate shortest time using distance formula (time = distance / speed)
    double shortest_time = shortest_distance / speed;

    return shortest_time;
}


// Function to find the minimum distance to reach affected areas
double minimumDistance(vector<Vertex>& graph, int source) 
{
    double min_distance = 0.0;
    // Additional logic to calculate minimum distance can be added here based on your requirements
    return min_distance;
}


// Function to display menu options
void displayMenu() 
{
    cout << "--------------------------------------------------------------\n"; 
    cout << "-----------------------------MENU-----------------------------\n"; 
    cout << "1. Calculate Shortest Path" << endl;
    cout << "2. Minimum Time to Reach Affected Areas" << endl;
    cout << "3. Minimum Distance to Reach Affected Areas" << endl;
    cout << "4. Calculate Distance from Source to All Destinations" << endl;
    cout << "5. Modify Graph Data" << endl;
    cout << "6. Display Graph" << endl;
    cout << "7. Exit" << endl;
    cout << "Enter your choice: ";
    cout << "\n--------------------------------------------------------------\n"; 

}


// Function to display the graph
void displayGraph(const vector<Vertex>& graph) 
{
    cout << "\n------ GRAPH ------" << endl;
    for (size_t i = 0; i < graph.size(); ++i) 
    {
        cout << "Vertex " << i << ": ";
        for (const auto& edge : graph[i].edges) 
        {
            cout << "(" << edge.destination << ", " << edge.weight << ") ";
        }
        cout << endl;
    }
}


// Function to calculate distance from source to all destinations
void calculateAllDistances(vector<Vertex>& graph, int source, vector<int>& distance) 
{
    priority_queue<pair<int, int>, vector<pair<int, int>>, greater<pair<int, int>>> pq;
    pq.push({0, source});
    distance[source] = 0;

    while (!pq.empty()) 
    {
        int u = pq.top().second;
        pq.pop();

        for (const Edge& e : graph[u].edges) 
        {
            int v = e.destination;
            int weight = e.weight;

            if (distance[v] > distance[u] + weight) 
            {
                distance[v] = distance[u] + weight;
                pq.push({distance[v], v});
            }
        }
    }
}


//function to modify graph data
void modifyGraphData(vector<Vertex>& graph, vector<vector<int>>& edge_data) 
{
    cout << "--------------------------------------------------------------\n"; 
    cout << "                     MODIFY GRAPH DATA" << endl;
    cout << "--------------------------------------------------------------\n"; 
    cout << "Enter the number of edges to modify: ";
    int num_modified_edges;
    cin >> num_modified_edges;

    cout << "Enter the new edge data in the format (source destination weight):" << endl;
    for (int i = 0; i < num_modified_edges; ++i) 
    {
        int source, destination, weight;
        cout << "Modified Edge " << i + 1 << ": ";
        cin >> source >> destination >> weight;

        // Update graph data
        edge_data.push_back({source, destination, weight});
        constructGraph(graph, edge_data);
    }

    cout << "Graph data modified successfully!" << endl;
    cout << "--------------------------------------------------------------\n"; 
}


int main() 
{

    int num_vertices, num_edges;
    vector<vector<int>> edge_data;
    
    // Input data from the user
    inputData(num_vertices, num_edges, edge_data);

    // Construct the graph
    vector<Vertex> graph(num_vertices);
    constructGraph(graph, edge_data);

    

    // Initialize distance array to store shortest distances from source to each vertex
    vector<int> distance(num_vertices, INT_MAX);
    vector<int> parent(num_vertices, -1);

    

    // Output shortest distances from source to each vertex
    // cout << "\nShortest distances from source vertex " << source << ":" << endl;
    // for (int i = 0; i < num_vertices; ++i) 
    // {
    //     cout << "Vertex " << i << ": " << setw(3) << distance[i] << endl;
    // }

    // Variables for minimum time calculation
    char choice;

    // Display menu options
    while (true) 
    {
        displayMenu();
        cin >> choice;

        switch (choice) 
        {
            case '1':
                // Source vertex for the shortest path calculation
                int source;
                cout << "Enter the source vertex: ";
                cin >> source;
                int destination;
                cout << "Enter the destination vertex: ";
                cin >> destination;
                // Calculate shortest paths using Dijkstra's algorithm
                dijkstra(graph, source, distance, parent);
                printShortestPath(source, destination, parent);
                break;

            case '2':
                int src;
                cout << "Enter the source vertex: ";
                cin >> src;
                int dest;
                cout << "Enter the destination vertex: ";
                cin >> dest;
                double speed;
                cout << "Enter speed (in km/hr): ";
                cin >> speed;
                cout << "Minimum time to reach affected area: " << fixed << setprecision(2)
                     << minimumTime(graph, src, dest, speed) << " hours" << endl;
                break;

            case '3':
                int srcc;
                cout << "Enter the source vertex: ";
                cin >> srcc;
                int destt;
                cout << "Enter the destination vertex: ";
                cin >> destt;
                cout << "Minimum distance to reach affected areas: " << dijkstraShortestDistance(graph, srcc, destt) << endl;
                break;

            case '4':
                // Calculate distances from source to all destinations
                int sr;
                cout << "Enter the source vertex: ";
                cin >> sr;
                calculateAllDistances(graph, sr, distance);

                cout << "Distances from source vertex " << source << " to all destinations:" << endl;
                for (int i = 0; i < num_vertices; ++i) 
                {
                    cout << "Vertex " << i << ": " << setw(3) << distance[i] << endl;
                }
                break;

            case '5':
                modifyGraphData(graph, edge_data);
                break;

            case '6':
                displayGraph(graph);
                break;

            case '7':
                cout << "Exiting program...\n";
                return 0;

            default:
                cout << "Invalid choice! Please enter a valid option.\n";
        }
    }

    return 0;
}
