#include <iostream>
#include <vector>
#include <limits>
#include <algorithm>
#include <queue>

using namespace std;

// Define infinity as maximum integer value
const int INF = numeric_limits<int>::max();

// Structure to represent an edge in the graph
struct Edge 
{
    int to;       // Destination vertex
    int weight;   // Weight of the edge
};

// Function to find shortest paths from a source node to multiple destinations in a graph
vector<vector<int>> findShortestPaths(const vector<vector<Edge>>& graph, const vector<vector<int>>& all_destinations) 
{
    int n = graph.size();  // Number of vertices in the graph
    vector<vector<int>> shortestPaths;  // Vector to store shortest paths

    // Iterate over all sets of destinations
    for (const auto& destinations : all_destinations) 
    {
        int destination = destinations[0];  // First destination
        int source = 0;                      // Source node

        // Initialize distance, previous node, and visited arrays
        vector<int> dist(n, INF);
        vector<int> prev(n, -1);
        vector<bool> visited(n, false);

        dist[source] = 0;  // Distance from source to itself is 0

        // Priority queue to store vertices based on their distances
        priority_queue<pair<int, int>, vector<pair<int, int>>, greater<pair<int, int>>> pq;
        pq.push({0, source});  // Push source vertex with distance 0

        // Dijkstra's algorithm to find shortest paths
        while (!pq.empty()) 
        {
            int u = pq.top().second;  // Extract vertex with smallest distance
            pq.pop();

            if (visited[u]) 
              continue;  // Skip if already visited
            visited[u] = true;         // Mark vertex as visited

            // Relax edges from the current vertex
            for (const auto& neighbor : graph[u]) 
            {
                int v = neighbor.to;      // Destination vertex
                int weight = neighbor.weight;  // Weight of the edge

                // Update distance and previous node if shorter path is found
                if (dist[u] + weight < dist[v]) 
                {
                    dist[v] = dist[u] + weight;
                    prev[v] = u;
                    pq.push({dist[v], v});  // Push updated distance and vertex to priority queue
                }
            }
        }

        // Reconstruct shortest path from source to destination
        vector<int> path;
        int current = destination;
        while (current != source) 
        {
            path.push_back(current);
            current = prev[current];
        }
        path.push_back(source);  // Add source node to the path
        reverse(path.begin(), path.end());  // Reverse the path to get source-to-destination order

        shortestPaths.push_back(path);  // Add shortest path to the result vector
    }

    return shortestPaths;
}

// Function to find common segments between paths of two buses
void findCommonSegments(const vector<vector<int>>& bus1Paths, const vector<vector<int>>& bus2Paths) 
{
    // Iterate over paths of both buses
    for (const auto& path1 : bus1Paths) 
    {
        for (const auto& path2 : bus2Paths) 
        {
            int path1Index = 0, path2Index = 0;  // Index for each path

            bool printed = false;  // Flag to track if anything is printed
            bool firstSegmentPrinted = false;  // Flag to track if the first segment is printed

            // Traverse both paths simultaneously
            while (path1Index < path1.size() && path2Index < path2.size()) 
            {
                if (path1[path1Index] == path2[path2Index]) 
                {
                    // If vertices match, print the common vertex
                    if (firstSegmentPrinted) 
                    {
                        cout << "->";  // Print arrow separator
                    } 
                    else 
                    {
                        firstSegmentPrinted = true;  // Mark first segment as printed
                    }
                    cout << path1[path1Index];  // Print common vertex
                    ++path1Index;  // Move to next vertex in path1
                    ++path2Index;  // Move to next vertex in path2
                    printed = true;  // Mark something as printed
                } 
                else 
                {
                    if (printed) 
                    {
                        cout << " ";  // Print space separator if something was printed before
                        cout << endl;  // Move to next line
                        printed = false;  // Reset printed flag
                    }
                    // Move index of path1 or path2 based on which vertex is smaller
                    if (path1[path1Index] < path2[path2Index]) 
                    {
                        ++path1Index;
                    } 
                    else 
                    {
                        ++path2Index;
                    }
                }
            }
            if (printed) 
            {
                cout << " ";  // Print space separator if something was printed before
                cout << endl;  // Move to next line
            }
        }
    }
}

// Function to compute all pairs shortest paths using Floyd Warshall algorithm
vector<vector<int>> floydWarshall(const vector<vector<Edge>>& graph, int n) 
{
    // Initialize distance matrix with maximum values
    vector<vector<int>> dist(n, vector<int>(n, INF));

    // Update distances with edge weights
    for (int i = 0; i < n; ++i) 
    {
        for (const Edge& edge : graph[i]) 
        {
            dist[i][edge.to] = edge.weight;
        }
    }

    // Floyd Warshall algorithm
    for (int k = 0; k < n; ++k) 
    {
        for (int i = 0; i < n; ++i) 
        {
            for (int j = 0; j < n; ++j) 
            {
                if (dist[i][k] != INF && dist[k][j] != INF && dist[i][k] + dist[k][j] < dist[i][j]) 
                {
                    dist[i][j] = dist[i][k] + dist[k][j];
                }
            }
        }
    }

    return dist;  // Return all pairs shortest paths matrix
}

int main() 
{
    int N, M;
    cin >> N >> M;  // Input number of vertices and edges

    vector<vector<Edge>> graph(N);  // Adjacency list representation of the graph

    // Input edges and weights
    for (int i = 0; i < M; ++i) 
    {
        int u, v, k;
        cin >> u >> v >> k;
        graph[u].push_back({v, k});  // Add directed edge from u to v with weight k
    }

    // Compute all pairs shortest paths using Floyd Warshall algorithm
    vector<vector<int>> shortestPaths = floydWarshall(graph, N);

    vector<vector<pair<int, int>>> buses;  // Vector to store buses and their destinations

    string operation;  // Operation to perform
    while (cin >> operation) 
    {
    if (operation == "add_bus") 
    {
    int k, A;
    cin >> k >> A;  // Input number of destinations and starting vertex

    vector<pair<int, int>> destination_times;  // Vector to store destinations and durations

    // Input destinations and initialize durations from the specified starting vertex
    for (int i = 0; i < k; ++i) 
    {
        int B;
        cin >> B;
        int duration = shortestPaths[A][B];  // Duration from starting vertex to destination
        destination_times.push_back({B, duration});  // Add destination with its duration
    }

    cout << endl;
    cout << "Bus " << buses.size() << " updated destinations and durations:" << endl;

    // Print updated destinations and durations
    for (const auto& dest_time : destination_times) 
    {
        int destination = dest_time.first;
        int duration = dest_time.second;
        cout << "Destination: " << destination << ", Duration: " << duration << endl;
    }

    buses.push_back(destination_times);  // Add bus with updated destinations and durations
    }
        else if (operation == "construct_crossroad") 
        {
            int k;
            cin >> k;  // Input number of crossroads

            vector<int> newCrossRoad(k * 3);  // Vector to store crossroads

            // Input crossroads
            for (int i = 0; i < 3 * k; ++i) 
            {
                cin >> newCrossRoad[i];
            }

            cout << endl;

            // Update graph with new crossroads
            for (int i = 0; i < k; ++i) 
            {
                int u = newCrossRoad[i * 3];
                int v = newCrossRoad[i * 3 + 1];
                int duration = newCrossRoad[i * 3 + 2];
                graph[u].push_back({v, duration});  // Add directed edge from u to v with weight duration
            }

            // Recompute all pairs shortest paths using Floyd Warshall algorithm
            shortestPaths.assign(N, vector<int>(N, 0));
            shortestPaths = floydWarshall(graph, N);

            // Print updated destinations and durations for each bus
            for (const auto& bus : buses) 
            {
                cout << "Bus " << (&bus - &buses[0]) << " updated destinations and durations:" << endl;
                for (const auto& dest_time : bus) 
                {
                    int destination = dest_time.first;
                    int duration = shortestPaths[dest_time.second][destination];  // Duration from source to destination
                    cout << "Destination: " << destination << ", Duration: " << duration << endl;
                }
                cout << endl;
            }
        }
        else if (operation == "common_streets") 
        {
            int I, J;
            cin >> I >> J;  // Input bus indices

            // Get destinations for buses I and J
            vector<vector<int>> destinationsI, destinationsJ;
            for (const auto& dest_time : buses[I]) 
            {
                int destination = dest_time.first;
                destinationsI.push_back({destination});
            }
            for (const auto& dest_time : buses[J]) 
            {
                int destination = dest_time.first;
                destinationsJ.push_back({destination});
            }

            // Find shortest paths for destinations of buses I and J
            vector<vector<int>> shortestPathsI = findShortestPaths(graph, destinationsI);
            vector<vector<int>> shortestPathsJ = findShortestPaths(graph, destinationsJ);

            // Find and print common segments between paths of buses I and J
            findCommonSegments(shortestPathsI, shortestPathsJ);
        }
    }

    return 0;
}


/*
6 9
0 1 5
0 2 3
1 3 6
1 4 2
2 1 7
2 3 4
3 5 1
4 5 3
5 0 2


12 15
0 1 1
0 6 2
1 2 7
2 3 5
3 7 15
3 10 12
4 0 3
4 5 8
5 6 6
6 9 10
7 11 4
8 4 6
9 8 4
10 9 11
11 10 5


*/
