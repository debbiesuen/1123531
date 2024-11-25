#include <iostream>
#include <vector>
#include <queue>
#include <unordered_map>
#include <climits>

using namespace std;

// Structure to represent a graph node
struct Edge {
    int destination;
    int weight; // Travel time or cost
};

// Graph class
class Graph {
private:
    unordered_map<int, vector<Edge>> adjList;

public:
    void addEdge(int source, int destination, int weight) {
        adjList[source].push_back({destination, weight});
        adjList[destination].push_back({source, weight}); // Bidirectional graph
    }

    vector<Edge>& getNeighbors(int node) {
        return adjList[node];
    }

    vector<int> getNodes() {
        vector<int> nodes;
        for (auto& entry : adjList) {
            nodes.push_back(entry.first);
        }
        return nodes;
    }
};

// Dijkstra's Algorithm to find the shortest path
vector<int> dijkstra(Graph& graph, int start, int end, int& totalTime) {
    unordered_map<int, int> distances; // Distance from start to each node
    unordered_map<int, int> previous; // To reconstruct the path
    for (auto node : graph.getNodes()) {
        distances[node] = INT_MAX;
    }
    distances[start] = 0;

    auto compare = [](pair<int, int>& a, pair<int, int>& b) {
        return a.second > b.second; // Min-heap based on distance
    };
    priority_queue<pair<int, int>, vector<pair<int, int>>, decltype(compare)> pq(compare);
    pq.push({start, 0});

    while (!pq.empty()) {
        int current = pq.top().first;
        int currentDist = pq.top().second;
        pq.pop();

        if (current == end) break; // Stop when we reach the destination

        for (auto& edge : graph.getNeighbors(current)) {
            int neighbor = edge.destination;
            int weight = edge.weight;
            int newDist = currentDist + weight;

            if (newDist < distances[neighbor]) {
                distances[neighbor] = newDist;
                previous[neighbor] = current;
                pq.push({neighbor, newDist});
            }
        }
    }

    // Reconstruct the shortest path
    vector<int> path;
    int current = end;
    totalTime = distances[end];
    while (current != start) {
        if (previous.find(current) == previous.end()) {
            path.clear();
            return path; // No path found
        }
        path.insert(path.begin(), current);
        current = previous[current];
    }
    path.insert(path.begin(), start);

    return path;
}

int main() {
    Graph graph;

    // Sample transportation network
    graph.addEdge(1, 2, 10); // Node 1 to Node 2 with travel time 10
    graph.addEdge(1, 3, 15);
    graph.addEdge(2, 4, 12);
    graph.addEdge(3, 4, 10);
    graph.addEdge(2, 5, 15);
    graph.addEdge(4, 5, 5);

    cout << "Welcome to the Public Transportation Route Optimizer!" << endl;
    cout << "Nodes represent stations (e.g., 1 = Station A, 2 = Station B, etc.)." << endl;

    int start, end;
    cout << "Enter starting station (number): ";
    cin >> start;
    cout << "Enter destination station (number): ";
    cin >> end;

    int totalTime;
    vector<int> path = dijkstra(graph, start, end, totalTime);

    if (!path.empty()) {
        cout << "Shortest path from " << start << " to " << end << ": ";
        for (size_t i = 0; i < path.size(); ++i) {
            cout << path[i];
            if (i < path.size() - 1) cout << " -> ";
        }
        cout << endl;
        cout << "Total travel time: " << totalTime << " minutes" << endl;
    } else {
        cout << "No path found between the selected stations." << endl;
    }

    return 0;
}
