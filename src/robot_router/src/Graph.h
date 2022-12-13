#ifndef GRAPH_H
#define GRAPH_H

// C++ Program to find Dijkstra's shortest path using
// priority_queue in STL
#include <bits/stdc++.h>
using namespace std;
#define INF 0x3f3f3f3f
 
// iPair ==> Integer Pair
typedef pair<int, int> iPair;

struct PlannerResponse{
    vector<double> distances;
    vector<vector<int>> directions; 
};

// This class represents a directed graph using
// adjacency list representation
class Graph {
    int V; // No. of vertices
 
    // In a weighted graph, we need to store vertex
    // and weight pair for every edge
    list<pair<int, int> >* adj;
 
public:
    Graph(int V); // Constructor
 
    // function to add an edge to graph
    void addEdge(int u, int v, int w);
 
    // prints shortest path from s
    PlannerResponse shortestPath(int s);
};
 
#endif // !GRAPH_H
