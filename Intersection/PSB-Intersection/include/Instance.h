#pragma once
#include <set>
#include <unordered_map>
#include <iostream>
#include <boost/graph/adjacency_list.hpp>
#include <fstream>
#include <string>
#include <cstdio>
#include <math.h>

#include "Common.h"
#include "rapidjson/document.h"
#include "rapidjson/filereadstream.h"

typedef boost::adjacency_list_traits<boost::vecS, boost::vecS, boost::undirectedS > searchGraphTraits_t;
typedef searchGraphTraits_t::vertex_descriptor vertex_t;
typedef searchGraphTraits_t::edge_descriptor edge_t;

struct position_t
{
    float x;
    float y;
    position_t(){};
    position_t(float a, float b) {
        x = a;
        y = b;
    };
};

struct Vertex
{
    std::string name;
    position_t pos;
    std::set<int> generalizedVertexConflicts;
    int id;
};

struct Edge
{
    std::string name;
    float length;
};

struct Node
{
    int current_point;
    int next_point;
    int previous_point;
    
    double arrival_time_min; // arrival time of the head of the vehicle
    double arrival_time_max; // arrival time of the head of the vehicle
    
    double cost_min;
	double g;      // 
	double f;      // f = h + g;
	double h;      // 
	int color;  // marking the point to see whether it was visited or not

    int interval_index;
    double interval_t_min;
    double interval_t_max;

    int index;
    Node *parent;

    int parent_index_1;
    int parent_index_2;
};

typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::undirectedS, Vertex, Edge> searchGraph_t;

struct Agent
{
    int id;
    int start_location;
    int goal_location;
    int importance;

    string name;
    std::vector<int> trajectory;

    double earliest_start_time;

    double v_min;
    double v_max;
    double a_min;
    double a_max;

    double length; // length of the vehicle
    double velocity;
};


class Instance
{
public:
    Instance(int step, const string& map_name, const string& pair_distance_file, const string& vehicle_file);
    vector<Agent> agents;
    int getNumOfVertices() const;
    void loadVehicles();
    //void loadVehicles(const std::string& arrivalFile, int step, vector<Agent> &agents, std::unordered_map<std::string, int>& vNameToID);
    void loadSearchGraph();
    position_t nodeAsPos(const float x, const float y);

    std::map<int, std::map<int, double> > getPairDistancesMap();
    std::vector<int> getVNameToDirection();
    std::unordered_map<std::string, int> getVNameToID();
    std::unordered_map<int, std::string> getVIDToName();
    std::unordered_map<int, std::vector<int>> getVIDToConflictPoints();
    std::vector<Agent> getAgents();
    std::vector<int> getConflictPoints(int);
    double getEarliestStartTime(int);
    int getStartCP(int index);
    void loadSamePoint();
    bool isSamePoint(int, int);

private:
    int _step;
    std::string _graph_name;
    std::string _pair_distance_filename;
    std::string _vehicle_arrival_filename;

    searchGraph_t searchGraph;
    std::unordered_map<std::string, int> vNameToID;
    std::unordered_map<int, std::string> vIDToName;
    std::vector<vertex_t> vNameToV;
    std::vector<int> vNameToDirection;

    rapidjson::Document pairDistances;
    std::map<int, std::map<int, double> > pairDistancesMap;
    std::unordered_map<int, std::vector<int> > vIDToConflictPoints;
    bool samePoint[104][104] = {{false}};
};