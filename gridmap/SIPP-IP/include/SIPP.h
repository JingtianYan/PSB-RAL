#pragma once

#include <iostream>
#include <fstream>
#include <memory>
#include <string>
#include <cstdio>
#include <math.h>
#include <iostream>

#include <ilcplex/ilocplex.h>

#include <chrono>
#include <vector>

#include "Instance.h"
#include "rapidjson/document.h"
#include "rapidjson/filereadstream.h"
#include "berstein.h"
#include "milp_cache.h"

#define EPS 0.0001
// #define CONST_W 2
#define CELL_DIS 1.0
#define TIME_STEP_SIZE 0.05

#define DEBUG_SIPP 0
#define DEBUG_BEZIER 0
#define DEBUG_LOC -1

using std::vector;


typedef vector<Node> Successors;

class NodeCompare
{
public:
    bool operator() (std::shared_ptr<Node> N1, std::shared_ptr<Node> N2) {
        if (N1->f > N2->f) {
            return true;
        } else {
            return false;
        }
    }
};

class SIPP {

public:
    SIPP(std::shared_ptr<Instance> instance);
    bool run(int agent, const ReservationTable& rt, BezierNode& solution, Path& path);
    // float Li(int direction, double agent_length);
    // double estimate_cost(int start_point, int end_point, double speed);

    int find_min(std::vector<std::shared_ptr<Node>>& open);
    // int find(std::vector<std::shared_ptr<Node>>& open);
    bool getSafeIntervals(std::list<TimeInterval> rt, vector<TimeInterval>& safe_intervals);

    // Successors get_successors( 
    //     std::vector<Node> p[],
    //     Node s,
    //     int trajectory_size,
    //     double v_min, 
    //     double v_max, 
    //     double length,
    //     const ReservationTable& rt,
    //     int first_conflict_point_counter);

    std::list<TimeInterval> get_successors( 
        std::shared_ptr<Node> s,
        int to_location,
        const ReservationTable& rt);
    // bool SolveMILP(double& speed, Path& potential_path);

    bool SolveBezier(double T_optimal, std::vector<double>& solution_control_points, Path& result_path);
    bool SolveSlackBezier(double T, std::vector<double>& solution_control_points, 
                            Path& result_path, double& slack_var);
    bool RecurrentSolver(BezierNode& solution, Path& result_path);
    bool SlackGradient(double target_t, double& gradient);
    list<TimeInterval> getSafeIntervals(list<TimeInterval> rt);
    bool GetFirstInterval(const ReservationTable& rt, std::shared_ptr<Node>& first_interval);

private:
    bool dominanceCheck(std::shared_ptr<Node> new_node);
    bool outWindowDominanceCheck(std::shared_ptr<Node> new_node);
    bool RemoveOpenNode(std::shared_ptr<Node> rm_node);
    bool RemoveAllTable(int location, std::shared_ptr<Node> rm_node);
    void PrintNonzeroRT(const ReservationTable& rt);
    void Reset();
    inline int DistHeuristic(int curr_loc)
    {
        int h_val = instance_ptr->getManhattanDistance(curr_loc, curr_agent.goal_location);
        // printf("In location: %d, dist heuristic: %d\n", curr_loc, h_val);
        return h_val;
    }
    bool earlyPruning(std::shared_ptr<Node> new_node);
    bool checkDuplicate(std::shared_ptr<Node> new_node);
    bool Dijstra(int start_loc);
    bool AStarSearch(std::shared_ptr<Node> root_node);
    bool NodeSearch(std::shared_ptr<Node> start_node);
    void updateResultNodes(std::shared_ptr<Node> end_node);

public:
    std::vector<std::vector<std::shared_ptr<Node>>> heuristic_vec;

private:
    // Instance& instance;
    std::shared_ptr<Instance> instance_ptr;
    vector<std::shared_ptr<Node>> result_nodes;
    // vector<Agent> agents;
    Agent curr_agent;
    float w = 7;
    int n_points = CONTROL_POINTS_NUM;
    std::unordered_map<int, std::vector< std::shared_ptr<Node> >> all_nodes_table;
    // vector<std::shared_ptr<Node>> open;
    std::priority_queue<std::shared_ptr<Node>, std::vector<std::shared_ptr<Node>>, NodeCompare> open;
    std::vector<CacheMILP> duplicate_table;
    std::shared_ptr<CacheMILP> cache_ptr;
    std::shared_ptr<CacheMILP> solution_cache_ptr;

    // std::unordered_map<std::string, int> vNameToID;
    // std::vector<int> vNameToDirection;
    // std::map<int, std::map<int, double> > pairDistancesMap;

    double total_runtime_ = 0;
    int count_called = 0;
    // double v_min_;
    // double v_max_;
    // double a_min_;
    // double a_max_;
    // double length_;
    // int start_;
    // int goal_;
    int hit_count_ = 0;
    int bizer_count_ = 0;
    int debug_rt_location_ = 0;
};