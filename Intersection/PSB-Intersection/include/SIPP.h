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

#include "Instance.h"
#include "rapidjson/document.h"
#include "rapidjson/filereadstream.h"
#include "berstein.h"
#include "milp_cache.h"
#define CONST_W 2

// #define DEBUG_SIPP 0
#define EPS 0.0001

#ifdef DEBUG_SIPP
#define DEBUG_PRINT(x) x
#else
#define DEBUG_PRINT(x) 
#endif

using std::vector;


typedef vector<Node> Successors;


class SIPP {

public:
    SIPP(std::shared_ptr<Instance> instance);
    Path run(int agent, const ReservationTable& rt, std::shared_ptr<CacheMILP> cache_ptr,double& optimal_T, double& start_time,
                     std::vector<double>& solution_control_points);
    float Li(int direction, double agent_length);
    double estimate_cost(int start_point, int end_point, double speed);

    int find_min(std::vector<Node>& open);
    int find(std::vector<Node>& open);
    int find_point(int n ,Node* p, int current_point);
    std::list<TimeInterval> getSafeIntervals(std::list<TimeInterval> rt);

    Successors get_successors( 
        std::vector<Node> p[],
        Node s,
        int trajectory_size,
        double v_min, 
        double v_max, 
        double length,
        const ReservationTable& rt,
        int first_conflict_point_counter);

    bool SolveMILP(int agentID, double& milp_run_time);
    bool SolveBezier(int agentID, double T_optimal, double& T_length, double& earliest_start_time, std::vector<double>& solution_control_points, Path& result_path);
    bool SolveSlackBezier(int agentID, double T, std::vector<double>& solution_control_points, 
                            Path& result_path, double& slack_var);
    bool RecurrentSolver(int agentID, double& optimal_T, double& T_length, double& earliest_start_time, 
                            std::vector<double>& solution_control_points, Path& result_path);
    double SlackGradient(int agentID, double target_t);

private:
    // Instance& instance;
    std::shared_ptr<Instance> instance_ptr;
    vector<Node> result_nodes;
    vector<Agent> agents;
    float turn_radius_right = 10*0.3048;
    float turn_radius_left = 40*0.3048;
    float w = 7;

    std::unordered_map<std::string, int> vNameToID;
    std::vector<int> vNameToDirection;
    std::map<int, std::map<int, double> > pairDistancesMap;

    double total_runtime_ = 0;
    double count_called = 0;
    int hit_count_ = 0;
    int bizer_count_ = 0;
};