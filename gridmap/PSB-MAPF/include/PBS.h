/**
 * This file is the implementation of PBS
 *
 * initialize the PBS tree, run the PBS algorithm for collision node
 * call SIPP for path planning.
 */

#pragma once
#include "SIPP.h"
#include "PTNode.h"
#include "milp_cache.h"
#include <cstddef>
#include <cstdint>
#include <memory>


#ifdef DEBUG_PBS 
#define DEBUG(x) x
#define DEBUG_PRINT 1
#else 
#define DEBUG(x)
#define DEBUG_PRINT 0
#endif

extern const double EPSILON;


class PBS
{
public:
    PBS(std::shared_ptr<Instance> instance, std::string single_agent_solver_name, double cutoff_time);
    bool solve(const string& outputFileName, bool& all_agent_solved);
    void printPath(Path);
    void printRT(ReservationTable);
    bool SolveSingleAgentBezier(PTNode& node, std::set<int>& rtp, int agent_id);
    bool checkValid(ReservationTable& rt, Path& path, int agent);
    bool CheckCollision(PTNode& node, int agent_first, int agent_second);
    void saveResults(int all_agents_solved, const string& fileName, const string& instanceName) const;
    bool GetNextRoundInit();
    std::string getSolverName() const
    {
        return single_agent_solver;
    }
    void printValidRT(ReservationTable rt);
    void updateCost();

private:
    bool initRootNode(std::shared_ptr<PTNode>& root_node);
    inline bool isTerminate(std::shared_ptr<PTNode> curr_n);
    std::shared_ptr<PTNode> selectNode();
    shared_ptr<Conflict> chooseConflict(const PTNode &node);
    void pushNodes(const std::shared_ptr<PTNode>& n1, const std::shared_ptr<PTNode>& n2);
    inline void pushNode(const std::shared_ptr<PTNode>& node);
    bool generateChild(int child_id, std::shared_ptr<PTNode> parent, int low, int high);
    void printPriorityGraph() const;
    void topologicalSort(list<int>& stack);
    void topologicalSortUtil(int v, vector<bool> & visited, list<int> & stack);
    void getHigherPriorityAgents(const list<int>::reverse_iterator & p1, set<int>& agents);
    void getLowerPriorityAgents(const list<int>::iterator & p1, set<int>& agents);
    bool hasHigherPriority(int low, int high) const; // return true if agent low is lower than agent high
    void recursivePrint(const std::shared_ptr<PTNode>& curr_node);

    
public:
    std::shared_ptr<Instance> instance_ptr;
    std::shared_ptr<SIPP> sipp_ptr = NULL;
    std::vector<Agent> arrivingVehicles;
    std::shared_ptr<CacheMILP> cached_milp_ptr;
    std::string single_agent_solver;

private:
    clock_t global_start_t;
    double time_limit;
    int screen=0;
    std::stack<std::shared_ptr<PTNode>> open_list;
    vector<vector<bool>> priority_graph;
    list<int> ordered_agents;
    int num_of_agents;
    

private:
/////////////////////////////////////////////////////////////////////////////////////
	// stats
	double runtime = 0;
	double runtime_generate_child = 0; // runtimr of generating child nodes
	double runtime_build_CT = 0; // runtimr of building constraint table
	double runtime_build_CAT = 0; // runtime of building conflict avoidance table
	double runtime_path_finding = 0; // runtime of finding paths for single agents
	double runtime_detect_conflicts = 0;
	double runtime_preprocessing = 0; // runtime of building heuristic table for the low level

    uint64_t num_HL_expanded = 0;
	uint64_t num_HL_generated = 0;
	uint64_t num_LL_expanded = 0;
	uint64_t num_LL_generated = 0;
    uint64_t num_PBS_called = 0;

    bool solution_found = false;
	double solution_cost = -2;
    double root_cost = 0;
    double cutoff_runtime = 0;

    std::shared_ptr<PTNode> solution_node;
};