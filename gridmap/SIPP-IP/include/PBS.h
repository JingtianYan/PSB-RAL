// This file is the implementation of PBS
// initialize the PBS tree, run the PBS algorithm for collision node
// call sipp for path planning
#pragma once
#include "SIPP.h"
#include "sipp-ip.h"
#include "PTNode.h"
#include "milp_cache.h"
#include <cstddef>
#include <cstdint>
#include <memory>

// #define DEBUG_PBS 1

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
    bool run(const string&, bool& all_agents_solved);
    bool UpdatePlan(PTNode&, int);
    void initializeAgents();
    void initializePriority(std::map<int, std::set<int> >&);
    void printPath(Path);
    void printRT(ReservationTable);
    void printPriority(std::map<int, std::set<int> >);
    bool SolveSingleAgentSIPP_IP(PTNode& node, int agent_id);
    bool SolveSingleAgentBezier(PTNode& node, int agent_id);
    bool checkValid(ReservationTable& rt, Path& path, int agent);
    bool CheckCollision(PTNode& node, int agent_first, int agent_second);
    void saveResults(int all_agents_solved, const string& fileName, const string& instanceName) const;
    bool GetNextRoundInit();
    std::string getSolverName() const
    {
        return single_agent_solver;
    }
    bool GetNextPreviousPath();
    void printValidRT(ReservationTable rt);
    void updateCost();
    
public:
    std::shared_ptr<Instance> instance_ptr;
    std::shared_ptr<SIPP> sipp_ptr = NULL;
    std::shared_ptr<SIPP_IP> sipp_ip_ptr = NULL;
    std::vector<Agent> arrivingVehicles;
    // std::map<int, std::map<int,std::vector<int>>> trajectoryToAgent;
    std::shared_ptr<CacheMILP> cached_milp_ptr;
    std::string single_agent_solver;
    

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