// This file is the implementation of PBS
// initialize the PBS tree, run the PBS algorithm for collision node
// call sipp for path planning
#pragma once
#include "SIPP.h"
#include "PTNode.h"
#include "milp_cache.h"
#include <memory>

extern const double EPSILON;

class PBS
{
public:
    PBS(std::shared_ptr<Instance> instance);
    bool run(const string&);
    bool UpdatePlan(PTNode&, int);
    void initializeAgents();
    void initializePriority(std::map<int, std::set<int> >&);
    void printPath(Path);
    void printRT(ReservationTable);
    void printPriority(std::map<int, std::set<int> >);
    bool checkValid(ReservationTable& rt, Path& path, int agent);
    void saveResults(const string& fileName, const string& instanceName) const;

public:
    std::shared_ptr<Instance> instance_ptr;
    std::shared_ptr<SIPP> sipp_ptr;
    std::vector<Agent> arrivingVehicles;
    std::map<int, std::map<int,std::vector<int>>> trajectoryToAgent;
    std::shared_ptr<CacheMILP> cached_milp_ptr;

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

    bool solution_found = false;
	double solution_cost = -2;
    double root_cost = 0;
    double cutoff_runtime = 0;
    double solution_delay = 0.0;

    std::shared_ptr<PTNode> solution_node;    
};