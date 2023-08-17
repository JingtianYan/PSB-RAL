#pragma once
#include "SIPP.h"
#include "PTNode.h"

extern const double EPSILON;

class PBS
{
public:
    PBS(Instance& instance);
    void run(const string&);
    //void writeToFile(const string& file_name);
    //ReservationTable getReservationTable(PTNode, std::list<int>);
    bool UpdatePlan(PTNode&, int);
    SIPP sipp;
    Instance& instance;
    std::vector<Agent> arrivingVehicles;
    std::map<int, std::map<int,std::vector<int> > > trajectoryToAgent;
    void initializeAgents(std::map<int, std::map<int, std::vector<int> > >&);
    void initializePriority(std::map<int, std::set<int> >&, std::map<int, std::map<int, std::vector<int> > >&);
    void printPath(Path);
    void printRT(ReservationTable);
    void printPriority(std::map<int, std::set<int> >);
    bool checkValid(ReservationTable& rt, Path& path, int agent);
    void saveResults(const string& fileName, const string& instanceName) const;

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
    double total_delay = 0.0;
};