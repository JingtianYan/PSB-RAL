// This file implement the PT Node
// TODO: what actually this file is working on
#pragma once
#include <iostream>
#include <fstream>
#include <stack>
#include <set>
#include <tuple>
#include <vector>
#include "common.h"
#include "Instance.h"

const double EPSILON = 0.000001;

class PTNode{
public:
	PTNode();
	PTNode(vector<Path>, BezierInfo& solution, std::map<int, std::set<int> >);
    PTNode(const std::shared_ptr<PTNode>& parent): plan(parent->plan), bezier_solution(parent->bezier_solution),
                                                   solution_cost(parent->solution_cost), cost(parent->cost), depth(parent->depth+1),
                                                   makespan(parent->makespan), conflicts(parent->conflicts), parent(parent) {}

	void writeToFile(Instance&, const string&);	
	void calculateCost();
	void calculateCost(std::shared_ptr<Instance> instance_ptr);

	void topologicalSortUtil(int, bool*, std::list<int>&);
	std::list<int> topologicalSort();
	std::list<int> topologicalSort(int);
	std::tuple<int, int, int> getFirstWindowCollision(Instance&);
	std::tuple<int, int, int> getFirstCollision(Instance&);
	void getRTP(std::set<int>&, int);
	void getRTFromP(Instance&, ReservationTable&, std::set<int>);
	void getWindowRTFromP(Instance&, ReservationTable&, std::set<int>&);
	void InsertPreviousPath(Instance& instance , ReservationTable& rt);
	void InsertInitLocation(int agent_id, Instance& instance, ReservationTable& rt);
	void printPriorityMap();
    bool checkSolution(Instance& instance);
	void printPlan();
    void getRTFromP(ReservationTable&, std::set<int>&);
    void insertPathToRT(ReservationTable& rt, int agent_id);
    int checkValid(ReservationTable &rt, int agent_id);
    bool CheckCollision(int agent_first, int agent_second);


public:
    vector<Path> plan; //plan is vector of paths	size of vector = size of vehicles
    BezierInfo bezier_solution;

    std::vector<double> solution_cost;
    double cost = 0.0;
    int depth = 0;
    double makespan = 0;
    // conflicts in the current node
    list<shared_ptr<Conflict> > conflicts;
    shared_ptr<Conflict> conflict;
    shared_ptr<Conflict> curr_conflict;

    std::pair<std::shared_ptr<PTNode>, std::shared_ptr<PTNode>> children;
    Constraint constraint;

    std::map<int, std::set<int>> priority; //map of index with set of lower priority
    std::shared_ptr<PTNode> parent;

};

