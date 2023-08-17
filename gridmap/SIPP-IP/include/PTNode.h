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

#define DEBUG_PTNODE 0

const double EPSILON = 0.000001;

class PTNode{
public:
	vector<Path> plan; //plan is vector of paths	size of vector = size of vehicles
	// std::vector<std::vector<double>> control_points;
	// std::vector<double> optimal_T;
	// std::vector<double> start_time;
	BezierInfo bezier_solution;

	std::map<int, std::set<int>> priority; //map of index with set of lower prio
	double cost;

	PTNode();
	PTNode(vector<Path>, BezierInfo& solution, std::map<int, std::set<int> >);
	void writeToFile(Instance&, const string&);	
	void calculateCost();
	void calculateCost(std::shared_ptr<Instance> instance_ptr);

	void topologicalSortUtil(int, bool*, std::list<int>&);
	std::list<int> topologicalSort();
	std::list<int> topologicalSort(int);
	std::tuple<int, int, int> getFirstWindowCollision(Instance&);
	std::tuple<int, int, int> getFirstCollision(Instance&);
	//void getRT(ReservationTable&, int);
	void getRTP(std::set<int>&, int);
	void getRTFromP(Instance&, ReservationTable&, std::set<int>);
	void getWindowRTFromP(Instance&, ReservationTable&, std::set<int>&);
	void InsertPreviousPath(Instance& instance , ReservationTable& rt);
	void InsertInitLocation(int agent_id, Instance& instance, ReservationTable& rt);

	// void getRTFromP(Instance&, ReservationTable&, std::set<int>, int,std::map<int, std::map<int,std::vector<int> > >&);
	void getRTFromP(Instance&, ReservationTable&, int,std::map<int, std::map<int,std::vector<int> > >&);
	void printPriorityMap();
	void printPlan();

};

