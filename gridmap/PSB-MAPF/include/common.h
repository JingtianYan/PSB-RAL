#pragma once

#include <tuple>
#include <list>
#include <vector>
#include <set>
#include <ctime>
#include <fstream>
#include <iostream>     // std::cout, std::fixed
#include <iomanip>      // std::setprecision
#include <chrono>
#include <boost/heap/pairing_heap.hpp>
#include <boost/unordered_set.hpp>
#include <boost/unordered_map.hpp>
#include <boost/graph/adjacency_list.hpp>
#include "conflict.h"
#include <numeric>

using boost::heap::pairing_heap;
using boost::heap::compare;
using boost::unordered_map;
using boost::unordered_set;
using std::vector;
using std::list;
using std::set;
using std::get;
using std::tuple;
using std::make_tuple;
using std::pair;
using std::make_pair;
using std::tie;
using std::min;
using std::max;
using std::shared_ptr;
using std::make_shared;
using std::clock;
using std::cout;
using std::endl;
using std::ofstream;
using std::cerr;
using std::string;

#define INF 10000.0


#define WINDOWS_SIZE 6.0
#define REPLAN_SIZE 4.0

#define CONTROL_POINTS_NUM 30

typedef std::chrono::high_resolution_clock Time;
typedef std::chrono::duration<float> time_s;


struct PathEntry
{
	int location = -1;
    double arrival_time = 0; // arrival time of the head of the vehicle
    double leaving_time_tail = 0; // leaving time of the tail of the vehicle;
    double cost;
	int mdd_width = 0;  // TODO:: Myabe this can be deleted as we always build/look for MDDs when we classify conflicts

	bool is_single() const
	{
		return mdd_width == 1;
	}
	PathEntry(int loc = -1) { location = loc;}
};

typedef vector<PathEntry> Path;
std::ostream& operator<<(std::ostream& os, const Path& path);

bool isSamePath(const Path& p1, const Path& p2);

struct TimeInterval
{
    double t_min;
    double t_max;
    int agent_id;
    int id;
};

typedef vector<list<TimeInterval>> ReservationTable; // vector: vertex -> ordered list of occupied time intervals


struct BezierNode
{
    std::vector<double> control_points;
	double optimal_T = -1;
	double start_time;
    bool valid = false;
};

typedef vector<BezierNode> BezierInfo;