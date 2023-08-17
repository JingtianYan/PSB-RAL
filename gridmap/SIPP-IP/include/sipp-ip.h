#pragma once

#include <iostream>
#include <algorithm>
#include <set>
#include <utility>
#include <vector>
#include <chrono>
#include <fstream>
#include <iomanip>
#include <cassert>
#include "Instance.h"
#include "common.h"

using namespace std;
using namespace std::chrono;
// Horizon timestep. This equals to INF/F (seconds). 
// Please be attention that this variable affects the stored memory in generate_obstacles file.
#define MXO 4 // number of different orientation. 0 is the East, then rotate with counterclockwise
#define MXV 2 // number of different velocities.
#define F 10 // number of timesteps in one second i.e. 1/T

// /** map random128 ready specifications. Uncomment the three lines below to use this map. **/
// #define MXH 128 // maximum height of the environment
// #define MXW 128 // maximum width of the environment

/** map random32 ready specifications. Uncomment the three lines below to use this map. **/
#define MXH 32 // maximum height of the environment
#define MXW 32 // maximum width of the environment

// /** map empty16 ready specifications. Uncomment the three lines below to use this map. **/
// #define MXH 16 // maximum height of the environment
// #define MXW 16 // maximum width of the environment

// /** map lak303d ready specifications. Uncomment the three lines below to use this map. **/
// #define MXH 194 // maximum height of the environment
// #define MXW 194 // maximum width of the environment

// /** map random64 ready specifications. Uncomment the three lines below to use this map. **/
// #define MXH 64 // maximum height of the environment
// #define MXW 64 // maximum width of the environment

// /** map Boston ready specifications. Uncomment the three lines below to use this map. **/
// #define MXH 256 // maximum height of the environment
// #define MXW 256 // maximum width of the environment

// /** map den312d ready specifications. Uncomment the three lines below to use this map. **/
// #define MXH 81 // maximum height of the environment
// #define MXW 65 // maximum width of the environment

// /** map den312d ready specifications. Uncomment the three lines below to use this map. **/
// #define MXH 141 // maximum height of the environment
// #define MXW 162 // maximum width of the environment

#define SIPP_STEP_SIZE 1


class Primitive{
public:
    struct move{
        // ftt (first-touch-time) = (t_lower) in the paper. 
        // swt (sweeping time) = (t_upper - t_lower) in the paper.
        move(int dx, int dy, int ftt, int swt, bool isEndCell): dx(dx), dy(dy), ftt(ftt), swt(swt), isEndCell(isEndCell){}
        int dx, dy, ftt, swt;
        bool isEndCell; // flag if this cell is still touched at the end of the primitive. `swt` in this case is equal to the remaining time until the end of the primitive.
    };
    vector<move> mvs;
    int o, v;
    Primitive(){
        mvs.clear();
        o = v = 0;
    }
    Primitive(vector<move> mvs, int o, int v):
    mvs(std::move(mvs)),o(o),v(v){}
    friend ostream &operator<<( ostream &output, const Primitive &p ) {
        output << "Moves:"<<endl;
        for(auto it:p.mvs){
            output << "  dx:"<<it.dx<<", dy:"<<it.dy<<", ftt:"<<it.ftt<<", swt:"<<it.swt<<endl;
        }
        output<<"Final_o:"<<p.o<<", Final_v:"<<p.v<<endl;
        return output;
    }
};

class Bot{
public:
    int x, y, o, v, t_lower, t_upper, parentID;
    int end_x, end_y, end_v;
    int minimalTransitionCost;
    Primitive prev_pm;

    Bot(int x, int y, int o, int v, int t_lower, int t_upper, 
        int end_x, int end_y, int end_v, int minimalTransitionCost, Primitive mp)
        :x(x),y(y),o(o),v(v),t_lower(t_lower),t_upper(t_upper), 
        end_x(end_x), end_y(end_y), end_v(end_v), minimalTransitionCost(minimalTransitionCost), prev_pm(mp)
    {
        parentID = -1;
    }
    Bot(int x, int y, int o, int v, int t_lower, int t_upper, int parentID,
        int end_x, int end_y, int end_v, int minimalTransitionCost):
        x(x),y(y),o(o),v(v),t_lower(t_lower),t_upper(t_upper),parentID(parentID),
        end_x(end_x), end_y(end_y), end_v(end_v), minimalTransitionCost(minimalTransitionCost)
        {}
    Bot(){
        x = y = o = v = t_lower = t_upper = 0;
        parentID=-1;
    }
    Bot(const Bot&) = default;
    Bot &operator=(const Bot&other)= default;
    friend bool operator == (const Bot& a, const Bot& b)
    {
    return a.x == b.x && a.y == b.y && a.o == b.o && a.v == b.v &&
             a.t_lower == b.t_lower && a.t_upper == b.t_upper && a.parentID==b.parentID;
    }
    friend bool operator < (const Bot& a, const Bot& b)
    {
        if(a.t_lower+(abs(a.x-a.end_x)+abs(a.y-a.end_y))*a.minimalTransitionCost != 
            b.t_lower+(abs(b.x-a.end_x)+abs(b.y-a.end_y))*a.minimalTransitionCost) {
            return a.t_lower + (abs(a.x - a.end_x) + abs(a.y - a.end_y))*a.minimalTransitionCost < 
                b.t_lower + (abs(b.x - a.end_x) + abs(b.y - a.end_y))*a.minimalTransitionCost;
        }
        if(abs(a.x-a.end_x)+abs(a.y-a.end_y) != abs(b.x-a.end_x)+abs(b.y-a.end_y)) {
            return abs(a.x - a.end_x) + abs(a.y - a.end_y) < abs(b.x - a.end_x) + abs(b.y - a.end_y);
        }
        if(a.t_upper != b.t_upper){
            return a.t_upper > b.t_upper;
        }
        return ((a.x*MXW+a.y)*4+a.o)*MXV+a.v
            < ((b.x*MXW+b.y)*4+b.o)*MXV+b.v;
    }
    friend ostream &operator<<( ostream &output, const Bot &p ) {
        output<<"x="<<p.x<<", y="<<p.y<<", o="<<p.o<<", v="<<p.v<<", t_lower="<<p.t_lower<<", t_upper="<<p.t_upper<<", parentID="<<p.parentID;
        return output;
    }
};

class SIPP_IP
{
public:
    SIPP_IP(std::shared_ptr<Instance> instance);
    Path run(int agent, const ReservationTable& rt,  double cutoff_time);
    void showTable();


private:
    bool outMap(int x, int y);
    int checkCLOSED(int x, int y, int o, int v, int t_lower);
    vector<Bot> applyPrimitive(int x, int y, int t_lower, int t_upper, Primitive mp);
    void generateSuccessors(int x, int y, int o, int v, int t_lower, int t_upper, vector<Bot>& succs);
    void getSolutionStates(int goalID, vector<Bot> &vecStates);
    void clr();
    void fillActions();
    Path RetrivePath(int goal_id);
    void RVToRSRV(const ReservationTable& rt);

public:
    int low_node_generated = 0;
    int low_node_expanded = 0;

private:
    const int MAX_NUM_NODES = 100000000; // one hundred million nodes (OPEN+CLOSED sets' nodes)
    ReservationTable debug_rt;
    int H, W;
    int minimalTransitionCost;
    std::set<pair<int, int>> rsrv_tbl[MXH][MXW];
    vector<Primitive> motion_primitives[MXO][MXV];

    std::set<Bot> OPEN;
    vector<Bot> CLOSED_vec;
    std::set<pair<int, int>> CLOSED[MXH][MXW][MXO][MXV];

    std::shared_ptr<Instance> instance_ptr;
    std::vector<Agent> agents;
    Agent curr_agent;
    int end_x;
    int end_y;
    int end_v;
};