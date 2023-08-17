#include "../include/sipp-ip.h"
#include "common.h"
#include <cstdio>

bool SIPP_IP::outMap(int x, int y) {
    return x<0 || x>=H || y<0 || y>=W;
}

// Check the CLOSED set if any state intersect with the given interval.
// return new t_lower (because the intersection (if exists) will be from lower bound always).
int SIPP_IP::checkCLOSED(int x, int y, int o, int v, int t_lower){
    if(CLOSED[x][y][o][v].empty())
        return t_lower;
    // there shouldn't be any interval in CLOSED with lower_bound > t_lower.
    auto it = --CLOSED[x][y][o][v].end();
    return max(t_lower, it->second+1);
}

vector<Bot> SIPP_IP::applyPrimitive(int x, int y, int t_lower, int t_upper, Primitive mp) {
    vector<pair<int, int>> timeIntervals, tmp;
    timeIntervals.push_back({t_lower, t_upper});
    bool endCellTouched = false;
    int xx = x, yy = y, past_time = 0;
    for(auto mv:mp.mvs){
        xx = x + mv.dx;
        yy = y + mv.dy;
        if (outMap(xx, yy)) {
            return {};
        }
        for(auto &it:timeIntervals){
            int t_l = min((int) INF, it.first + (mv.ftt - past_time));
            int t_u = min((int) INF, it.second + (mv.ftt - past_time));
            
            auto itr= rsrv_tbl[xx][yy].lower_bound({t_l, INF - 1});
            auto itr_prev = itr;
            --itr_prev;
            while (itr_prev->second < t_u) {
                int new_tlower = max(t_l, itr_prev->second + 1);
                int new_tupper = min(t_u, itr->first - 1 - mv.swt);
                // printf("At location: %d, Generate new interval from %d to %d\n", instance_ptr->linearizeCoordinate(xx, yy),
                //     new_tlower, new_tupper);
                if (new_tlower <= new_tupper && (mv.isEndCell && !endCellTouched) &&
                    mp.v == 0) { // if there is at least one timestep to get there and vel there is zero and this is the first end-cell in the primitive.
                    new_tupper = itr->first - 1 - mv.swt;
                }
                if (new_tupper - new_tlower >= mv.swt){
                    tmp.push_back({new_tlower, new_tupper});
                }
                ++itr;
                ++itr_prev;
            }
        }
        timeIntervals = tmp;
        tmp.clear();
        past_time = mv.ftt;
        if(mv.isEndCell){
            endCellTouched = true;
        }
    }
    vector<Bot> vBot;
    for(auto it:timeIntervals){
        vBot.push_back(Bot(xx, yy, mp.o, mp.v, min((int) INF, it.first+mp.mvs.back().swt), 
                            min((int) INF,it.second+mp.mvs.back().swt), 
                            end_x, end_y, end_v, minimalTransitionCost, mp));
    }
    return vBot;
}

void SIPP_IP::generateSuccessors(int x, int y, int o, int v, int t_lower, int t_upper, vector<Bot>& succs) {
    for (auto &mp: motion_primitives[o][v]) {
        auto vBots = applyPrimitive(x, y, t_lower, t_upper, mp);
        move(vBots.begin(), vBots.end(), back_inserter(succs));
    }
}

void SIPP_IP::getSolutionStates(int goalID, vector<Bot> &vecStates){
    vecStates.clear();
    while(goalID!=-1){
        vecStates.push_back(CLOSED_vec[goalID]);
        goalID = CLOSED_vec[goalID].parentID;
    }
    // reverse(vecStates.begin(), vecStates.end());
}

void SIPP_IP::showTable()
{
    for(int i=0; i<MXH; ++i){
        for(int j=0; j<MXW; ++j){
            printf("At location cp %d:\t", instance_ptr->linearizeCoordinate(i, j));
            for (std::pair<int, int> tmp_slot: rsrv_tbl[i][j]) {
                printf("%d\t%d  ####  ", tmp_slot.first, tmp_slot.second);
            }
            printf("\n");
        }
    }
}

void SIPP_IP::RVToRSRV(const ReservationTable& rt)
{
    clr();
    for(int i=0; i<MXH; ++i){
        for(int j=0; j<MXW; ++j){
            for (TimeInterval tmp_interval: rt[instance_ptr->linearizeCoordinate(i, j)]) {
                printf("New time interval from %f to %f\n", tmp_interval.t_min, tmp_interval.t_max);
                int t_lower = (int) floor(tmp_interval.t_min*F);
                int t_upper = (int) ceil(tmp_interval.t_max*F);
                std::pair<int, int> tmp_t(t_lower, t_upper);
                if (rsrv_tbl[i][j].size() <= 2) {
                    rsrv_tbl[i][j].insert({t_lower, t_upper});
                } else {
                    bool flag = true;
                    for (auto interval_it = rsrv_tbl[i][j].begin(); interval_it != rsrv_tbl[i][j].end(); ) {
                        if (interval_it->second <= tmp_t.first or interval_it->first >= tmp_t.second) {
                            interval_it++;
                        } else {
                            if (interval_it->first <= tmp_t.first and interval_it->second >= tmp_t.second) {
                                flag = false;
                                break;
                            }
                            tmp_t.first = min(interval_it->first, tmp_t.first);
                            tmp_t.second = max(interval_it->second, tmp_t.second);
                            interval_it = rsrv_tbl[i][j].erase(interval_it);
                        }
                        // if (interval_it->first <= tmp_t.first and interval_it->second >= tmp_t.second) {
                        //     flag = false;
                        //     break;
                        // } else if (interval_it->first <= tmp_t.first and interval_it->second <= tmp_t.second) {
                        //     tmp_t.first = interval_it->first;
                        //     interval_it = rsrv_tbl[i][j].erase(interval_it);
                        // } else if (interval_it->first >= tmp_t.first and interval_it->second >= tmp_t.second) {
                        //     tmp_t.second = interval_it->second;
                        //     interval_it = rsrv_tbl[i][j].erase(interval_it);
                        // } else if (interval_it->first >= tmp_t.first and interval_it->second <= tmp_t.second) {
                        //     tmp_t.first = interval_it->first;
                        //     tmp_t.second = interval_it->second;
                        //     interval_it = rsrv_tbl[i][j].erase(interval_it);
                        // } else {
                        //     interval_it++;
                        // }
                    }
                    if (flag) {
                        rsrv_tbl[i][j].insert(tmp_t);
                    }
                }
                // printf("At location %d, insert Interval: %f, %f\n", instance_ptr->linearizeCoordinate(i, j), floor(tmp_interval.t_min*F), ceil(tmp_interval.t_max*F));
            }
        }
    }
    // showTable();
}

Path SIPP_IP::run(int agent, const ReservationTable& rt){
    // printf("Single agent solver for Agent %d\n", agent);
    low_node_generated = 0;
    low_node_expanded = 0;
    curr_agent = agents[agent];
    RVToRSRV(rt);
    int st_x = instance_ptr->getRowCoordinate(curr_agent.start_location);
    int st_y = instance_ptr->getColCoordinate(curr_agent.start_location);
    int st_o = 0;
    int st_v = 0;
    int st_tlower = 0;

    end_x = instance_ptr->getRowCoordinate(curr_agent.goal_location);
    end_y = instance_ptr->getColCoordinate(curr_agent.goal_location);
    end_v = 0;

    auto it = rsrv_tbl[st_x][st_y].lower_bound({st_tlower,INF});
    int st_tupper = it->first - 1;
    --it;
    if (st_tupper < st_tlower || it->second >= st_tlower) {
        // cout<<"Error, the initial state falls in an obstacle!!"<<endl;
        Path result_empty_path;
        // printf("[INFO] No solution find for Agent %d\n", agent);
        return result_empty_path;
    }
    OPEN.clear();
    CLOSED_vec.clear();
    OPEN.insert(Bot(st_x, st_y, st_o, st_v, st_tlower, st_tupper, -1, end_x, end_y, end_v, minimalTransitionCost));
    int x, y, o, v, t_lower, t_upper, id;
    int cntNodes = 0;

    while(!OPEN.empty() && cntNodes<MAX_NUM_NODES){
        auto tmp = *OPEN.begin();
        OPEN.erase(OPEN.begin());
        // printf("pop out new state with location: %d, %d\n", tmp.x, tmp.y);
        --cntNodes;
        x = tmp.x, y = tmp.y, o = tmp.o, v = tmp.v, t_lower = tmp.t_lower, t_upper = tmp.t_upper;
        auto new_t_lower = checkCLOSED(x, y, o, v, t_lower);
        if(new_t_lower > t_upper){
            continue;
        }
        if(t_lower!=new_t_lower){ // if the state got trimmed, insert it again in OPEN.
            tmp.t_lower = new_t_lower;
            OPEN.insert(tmp);
            continue;
        }
        CLOSED[x][y][o][v].insert({t_lower, t_upper});
        ++cntNodes;
        id = CLOSED_vec.size();
        CLOSED_vec.emplace_back(tmp);
        // if(x == end_x && y == end_y && v == end_v){
        if(x == end_x && y == end_y and t_upper == INF){
            Path result_path = RetrivePath(id);
            return result_path;
        }
        vector<Bot> succs;
        low_node_expanded++;
        generateSuccessors(x, y, o, v, t_lower, t_upper, succs);
        for(auto &it:succs){
            it.parentID = id;
            OPEN.insert(it);
            ++cntNodes;
            low_node_generated++;
        }
    }
    // printf("Total number of node expanded: %d\n", cntNodes);
    Path result_empty_path;
    // printf("[INFO] No solution find for Agent %d\n", agent);
    return result_empty_path;
}


void SIPP_IP::clr(){
    for(int i=0; i<MXH; ++i){
        for(int j=0; j<MXW; ++j){
            rsrv_tbl[i][j].clear();
            rsrv_tbl[i][j].insert({-1, -1});
            rsrv_tbl[i][j].insert({INF+1, INF+1});
            for(int k=0; k<MXO; ++k){
                for(int l=0; l<MXV; ++l){
                    CLOSED[i][j][k][l].clear();
                }
            }
        }
    }
}

void SIPP_IP::fillActions() {
    minimalTransitionCost = 5; // the minimum cost to go from one cell to next (0.5s = 5 timesteps).
    // turn actions
    for (int o = 0; o < MXO; ++o) {
        Primitive tmp;
        // Set the rotate time to 0
        tmp.mvs = {Primitive::move(0, 0, 0, 0, 1)}; // two seconds (20 timesteps) to rotate.
        tmp.o = (o + 1) % MXO;
        tmp.v = 0;
        motion_primitives[o][0].emplace_back(tmp);
        motion_primitives[o][1].emplace_back(tmp);
        tmp.o = (o + 3) % MXO;
        motion_primitives[o][0].emplace_back(tmp);
        motion_primitives[o][1].emplace_back(tmp);
    }
    int dx[4] = {0, -1, 0, 1}, dy[4] = {1, 0, -1, 0};
    // Acceleration and Deceleration primitives:
    // take 4 cells and 4 seconds (40 timesteps) to finish - a=+-0.5m/s^2, v_max=2m/s.
    vector<pair<int, int>> costs = {make_pair(0,20), make_pair(0,29), make_pair(20, 15), make_pair(28,12), make_pair(34,6)};
    int full_cost = 40;
    int sz = costs.size();
    for (int o = 0; o < MXO; ++o) {
        Primitive tmp;
        tmp.v = 1;
        tmp.o=o;
        for(int j=0; j<sz; ++j){
            tmp.mvs.push_back(Primitive::move(dx[o]*j,dy[o]*j,costs[j].first, costs[j].second,int(j==sz-1)));
        }
        motion_primitives[o][0].push_back(tmp);
        tmp.mvs.clear();
        tmp.v = 0;
        for(int j=0; j<sz; ++j){
            tmp.mvs.push_back(Primitive::move(dx[o]*j,dy[o]*j,full_cost-(costs[sz-1-j].first+costs[sz-1-j].second), costs[sz-1-j].second,int(j==sz-1)));
        }
        motion_primitives[o][1].push_back(tmp);
        tmp.mvs = {Primitive::move(0, 0, 0, 5, 0), Primitive::move(dx[o], dy[o], 0, 5, 1)};
        tmp.o = o;
        tmp.v = 1;
        motion_primitives[o][1].emplace_back(tmp);
    }
}

Path SIPP_IP::RetrivePath(int goal_id){
    Path result_path;
    vector<Bot> v;
    getSolutionStates(goal_id, v);
    // cout<<"Solution states:\n";
    // for (auto it: v) {
    //     cout << "At location: " << instance_ptr->linearizeCoordinate(it.x, it.y) << "\t";
    //     cout << it << endl;
    // }
    int curr_time = 0;
    int start_time = 0;
    int last_leave_time = 0;
    int last_arr_time = 0;
    PathEntry tmp_entry;

    for (unsigned int i = 0; i < v.size()-1; i++) {
        Bot curr_node = v[i];
        Bot prev_node = v[i+1];
        int speed = curr_node.v;
        Primitive pm = curr_node.prev_pm;
        if (i == 0) {
            curr_time = curr_node.t_lower;
            // last_leave_time = INF;
            last_arr_time = curr_time;
            last_leave_time = curr_time + pm.mvs.back().swt;
        } else {
            // If speed is 0, the curr time is lower_bound (can stop at any where)
            if (speed == 0) {
                // printf("Speed is zeros\n");
                curr_time = curr_node.t_lower - pm.mvs.back().swt;
            } else {
                curr_time = last_arr_time - pm.mvs.back().swt;
            }
        }
        // printf("[INFO] Current time is: %d\n", curr_time);

        if (pm.mvs.size() == 1) {
            // printf("[INFO] Ignore rotation operation!\n");
            if (i == (v.size() - 2)) {
                tmp_entry.location = instance_ptr->linearizeCoordinate(prev_node.x, prev_node.y);
                tmp_entry.arrival_time = (curr_time*1.0)/F;
                tmp_entry.leaving_time_tail = (last_leave_time*1.0)/F;
                result_path.push_back(tmp_entry);
            }
            continue;
        }
        // printf("[INFO] Execute new action!\n");

        for (unsigned int j = (pm.mvs.size()-1); ; j--) {
            Primitive::move mv = pm.mvs[j];
            int x = prev_node.x + mv.dx;
            int y = prev_node.y + mv.dy;
            tmp_entry.location = instance_ptr->linearizeCoordinate(x, y);
            if (j == 0) {
                last_arr_time = start_time + mv.ftt;
                last_leave_time = start_time + mv.ftt + mv.swt;
                // printf("[INFO] last arr time: %d, last leave time: %d\n", last_arr_time, last_leave_time);
                break;
            } else if (j == (pm.mvs.size() - 1)) {
                start_time = curr_time - mv.ftt;
                tmp_entry.arrival_time = start_time + mv.ftt;
                tmp_entry.leaving_time_tail = last_leave_time;
            } else {
                tmp_entry.arrival_time = start_time + mv.ftt;
                tmp_entry.leaving_time_tail = start_time + mv.ftt + mv.swt;
            }
            // printf("[INFO] At location: %d, push back entry at %d, %d from %f to %f\n", tmp_entry.location, 
            //             instance_ptr->getRowCoordinate(tmp_entry.location),
            //             instance_ptr->getColCoordinate(tmp_entry.location), 
            //             tmp_entry.arrival_time, 
            //             tmp_entry.leaving_time_tail);
            tmp_entry.arrival_time = tmp_entry.arrival_time/F;
            tmp_entry.leaving_time_tail = tmp_entry.leaving_time_tail/F;
            // printf("[INFO] Result path at %d, from %f to %f\n", tmp_entry.location, tmp_entry.arrival_time, tmp_entry.leaving_time_tail);
            result_path.push_back(tmp_entry);
        }
    }

    std::reverse(result_path.begin(), result_path.end());
    instance_ptr->agents[curr_agent.id].total_cost = result_path.back().leaving_time_tail;
    result_path.back().leaving_time_tail = INF;
    for (auto res_entry: result_path) {
        printf("[INFO] The entry in result path %d, from %f to %f\n", res_entry.location, res_entry.arrival_time, 
                res_entry.leaving_time_tail);
    }
    cout<<"--------"<<endl;
    printf("The length of the path: %ld\n", result_path.size());
    return result_path;
}

SIPP_IP::SIPP_IP(std::shared_ptr<Instance> instance) {
    fillActions();
    instance_ptr = instance;
    agents = instance->agents;
    H = instance->getRows();
    W = instance->getCols();
    // auto ans = run(stx, sty, sto, 0, 0);
}

