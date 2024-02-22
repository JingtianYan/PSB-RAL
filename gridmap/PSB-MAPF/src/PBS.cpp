/**
 * Implementation of the PBS class
 * 
 * TODO: add more info
*/
#include "../include/PBS.h"
#include "Instance.h"
#include "PTNode.h"
#include "SIPP.h"
#include "common.h"
#include "milp_cache.h"
#include <cmath>
#include <cstdio>
#include <iostream>
#include <memory>
#include <set>
#include <vector>

/**
 * Class constructor for PBS
 * 
 * @param instance the pointer of instance that is derived from the input files
*/
PBS::PBS(std::shared_ptr<Instance> user_instance_ptr, std::string single_agent_solver_name, double cutoff_time)
{
	instance_ptr = user_instance_ptr;
	if (single_agent_solver_name == "bezier") {
		sipp_ptr = std::make_shared<SIPP>(user_instance_ptr);
	} else {
		std::cerr << "Invalid single agent solver!\n";
		exit(-1);
	}
	single_agent_solver = single_agent_solver_name;
	cutoff_runtime = cutoff_time;
    num_of_agents = (int) user_instance_ptr->agents.size();
    global_start_t = clock();
}


void Showlist(std::list<int>& tmp_list)
{
	std::cout << "Show Priority: ";
	for (auto tmp_item: tmp_list){
		std::cout << tmp_item << ", ";
	}
	std::cout << std::endl;
}

void InitBezier(BezierInfo& bezier, int agent_num)
{
	BezierNode empty_node;
	empty_node.optimal_T = 0.0;
	empty_node.start_time = 0.0;
	empty_node.control_points = std::vector<double> {};
	for(int i = 0; i < agent_num; i++){
		bezier.push_back(empty_node);
	};

}

bool PBS::SolveSingleAgentBezier(PTNode& node, std::set<int>& rtp, int agent_id)
{
	DEBUG(printf("single agent slover for agent: %d\n", agent_id);)
	ReservationTable rt(instance_ptr->GetNumOfVertices());
	node.getRTP(rtp, agent_id);
	node.getWindowRTFromP(*instance_ptr, rt, rtp);
	DEBUG(printf("Get window RT\n");)
	node.InsertInitLocation(agent_id, *instance_ptr, rt);
	Path path;
	bool sipp_success = sipp_ptr->run(agent_id, rt, node.bezier_solution[agent_id], path, cutoff_runtime/3);
	if(!sipp_success) {
		return false;
	}
	node.plan[agent_id].clear();
	node.plan[agent_id] = path;
	ReservationTable rtdebug(instance_ptr->GetNumOfVertices());
	node.getWindowRTFromP(*instance_ptr, rtdebug, rtp);
	if(!checkValid(rtdebug, path, agent_id)) {
		std::fstream outputRT("ReservationTable.txt", std::fstream::in |
					std::fstream::out |
					std::fstream::trunc |
					std::fstream::binary );		
		for(auto itt = rt.begin(); itt != rt.end(); ++itt){
			for(auto itt2 = itt->begin(); itt2 != itt->end(); ++itt2){
				// std::cout<<"writting!\n";
				outputRT.write((char*)&itt2->t_min, sizeof(itt2->t_min));
				outputRT << ',';
				outputRT.write((char*)&itt2->t_max, sizeof(itt2->t_max));
				outputRT << ',' << itt2->agent_id <<';';
			}
			outputRT<<'\n';
		}

		outputRT.close();
		std::cout << "replanned agent " << agent_id << '\n';
		std::cout << "rtp: ";
		for(auto itt = rtp.begin(); itt != rtp.end(); ++ itt){
			std::cout << *itt <<" ";
		}
		printValidRT(rt);
		std::cout << "agent " << agent_id <<'\n';
		printPath(path);
		exit(-1);
	}
	return true;
}

bool PBS::CheckCollision(PTNode& node, int agent_first, int agent_second)
{
	for(auto first_agent_node = node.plan[agent_first].begin(); first_agent_node != node.plan[agent_first].end(); ++first_agent_node){
		for(auto second_agent_node = node.plan[agent_second].begin(); second_agent_node != node.plan[agent_second].end(); ++second_agent_node) {
			if( first_agent_node->location == second_agent_node->location){
				if(!(first_agent_node->leaving_time_tail - second_agent_node->arrival_time < EPSILON) &&
					 !(second_agent_node->leaving_time_tail - first_agent_node->arrival_time < EPSILON)) {
					return true;
				}
			}		
		}
	}
	return false;
}

bool PBS::GetNextRoundInit()
{
	for (unsigned int i = 0; i < instance_ptr->agents.size(); i++) {
		DEBUG(printf("[INFO] Retrive solution for agent %d!\n", i);)
		BezierNode curr_bezier = solution_node->bezier_solution[i];
		double solution_t = curr_bezier.optimal_T;
		if (solution_t < REPLAN_SIZE) {
			// printf("[INFO] Solution for Agent %d is get!\n", i);
			if (!instance_ptr->agents[i].is_solved) {
				instance_ptr->agents[i].total_cost += curr_bezier.optimal_T;
				instance_ptr->finishAgent(instance_ptr->agents[i].goal_location);
			}
			instance_ptr->agents[i].is_solved = true;
			instance_ptr->agents[i].start_location = instance_ptr->agents[i].goal_location;
			instance_ptr->agents[i].earliest_start_time = 0;
			instance_ptr->agents[i].previous_path.clear();
		} else {
			BernsteinPolynomial tmp_bern(CONTROL_POINTS_NUM, solution_t);
			double travel_dist = tmp_bern.GetVal(REPLAN_SIZE-instance_ptr->agents[i].earliest_start_time, curr_bezier.control_points);
			
			double prev_node_dist = floor(travel_dist);
  			int prev_start_loc = solution_node->plan[i][(int) prev_node_dist].location;
			double next_node_dist = ceil(travel_dist);
  			int next_start_loc = solution_node->plan[i][(int) next_node_dist].location;

			double tmp_travel_dist = travel_dist;
			double time_vist_next_node = REPLAN_SIZE;
			double arr_next_dist = CELL_DIS/2 - instance_ptr->agents[i].length + prev_node_dist;
			double leave_prev_dist = CELL_DIS/2 + instance_ptr->agents[i].length + prev_node_dist;
			// need to insert two nodes
			double time_leave_prev = -1.0;

			if (tmp_travel_dist < arr_next_dist) {
				instance_ptr->agents[i].previous_path.clear();
				next_start_loc = prev_start_loc;
			} else if (tmp_travel_dist < CELL_DIS/2 + instance_ptr->agents[i].length) {
				while (tmp_travel_dist < next_node_dist) {
					time_vist_next_node += TIME_STEP_SIZE;
					tmp_travel_dist = tmp_bern.GetVal(time_vist_next_node-instance_ptr->agents[i].earliest_start_time, curr_bezier.control_points);
					if (tmp_travel_dist > leave_prev_dist and time_leave_prev < 0) {
						time_leave_prev = time_vist_next_node;
					}
				}
				PathEntry prev_node;
				prev_node.arrival_time = 0;
				prev_node.leaving_time_tail = time_leave_prev - REPLAN_SIZE;
				prev_node.location = prev_start_loc;
				PathEntry next_node;
				next_node.arrival_time = 0;
				next_node.leaving_time_tail = time_vist_next_node - REPLAN_SIZE;
				next_node.location = next_start_loc;
				instance_ptr->agents[i].previous_path.clear();
				instance_ptr->agents[i].previous_path.push_back(prev_node);
				instance_ptr->agents[i].previous_path.push_back(next_node);
			} else{
				while (tmp_travel_dist < next_node_dist) {
					time_vist_next_node += TIME_STEP_SIZE;
					tmp_travel_dist = tmp_bern.GetVal(time_vist_next_node-instance_ptr->agents[i].earliest_start_time, curr_bezier.control_points);
				}
				PathEntry next_node;
				next_node.arrival_time = 0;
				next_node.leaving_time_tail = time_vist_next_node - REPLAN_SIZE;
				next_node.location = next_start_loc;
				instance_ptr->agents[i].previous_path.clear();
				instance_ptr->agents[i].previous_path.push_back(next_node);
			}
			double speed = (tmp_travel_dist - tmp_bern.GetVal(time_vist_next_node-instance_ptr->agents[i].earliest_start_time - TIME_STEP_SIZE, 
					curr_bezier.control_points))/TIME_STEP_SIZE;
			speed = round(speed*10)/10;
			instance_ptr->agents[i].init_velocity = max(instance_ptr->agents[i].v_min+EPS, min(instance_ptr->agents[i].v_max, speed));
			instance_ptr->agents[i].start_location = next_start_loc;
			double earliest_start_t = time_vist_next_node - REPLAN_SIZE;
			instance_ptr->agents[i].earliest_start_time = earliest_start_t;
			instance_ptr->agents[i].total_cost += REPLAN_SIZE;
		}
	}
	return true;
}

inline bool PBS::isTerminate(std::shared_ptr<PTNode> curr_n)
{
    if (curr_n->conflicts.empty()){
        solution_found = true;
        if (!curr_n->checkSolution(*instance_ptr)) {
            exit(-1);
        } else {
//            printf("success\n");
            ;
        }
        return true;
    } else {
        return false;
    }
}

std::shared_ptr<PTNode> PBS::selectNode()
{
    std::shared_ptr<PTNode> curr = open_list.top();
    open_list.pop();
    priority_graph.assign(num_of_agents, vector<bool>(num_of_agents, false));
    for (auto tmp_n = curr; tmp_n != nullptr; tmp_n = tmp_n->parent)
    {
        if (tmp_n->parent != nullptr) // non-root node
            priority_graph[tmp_n->constraint.low][tmp_n->constraint.high] = true;
    }
    num_HL_expanded++;
    return curr;
}

shared_ptr<Conflict> PBS::chooseConflict(const PTNode &node)
{
    if (node.conflicts.empty())
        return nullptr;
    return node.conflicts.back();
}

inline void PBS::pushNode(const std::shared_ptr<PTNode>& node)
{
    // update handles
    open_list.push(node);
}

void PBS::pushNodes(const std::shared_ptr<PTNode>& n1, const std::shared_ptr<PTNode>& n2)
{
    if (n1 != nullptr and n2 != nullptr)
    {
        if (n1->cost < n2->cost)
        {
            pushNode(n2);
            pushNode(n1);
        }
        else
        {
            pushNode(n1);
            pushNode(n2);
        }
    }
    else if (n1 != nullptr)
    {
        pushNode(n1);
    }
    else if (n2 != nullptr)
    {
        pushNode(n2);
    }
}

void PBS::topologicalSort(list<int>& stack)
{
    stack.clear();
    vector<bool> visited(num_of_agents, false);

    // Call the recursive helper function to store Topological
    // Sort starting from all vertices one by one
    for (int i = 0; i < num_of_agents; i++)
    {
        if (!visited[i])
            topologicalSortUtil(i, visited, stack);
    }
}

void PBS::topologicalSortUtil(int v, vector<bool> & visited, list<int> & stack)
{
    // Mark the current node as visited.
    visited[v] = true;

    // Recur for all the vertices adjacent to this vertex
    assert(!priority_graph.empty());
    for (int i = 0; i < num_of_agents; i++)
    {
        if (priority_graph[v][i] and !visited[i])
            topologicalSortUtil(i, visited, stack);
    }
    // Push current vertex to stack which stores result
    stack.push_back(v);
}

void PBS::getHigherPriorityAgents(const list<int>::reverse_iterator & p1, set<int>& higher_agents)
{
    for (auto p2 = std::next(p1); p2 != ordered_agents.rend(); ++p2)
    {
        if (priority_graph[*p1][*p2])
        {
            auto ret = higher_agents.insert(*p2);
            if (ret.second) // insert successfully
            {
                getHigherPriorityAgents(p2, higher_agents);
            }
        }
    }
}

void PBS::getLowerPriorityAgents(const list<int>::iterator & p1, set<int>& lower_subplans)
{
    for (auto p2 = std::next(p1); p2 != ordered_agents.end(); ++p2)
    {
        if (priority_graph[*p2][*p1])
        {
            auto ret = lower_subplans.insert(*p2);
            if (ret.second) // insert successfully
            {
                getLowerPriorityAgents(p2, lower_subplans);
            }
        }
    }
}

/**
 * @brief return true if agent low is lower than agent high
 *
 * */
bool PBS::hasHigherPriority(int low, int high) const
{
    std::queue<int> Q;
    vector<bool> visited(num_of_agents, false);
    visited[low] = false;
    Q.push(low);
    while(!Q.empty())
    {
        auto n = Q.front();
        Q.pop();
        if (n == high)
            return true;
        for (int i = 0; i < num_of_agents; i++)
        {
            if (priority_graph[n][i] and !visited[i])
                Q.push(i);
        }
    }
    return false;
}

bool PBS::generateChild(int child_id, std::shared_ptr<PTNode> parent, int low, int high)
{
    assert(child_id == 0 or child_id == 1);
    std::shared_ptr<PTNode> node = std::make_shared<PTNode>(parent);
    node->parent = parent;
    node->curr_conflict = std::make_shared<Conflict>(low, high);
    if (child_id == 0) parent->children.first = node;
    else parent->children.second = node;

    node->constraint.set(low, high);
    priority_graph[high][low] = false;
    priority_graph[low][high] = true;
    if (screen > 2)
        printPriorityGraph();
    topologicalSort(ordered_agents);
    if (screen > 2)
    {
        cout << "Ordered agents: ";
        for (int i : ordered_agents)
            cout << i << ",";
        cout << endl;
    }
    vector<int> topological_orders(num_of_agents); // map agent i to its position in ordered_agents
    auto i = num_of_agents - 1;
    for (const auto & a : ordered_agents)
    {
        topological_orders[a] = i;
        i--;
    }

    std::priority_queue<pair<int, int>> to_replan; // <position in ordered_agents, agent id>
    vector<bool> lookup_table(num_of_agents, false);
    to_replan.emplace(topological_orders[low], low);
    lookup_table[low] = true;
    { // find conflicts where one agent is higher than high and the other agent is lower than low
        set<int> higher_agents;
        auto p = ordered_agents.rbegin();
        std::advance(p, topological_orders[high]);
        assert(*p == high);
        getHigherPriorityAgents(p, higher_agents);
        higher_agents.insert(high);

        set<int> lower_agents;
        auto p2 = ordered_agents.begin();
        std::advance(p2, num_of_agents - 1 - topological_orders[low]);
        assert(*p2 == low);
        getLowerPriorityAgents(p2, lower_agents);

        for (const auto & conflict : node->conflicts)
        {
            int a1 = conflict->a1;
            int a2 = conflict->a2;
            if (a1 == low or a2 == low)
                continue;
            if (topological_orders[a1] > topological_orders[a2])
            {
                std::swap(a1, a2);
            }
            if (!lookup_table[a1] and lower_agents.find(a1) != lower_agents.end() and higher_agents.find(a2) != higher_agents.end())
            {
                to_replan.emplace(topological_orders[a1], a1);
                lookup_table[a1] = true;
            }
        }
    }

    while(!to_replan.empty())
    {
        int a, rank;
        tie(rank, a) = to_replan.top();
        to_replan.pop();
        lookup_table[a] = false;
        if (screen > 2) cout << "Replan agent " << a << endl;
        // Re-plan path
        set<int> higher_agents;
        auto p = ordered_agents.rbegin();
        std::advance(p, rank);
        assert(*p == a);
        getHigherPriorityAgents(p, higher_agents);
        assert(!higher_agents.empty());
        if (screen > 2)
        {
            cout << "Higher agents: ";
            for (auto i : higher_agents)
                cout << i << ",";
            cout << endl;
        }
        if(!SolveSingleAgentBezier(*node, higher_agents, a))
        {
            if (child_id == 0) parent->children.first = nullptr;
            else parent->children.second = nullptr;
            return false;
        }

        // Delete old conflicts
        for (auto c = node->conflicts.begin(); c != node->conflicts.end();)
        {
            if ((*c)->a1 == a or (*c)->a2 == a)
                c = node->conflicts.erase(c);
            else
                ++c;
        }

        // Update conflicts and to_replan
        set<int> lower_agents;
        auto p2 = ordered_agents.begin();
        std::advance(p2, num_of_agents - 1 - rank);
        assert(*p2 == a);
        getLowerPriorityAgents(p2, lower_agents);
        if (screen > 2 and !lower_agents.empty())
        {
            cout << "Lower agents: ";
            for (auto i : lower_agents)
                cout << i << ",";
            cout << endl;
        }

        // Find new conflicts
        for (auto a2 = 0; a2 < num_of_agents; a2++)
        {
            if (a2 == a or lookup_table[a2] or higher_agents.count(a2) > 0) // already in to_replan or has higher priority
                continue;
            auto t = clock();
            if (node->CheckCollision(a, a2))
            {
                node->conflicts.emplace_back(new Conflict(a, a2));
                if (lower_agents.count(a2) > 0) // has a collision with a lower priority agent
                {
                    if (screen > 1)
                        cout << "\t" << a2 << " needs to be replanned due to collisions with " << a << endl;
                    to_replan.emplace(topological_orders[a2], a2);
                    lookup_table[a2] = true;
                }
            }
            runtime_detect_conflicts += (double)(clock() - t) / CLOCKS_PER_SEC;
        }
    }
    num_HL_generated++;
    return true;
}

void PBS::printPriorityGraph() const
{
    cout << "Priority graph:";
    for (int a1 = 0; a1 < num_of_agents; a1++)
    {
        for (int a2 = 0; a2 < num_of_agents; a2++)
        {
            if (priority_graph[a1][a2])
                cout << a1 << "<" << a2 << ",";
        }
    }
    cout << endl;
}

void PBS::recursivePrint(const std::shared_ptr<PTNode>& curr_node)
{
    std::shared_ptr<PTNode> tmp_node = curr_node;
    cout << "[INFO] ";
    while (tmp_node->parent != nullptr) {
        cout << "Conflict: " << *(tmp_node->curr_conflict) << "; ";
        tmp_node = tmp_node->parent;
    }
    cout << endl;
}

bool PBS::initRootNode(std::shared_ptr<PTNode>& root_node)
{
    vector<Path> plan(num_of_agents);
    // int: the id of agent, std::set<int>: the agents with higher priority
    std::map<int, std::set<int>> priority;
    for (int i = 0; i < num_of_agents; i++) {
        priority[i] = std::set<int>();
    }

    // TODO: Modify this part for the output of the result
    BezierInfo init_bezier(num_of_agents);

    root_node = std::make_shared<PTNode>(plan, init_bezier, priority);
    for(int i = 0; i < num_of_agents; ++i){
        if (instance_ptr->agents[i].is_solved) {
            continue;
        }
        std::set<int> empty_set;
        if(!SolveSingleAgentBezier(*root_node, empty_set, i)) {
            return false;
        }
    }
    root_node->calculateCost(instance_ptr);
    root_cost = root_node->cost;
    root_node->parent = nullptr;
    for (int a1 = 0; a1 < num_of_agents; a1++)
    {
        for (int a2 = a1 + 1; a2 < num_of_agents; a2++)
        {
            if(root_node->CheckCollision(a1, a2))
            {
                root_node->conflicts.emplace_back(new Conflict(a1, a2));
            }
        }
    }
    root_cost = std::accumulate(root_node->solution_cost.begin(), root_node->solution_cost.end(), 0.0);
    return true;
}

bool PBS::solve(const string& outputFileName, bool& all_agent_solved)
{
    if (screen > 0) // 1 or 2
    {
        string name = getSolverName();
        name.resize(35, ' ');
        cout << name << ": ";
    }
    // set timer
    auto start = clock();

    std::shared_ptr<PTNode> Root;
    if(not initRootNode(Root)) {
        printf("[Error] Fail to find a initial plan!\n");
        return false;
    }
    open_list.push(Root);
    while (!open_list.empty() and ((double)(clock() - start) / CLOCKS_PER_SEC) < this->cutoff_runtime)
    {
        auto curr = selectNode();
        if (this->isTerminate(curr)) {
            solution_node = curr;
            GetNextRoundInit();
            break;
        }

        curr->conflict = chooseConflict(*curr);

        if (screen > 0) {
            recursivePrint(curr);
            cout << "	Expand " << curr->depth << "	on " << *(curr->conflict) << endl;
        }
        auto t1 = clock();
        generateChild(0, curr, curr->conflict->a1, curr->conflict->a2);
        generateChild(1, curr, curr->conflict->a2, curr->conflict->a1);
        runtime_generate_child += (double)(clock() - t1) / CLOCKS_PER_SEC;
        pushNodes(curr->children.first, curr->children.second);
    }  // end of while loop
    runtime = (double)(clock() - global_start_t) / CLOCKS_PER_SEC;
    for (auto& tmp_agent: instance_ptr->agents) {
        if (not tmp_agent.is_solved) {
            all_agent_solved = false;
        }
    }
    return solution_found;
}


void PBS::updateCost()
{
	solution_cost = 0;
	for (unsigned int i = 0; i < instance_ptr->agents.size(); i++) {
		solution_cost += instance_ptr->agents[i].total_cost;
	}
}

void PBS::saveResults(int all_agents_solved, const string &fileName, const string &instanceName) const
{
	std::ifstream infile(fileName);
	bool exist = infile.good();
	infile.close();
	if (!exist) {
		ofstream addHeads(fileName);
		addHeads << "runtime, is solved," <<
			"solution cost," <<
			"solver name,instance name,#agents" << endl;
		addHeads.close();
	}
	ofstream stats(fileName, std::ios::app);
	stats << runtime << "," << all_agents_solved << ", " <<
		solution_cost <<  "," <<
		getSolverName() << "," << instanceName << "," << instance_ptr->agents.size() << endl;
	stats.close();
}


void PBS::printPath(Path path){
	std::cout<<"\n\n_________________________________printing Path____________________________________\n";
	std::cout << "path size: " << path.size() <<"\n";
		for(auto itt = path.begin(); itt != path.end(); ++itt){
			std::cout << "cp" << itt->location << "\tarrival: "  << itt->arrival_time << "\tleave: " << itt->leaving_time_tail << "\n";
	}
	std::cout<<"__________________________________________________________________________________\n\n";
}

void PBS::printRT(ReservationTable rt){
	std::cout<<"\n\n_________________________________printing ReservationTable PBS_________________________\n";
	for(int i = 0; i < (signed) rt.size(); ++i){
		std::cout<<"cp" << i << "\t";
		for(auto ittemp = rt[i].begin(); ittemp != rt[i].end(); ++ittemp){
			std::cout<< ittemp->t_min << "\t" <<ittemp->t_max << "\t#" << ittemp->agent_id << "#\t";
		}
		std::cout<<"\n";
	}
	std::cout<<"_____________________________________________________________________________________\n\n";
}

void PBS::printValidRT(ReservationTable rt){
	std::cout<<"\n\n_________________________________printing ReservationTable_________________________\n";
	for(int i = 0; i < (signed) rt.size(); ++i){
		if (rt[i].empty()) {
			continue;
		}
		std::cout<<"cp" << i << "\t";
		for(auto ittemp = rt[i].begin(); ittemp != rt[i].end(); ++ittemp){
			std::cout<< ittemp->t_min << "\t" <<ittemp->t_max << "\t#" << ittemp->agent_id << "#\t";
		}
		std::cout<<"\n";
	}
	std::cout<<"_____________________________________________________________________________________\n\n";
}


bool PBS::checkValid(ReservationTable& rt, Path& path, int agent){
	for(auto it = path.begin(); it != path.end(); ++it){
		int cp = 0;
		for(auto it2 = rt.begin(); it2 != rt.end(); ++it2,++cp){
			if(it->location == cp){
				for(auto it3 = it2->begin(); it3 != it2->end(); ++it3){
					if(it3->agent_id == agent) continue;

					if(!(it->leaving_time_tail - it3->t_min < EPSILON) && !(it3->t_max - it->arrival_time < EPSILON)) {
						std::cout << "agent " << agent << ": " << it->arrival_time << ' ' << it->leaving_time_tail <<'\n' << "agent " << it3->agent_id << ": " << it3->t_min << " " << it3->t_max << '\n';
						return false;
					}
				}
			}
			
		}
	}
	return true;
}