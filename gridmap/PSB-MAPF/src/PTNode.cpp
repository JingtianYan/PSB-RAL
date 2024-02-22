#include "PTNode.h"
#include "Instance.h"
#include "common.h"
#include <memory>
#include <vector>

PTNode::PTNode(vector<Path> pl,
				BezierInfo& solution,
				std::map<int, std::set<int>> pr){
	plan = pl;
	priority = pr;
	bezier_solution = solution;
}

bool PTNode::checkSolution(Instance& instance){
    std::pair<int, int> result(-1, -1);
    ReservationTable rt_table(instance.GetNumOfVertices());
    std::set<int> s;
    for(int i = 0; i < (signed) plan.size();++i) s.insert(i);
//    getRTFromP(rt_table, s);
    getWindowRTFromP(instance, rt_table, s);
    for (size_t agent_id = 0; agent_id < plan.size(); agent_id++) {
        int conflict_id = checkValid(rt_table, agent_id);
        if (conflict_id != -1) {
            return false;
        }
    }
    return true;
}

int PTNode::checkValid(ReservationTable& rt, int agent_id){
    for (auto& path_entry: plan[agent_id]) {
        for (auto& rt_interval: rt[path_entry.location]) {
            if (rt_interval.agent_id == agent_id) continue;
            if(path_entry.leaving_time_tail - rt_interval.t_min >= EPSILON and
               rt_interval.t_max - path_entry.arrival_time >= EPSILON) {
                return rt_interval.agent_id;
            }
        }
    }
    return -1;
}


/***
 * Finish this after working the structure
 * TODO: 1. agent_x path optimal_T control_points
*/
void PTNode::writeToFile(Instance& instance, const string& file_name)
{
	std::ifstream infile(file_name);
	bool exist = infile.good();
	infile.close();
	if (!exist) {
		ofstream addHeads(file_name);
		addHeads << "Num of agents: " << instance.agents.size() << std::endl;
		addHeads << "Agent id; Start Point;Goal Point;Length; Optimal T; Start time; Control Points; Trajectory" << '\n';
		addHeads.close();
	}

	ofstream outfile(file_name, std::ios::app);
	for(int i = 0; i < (signed)plan.size(); ++i){
		outfile <<  i \
			<< ";" << instance.agents[i].start_location  \
			<< ";" << instance.agents[i].goal_location  \
			<< ";" << instance.agents[i].length \
			<< ";" << bezier_solution[i].optimal_T \
			<< ";" << instance.agents[i].earliest_start_time << ";";
		for (double tmp_point : bezier_solution[i].control_points){
			outfile << tmp_point << ", ";
		}
		outfile << "; ";
		for (auto tmp_node : plan[i]) {
			outfile << tmp_node.location << ", ";
		}
		outfile << '\n';
	}
	outfile.close();	
}

bool PTNode::CheckCollision(int agent_first, int agent_second)
{
    for(auto first_agent_node = plan[agent_first].begin(); first_agent_node != plan[agent_first].end(); ++first_agent_node){
        for (auto second_agent_node = plan[agent_second].begin(); second_agent_node != plan[agent_second].end(); ++second_agent_node) {
            if( first_agent_node->location == second_agent_node->location){
                if(first_agent_node->leaving_time_tail - second_agent_node->arrival_time >= EPSILON &&
                   second_agent_node->leaving_time_tail - first_agent_node->arrival_time >= EPSILON) {
                    return true;
                }
            }
        }
    }
    return false;
}

/***
 * Generate the priority list for all the agents
 * 
 * @param v The id of agent need to be sort
 * @param visited The list to indicate visited agents
 * @param List The already sorted list
*/
void PTNode::topologicalSortUtil(int v, bool visited[], std::list<int>& List)
{
    // Mark the current node as visited.
    visited[v] = true;
 
    // Recur for all the vertices
    // adjacent to this vertex

    for (auto it = priority[v].begin(); it != priority[v].end(); ++it) {
		if (!visited[*it]) {
            topologicalSortUtil(*it, visited, List);
		}
	}

    // Push current vertex to stack
    // which stores result priority
    List.push_back(v);
}

std::list<int> PTNode::topologicalSort(){
	std::list<int> List;
 
    // Mark all the vertices as not visited
    bool* visited = new bool[plan.size()];
    for (int i = 0; i < (signed) plan.size(); ++i)
        visited[i] = false;
 
    // Call the recursive helper function
    // to store Topological
    // Sort starting from all
    // vertices one by one
    for (int i = 0; i < (signed)plan.size(); ++i){
        if (visited[i] == false)
            topologicalSortUtil(i, visited, List);
    }
 	delete[] visited;
	return List;
}

std::list<int> PTNode::topologicalSort(int agent){
	std::list<int> List;
 
    // Mark all the vertices as not visited
    bool* visited = new bool[plan.size()];
    for (int i = 0; i < (signed) plan.size(); ++i)
        visited[i] = false;
 
    // Call the recursive helper function
    // to store Topological
    // Sort starting from all
    // vertices one by one

    topologicalSortUtil(agent, visited, List);

 	delete[] visited;
	return List;
}

/**
 * Calculate the time cost of the current plan
 * 
 * TODO: need to modify this function for online implementation
*/
void PTNode::calculateCost(){
	//std::cout << "calculate cost running\n";
	double c = 0;
	for(auto it = plan.begin(); it != plan.end(); ++it){
		if (!it->empty()){
			c += it->back().leaving_time_tail;
		}
	}
	cost = c;
}

void PTNode::calculateCost(std::shared_ptr<Instance> instance_ptr){
	//std::cout << "calculate cost running\n";
	double c = 0;
	for(unsigned int i = 0; i < instance_ptr->agents.size(); i++){
		if (!instance_ptr->agents[i].is_solved) {
			c += instance_ptr->agents[i].tmp_cost;
		}
	}
	cost = c;
}


void printRT(ReservationTable rt){
	std::cout<<"\n\n_________________________________printing ReservationTable_________________________\n";
	for(int i = 0; i < (signed) rt.size(); ++i){
		std::cout<<"cp" << i << "\t";
		for(auto ittemp = rt[i].begin(); ittemp != rt[i].end(); ++ittemp){
			std::cout<< ittemp->t_min << "\t" <<ittemp->t_max << "\t" << ittemp->agent_id << "\t";
		}
		std::cout<<"\n";
	}
	std::cout<<"_____________________________________________________________________________________\n\n";
}

void PTNode::printPlan(){
	std::cout<<"\n\n_________________________________printing Plans_________________________\n";
	for(int i = 0; i < (signed) plan.size(); ++i){
		std::cout<<"Agent " << i << ": \t";
		for(PathEntry tmp_path : plan[i]){
			printf("Loc: %d, From %f to %f\t", tmp_path.location, tmp_path.arrival_time, tmp_path.leaving_time_tail);
		}
		std::cout<<"\n";
	}
	std::cout<<"_____________________________________________________________________________________\n\n";
}

std::tuple<int, int, int> PTNode::getFirstWindowCollision(Instance& instance){
	std::tuple<int, int, int> result = std::make_tuple(-1, -1, -1);
	ReservationTable table(instance.GetNumOfVertices());
	std::set<int> s;
	for(int i = 0; i < (signed) plan.size();++i) s.insert(i);
	getWindowRTFromP(instance, table, s);
	// printPlan();
	//iterating path in plan vector<vector<PathEntry>>
	int aid = 0;
	for(auto it = plan.begin(); it != plan.end(); ++it, ++aid){
		//iterating points in path
		for(auto it2 = it->begin(); it2 != it->end(); ++it2){
			//it2 is path entry
			if (it2->arrival_time >= WINDOWS_SIZE - EPSILON) {
				continue;
			}
			std::list<TimeInterval> vertexTimeTable = table[it2->location];
			for(auto it3 = vertexTimeTable.begin(); it3 != vertexTimeTable.end(); ++it3){
				if(aid == it3->agent_id)
					continue;
				//if  collision
				if(!(min(it2->leaving_time_tail, WINDOWS_SIZE - EPSILON) - it3->t_min < EPSILON) and
				 !(it3->t_max - it2->arrival_time < EPSILON)){
					// printf("Collision between: %d and %d at location: %d, for agent %d, from: "
					// "%f to %f, for agent %d, from %f to %f\n", aid, it3->agent_id, it2->location,
					// aid, it2->arrival_time, it2->leaving_time_tail, 
					// it3->agent_id, it3->t_min, it3->t_max);
					result = std::make_tuple(aid, it3->agent_id, it2->location);
					return result;
				}
			}
		}
	}	
	return result;
}

std::tuple<int, int, int> PTNode::getFirstCollision(Instance& instance){
	std::tuple<int, int, int> result = std::make_tuple(-1, -1, -1);
	ReservationTable table(instance.GetNumOfVertices());
	std::set<int> s;
	for(int i = 0; i < (signed) plan.size();++i) s.insert(i);
	getRTFromP(instance, table, s);
	// getWindowRTFromP(instance, table, s);
	//iterating path in plan vector<vector<PathEntry>>
	int aid = 0;
	for(auto it = plan.begin(); it != plan.end(); ++it, ++aid){
		//iterating points in path
		for(auto it2 = it->begin(); it2 != it->end(); ++it2){
			//it2 is path entry
			std::list<TimeInterval> vertexTimeTable = table[it2->location];
			for(auto it3 = vertexTimeTable.begin(); it3 != vertexTimeTable.end(); ++it3){
				if(aid == it3->agent_id)
					continue;
				//if  collision
				if(!(it2->leaving_time_tail - it3->t_min < EPSILON) && !(it3->t_max - it2->arrival_time < EPSILON)){
					result = std::make_tuple(aid, it3->agent_id, it2->location);
					return result;
				}
			}
		}
	}	
	return result;
}

void PTNode::printPriorityMap()
{
	for (auto tmp_priority: priority) {
		printf("agent %d: ", tmp_priority.first);
		for (auto item : tmp_priority.second) {
			printf("agent %d\t", item);
		}
		std::cout << std::endl;
	}
}

/**
 * Set the reservation table for each index
 * 
 * @param index the index of the 
 * @return p, the set index
*/
void PTNode::getRTP(std::set<int> &p, int index){
	for (int i = 0; i < (signed) plan.size(); ++i){
		//not already in the list
		if(p.find(i) == p.end() && (priority[i].find(index) != priority[i].end())){
			p.insert(i);
			getRTP(p, i);
		}
	}
}

/**
 * @brief Get the reservation table given the agents priority
 *
 * @param p The agent with higher priority
 * @param[out] rt The reservation table
*/
void PTNode::getRTFromP(ReservationTable& rt, std::set<int>& p){
    for(int agent_id: p){
        insertPathToRT(rt, agent_id);
    }
}

void PTNode::insertPathToRT(ReservationTable& rt, int agent_id)
{
    for(auto it2 = plan[agent_id].begin(); it2 != plan[agent_id].end(); ++it2){
        TimeInterval newTI;
        newTI.t_max = it2->leaving_time_tail;
        newTI.t_min = it2->arrival_time;
        newTI.agent_id = agent_id;
        rt[it2->location].push_back(newTI);
    }
}

void PTNode::getRTFromP(Instance& instance , ReservationTable& rt, std::set<int> p){
	for(auto it = p.begin(); it != p.end(); ++it){
		for(auto it2 = plan[*it].begin(); it2 != plan[*it].end(); ++it2){
			TimeInterval newTI;
			newTI.t_max = it2->leaving_time_tail;
			newTI.t_min = it2->arrival_time;
			newTI.agent_id = *it;
			rt[it2->location].push_back(newTI);
		}
	}
}

void PTNode::getWindowRTFromP(Instance& instance , ReservationTable& rt, std::set<int>& p){
	InsertPreviousPath(instance, rt);
	for(auto it = p.begin(); it != p.end(); ++it){
		for(auto it2 = plan[*it].begin(); it2 != plan[*it].end(); ++it2){
			if (it2->arrival_time < WINDOWS_SIZE) {
				TimeInterval newTI;
                newTI.t_max = min(it2->leaving_time_tail, WINDOWS_SIZE);
				newTI.t_min = it2->arrival_time;
				newTI.agent_id = *it;
				rt[it2->location].push_back(newTI);
			}
		}
	}
}

void PTNode::InsertPreviousPath(Instance& instance, ReservationTable& rt)
{
	for (Agent tmp_agent : instance.agents) {
		if (tmp_agent.is_solved) {
			continue;
		}
		for (PathEntry tmp_entry : tmp_agent.previous_path) {
			TimeInterval newTI;
			newTI.t_max = tmp_entry.leaving_time_tail;
			newTI.t_min = tmp_entry.arrival_time;
			newTI.agent_id = tmp_agent.id;
			rt[tmp_entry.location].push_back(newTI);
			
		}
	}
}

void PTNode::InsertInitLocation(int agent_id, Instance& instance, ReservationTable& rt)
{
	for (Agent tmp_agent : instance.agents) {
		if (agent_id == tmp_agent.id or tmp_agent.is_solved){
			continue;
		}
		// avoid take the initial position
		TimeInterval newTI;
		newTI.t_max = tmp_agent.earliest_start_time + 2.5;
		newTI.t_min = tmp_agent.earliest_start_time;
		newTI.agent_id = tmp_agent.id;
		rt[tmp_agent.start_location].push_back(newTI);
	}
}
