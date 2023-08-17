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
	// for(int i = 0; i < pl.size(); i++){
	// 	init_drift.push_back(0);
	// 	control_points.push_back(std::vector<double> {});
	// }
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
		// std::cout << "All control points: ";
		for (double tmp_point : bezier_solution[i].control_points){
			// std::cout << tmp_point<< ", ";
			outfile << tmp_point << ", ";
		}
		outfile << "; ";
		for (auto tmp_node : plan[i]) {
			outfile << tmp_node.location << ", ";
		}
		// std::cout << std::endl;
		outfile << '\n';
	}
	outfile.close();	
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
        
	// printf("add v to list: %d,\t", v);
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

/*
void PTNode::getRT(ReservationTable &rt, int index){
	for (int i = 0; i < plan.size(); ++i){
		if(priority[i].find(index) != priority[i].end()){
			for(auto it = plan[i].begin(); it != plan[i].end(); ++it){
				TimeInterval newTI;
				newTI.t_max = it->leaving_time_tail;
				newTI.t_min = it->arrival_time;
				newTI.agent_id = i;
				rt[it->vertex].push_back(newTI);
			}
			getRT(rt, i);
		}
	}
}*/

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

// void PTNode::getRTP(std::set<int> &p, int index){
// 	std::set<int> tmp_list = priority[index];
// 	for (int item : tmp_list){
// 		if (p.find(item) == p.end()) {
// 			p.insert(item);
// 			getRTP(p, item);
// 		}
// 	}

// 	// for (int i = 0; i < (signed) plan.size(); ++i){
// 	// 	//not already in the list
// 	// 	if(p.find(i) == p.end() && (priority[i].find(index) != priority[i].end())){
// 	// 		p.insert(i);
// 	// 		getRTP(p, i);
// 	// 	}
// 	// }
// }

/**
 * 
*/
void PTNode::getRTFromP(Instance& instance , ReservationTable& rt, std::set<int> p){
	// std::cout<<"running getRTFromP\n";
	for(auto it = p.begin(); it != p.end(); ++it){
		for(auto it2 = plan[*it].begin(); it2 != plan[*it].end(); ++it2){
			//plan: vector<vector<pathentry>>
			TimeInterval newTI;
			
			// std::vector<int> CPs = instance.getConflictPoints(it2->conflict_point);
			newTI.t_max = it2->leaving_time_tail;
			newTI.t_min = it2->arrival_time;
			newTI.agent_id = *it;
			if (DEBUG_PTNODE) {
				printf("Add entry for agent %d at location %d, interval %f to %f\n", 
				*it, it2->location, it2->arrival_time, it2->leaving_time_tail);
			}
			rt[it2->location].push_back(newTI);
			//std::cout << it2->conflict_point << " conflict with ";
			// for(auto it3 = CPs.begin(); it3 != CPs.end(); ++it3){
			// 	rt[*it3].push_back(newTI);
			// }
		}
	}
	// std::cout<<"Finish getRTFromP\n";
}

/**
 * 
*/
void PTNode::getWindowRTFromP(Instance& instance , ReservationTable& rt, std::set<int>& p){
	InsertPreviousPath(instance, rt);
	for(auto it = p.begin(); it != p.end(); ++it){
		for(auto it2 = plan[*it].begin(); it2 != plan[*it].end(); ++it2){
			if (it2->arrival_time < WINDOWS_SIZE) {
				//plan: vector<vector<pathentry>>
				TimeInterval newTI;
				
				// std::vector<int> CPs = instance.getConflictPoints(it2->conflict_point);
				newTI.t_max = min(it2->leaving_time_tail, WINDOWS_SIZE);
				newTI.t_min = it2->arrival_time;
				newTI.agent_id = *it;
				if (DEBUG_PTNODE) {
					printf("Add entry for agent %d at location %d, interval %f to %f\n", 
					*it, it2->location, it2->arrival_time, it2->leaving_time_tail);
				}
				rt[it2->location].push_back(newTI);
			}
			//std::cout << it2->conflict_point << " conflict with ";
			// for(auto it3 = CPs.begin(); it3 != CPs.end(); ++it3){
			// 	rt[*it3].push_back(newTI);
			// }
		}
	}
	// std::cout<<"Finish getRTFromP\n";
}

void PTNode::InsertPreviousPath(Instance& instance, ReservationTable& rt)
{
	for (Agent tmp_agent : instance.agents) {
		for (PathEntry tmp_entry : tmp_agent.previous_path) {
			if (DEBUG_PTNODE) {
				printf("Add entry for agent %d at location %d, interval %f to %f\n", 
				tmp_agent.id, tmp_entry.location, tmp_entry.arrival_time, tmp_entry.leaving_time_tail);
			}
			TimeInterval newTI;
			// std::vector<int> CPs = instance.getConflictPoints(it2->conflict_point);
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
		// std::vector<int> CPs = instance.getConflictPoints(it2->conflict_point);
		newTI.t_max = tmp_agent.earliest_start_time + 2.5;
		newTI.t_min = tmp_agent.earliest_start_time;
		newTI.agent_id = tmp_agent.id;
		rt[tmp_agent.start_location].push_back(newTI);
	}
}

// void PTNode::getRTFromP(Instance& instance, 
// 	ReservationTable& rt, 
// 	// std::set<int> p, 
// 	int index, 
// 	std::map<int, std::map<int,std::vector<int> > >& ta)
// {
// 	vector<int> same = ta[instance.agents[index].trajectory[0]]
// 							[instance.agents[index].trajectory[instance.agents[index].trajectory.size()-1]];
// 	//TODO: change type
//     for(int i = 0; i < (int) same.size(); ++i){
//         if(same[i] != index){
//             if (instance.getEarliestStartTime(same[i]) < instance.getEarliestStartTime(index)){
//                 // std::cout <<"agent: " << index << "- pred: " << same[i] << "   ===   \n";
// 				//TODO: change type
//                 for(int i2 = 0; i2 < (int) plan[same[i]].size(); ++i2){
//                     TimeInterval newTI;
//                     std::vector<int> CPs = instance.getConflictPoints(plan[same[i]][i2].conflict_point);
//                     newTI.t_max = plan[same[i]][i2].leaving_time_tail;
//                     newTI.t_min = 0;
//                     newTI.agent_id = same[i];
//                     rt[plan[same[i]][i2].conflict_point].push_back(newTI);
//                     //std::cout << it2->conflict_point << " conflict with ";
//                     // std::cout  << newTI.t_min << " " << newTI.t_max << " " << newTI.agent_id << "===";
//                     for(auto it3 = CPs.begin(); it3 != CPs.end(); ++it3)
//                         rt[*it3].push_back(newTI);

//                     // std::cout << plan[same[i]][i2].conflict_point << ":" << CPs[0] << "----";
//                 }
//                 // std::cout << "\n";
                
//             }

//         }
//     }


//     // get vehicles having same entry point only
//     int conflictPointID = instance.getConflictPoints(instance.agents[index].trajectory[0])[0];
//     for(auto it5 = ta.begin(); it5 != ta.end(); ++it5){
//         if (conflictPointID == it5->first){
//             // std::cout <<  conflictPointID << "--" << it5->first << std::endl;
//             for(auto it6 = it5->second.begin(); it6 != it5->second.end(); ++it6){
//                 for(auto it7 = it6->second.begin(); it7 != it6->second.end(); ++it7){
//                     if (instance.getEarliestStartTime(*it7) < instance.getEarliestStartTime(index)){
//                         // std::cout << "same entry point agent: " << *it7 << std::endl;

//                         // if vehicle is already having a plan
//                         if (plan[*it7].size() > 0){
//                             TimeInterval newTI;
//                             std::vector<int> CPs = instance.getConflictPoints(plan[*it7][0].conflict_point);
//                             newTI.t_max = plan[*it7][0].leaving_time_tail;
//                             newTI.t_min = 0;
//                             newTI.agent_id = *it7;
//                             rt[plan[*it7][0].conflict_point].push_back(newTI);
//                             //std::cout << it2->conflict_point << " conflict with ";
//                             // std::cout  << newTI.t_min << " " << newTI.t_max << " " << newTI.agent_id << "===";
//                             // std::cout << plan[*it7][0].conflict_point << ":" << CPs[0] << "----\n";
//                             for(auto it8 = CPs.begin(); it8 != CPs.end(); ++it8)
//                                 rt[*it8].push_back(newTI);
                            
//                         }
//                         else{
//                             TimeInterval newTI;
//                             std::vector<int> CPs = instance.getConflictPoints(instance.getAgents()[*it7].start_location);
//                             newTI.t_max = instance.getAgents()[*it7].earliest_start_time;
//                             newTI.t_min = 0;
//                             newTI.agent_id = *it7;
//                             //std::cout << it2->conflict_point << " conflict with ";
//                             // std::cout  << newTI.t_min << " " << newTI.t_max << " " << newTI.agent_id << "===";
//                             // std::cout << instance.getAgents()[*it7].start_location << ":" << CPs[0] << "----\n";
//                             for(auto it8 = CPs.begin(); it8 != CPs.end(); ++it8)
//                                 rt[*it8].push_back(newTI);

//                         }
//                     }
//                 }
//                 // std::cout << "\n";
//             }
            
//         }
//     }
    

// 	// for(int i = 1; i < same.size(); ++i){
//     //     // std::cout << "same: " << index << "  " << same[i] << "  " << same[i-1] << "\n";
// 	// 	if(same[i] == index){
//     //         pred = same[i - 1];
            
//     //     } 
//     // }


// 	// if(pred != -1){
//     //     std::cout <<"agent: " << index << "- pred: " << pred << "   ===   \n";
// 	// 	for(int i = 0; i < plan[pred].size(); ++i){
// 	// 		TimeInterval newTI;
// 	// 		std::vector<int> CPs = instance.getConflictPoints(plan[pred][i].conflict_point);
// 	// 		newTI.t_max = plan[pred][i].leaving_time_tail;
// 	// 		newTI.t_min = 0;
// 	// 		newTI.agent_id = pred;
// 	// 		rt[plan[pred][i].conflict_point].push_back(newTI);
// 	// 		//std::cout << it2->conflict_point << " conflict with ";
//     //         std::cout  << newTI.t_min << " " << newTI.t_max << " " << newTI.agent_id << "===";
// 	// 		for(auto it3 = CPs.begin(); it3 != CPs.end(); ++it3)
// 	// 			rt[*it3].push_back(newTI);

//     //         std::cout << plan[pred][i].conflict_point << ":" << CPs[0] << "----";
// 	// 	}
//     //     std::cout << "\n";
// 	// }
	


// 	//std::cout<<"running getRTFromP\n";
//     // Vehicle could not arrive before the Earliest arrival time
// 	TimeInterval beginTI;
// 	beginTI.t_min = 0;
// 	beginTI.t_max = instance.getEarliestStartTime(index);
// 	beginTI.agent_id = index;
// 	rt[instance.getStartCP(index)].push_back(beginTI);
// 	std::vector<int> startCPs = instance.getConflictPoints(instance.getStartCP(index));
// 	for(auto it = startCPs.begin(); it != startCPs.end(); ++it)
// 		rt[*it].push_back(beginTI);


//     // std::cout << "RTP: \n" ;
// 	for(auto it = p.begin(); it != p.end(); ++it){
//         // std::cout << *it << ": ";
// 		for(auto it2 = plan[*it].begin(); it2 != plan[*it].end(); ++it2){
// 			//plan: vector<vector<pathentry>>
// 				TimeInterval newTI;
// 				std::vector<int> CPs = instance.getConflictPoints(it2->conflict_point);
// 				newTI.t_max = it2->leaving_time_tail;
// 				newTI.t_min = it2->arrival_time;
// 				newTI.agent_id = *it;
// 				rt[it2->conflict_point].push_back(newTI);
//                 // std::cout << newTI.t_min << " " << newTI.t_max << " " << newTI.agent_id << ", ===";
// 				//std::cout << it2->conflict_point << " conflict with ";
// 				for(auto it3 = CPs.begin(); it3 != CPs.end(); ++it3)
// 					rt[*it3].push_back(newTI);
                
//                 // std::cout << "    " << *it << ":" << it2->conflict_point << ":" << CPs[0] << "----";
// 		}
//         // std::cout << "\n";

        
// 	}
//     // std::cout << "\n\n";
// 	//std::cout<<"rtsize: " << rt.size() << "\n";
// }
