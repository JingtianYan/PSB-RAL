#include "../include/PTNode.h"
#include <vector>

PTNode::PTNode(vector<Path> pl,
				std::vector<std::vector<double>> old_control_points, 
				std::vector<double> old_optimal_T,
				std::vector<double> old_start_time,
				std::map<int, std::set<int>> pr){
	plan = pl;
	priority = pr;
	control_points = old_control_points;
	optimal_T = old_optimal_T;
	start_time = old_start_time;
	// for(int i = 0; i < pl.size(); i++){
	// 	init_drift.push_back(0);
	// 	control_points.push_back(std::vector<double> {});
	// }
}

void PTNode::writeToFile(Instance& instance, const string& file_name)
{
	std::ofstream file;
	// file.open(file_name, std::ios::app);
	file.open(file_name);
	if(file.is_open()){
		file << "In Point;Out Point;Length;Earliest Arrival;In Time;Out Time Head; Out Time Tail;Time spend; Speed;Importance; Optimal T; Start Time; Control Points;" << '\n';

		for(int i = 0; i < (signed)plan.size(); ++i){
			double pairDistanceValue = (instance.getPairDistancesMap())[plan[i][0].conflict_point][plan[i][plan[i].size()-1].conflict_point];
			file <<  instance.getVIDToName()[plan[i][0].conflict_point]  \
				<< ";" <<  instance.getVIDToName()[plan[i][plan[i].size()-1].conflict_point]  \
				<< ";" <<  pairDistanceValue \
				<< ";" << instance.getEarliestStartTime(i) \
				<< ";" << plan[i][0].arrival_time \
				<< ";" << plan[i][plan[i].size()-1].arrival_time \
				<< ";" << plan[i][plan[i].size()-1].leaving_time_tail \
				<< ";" << plan[i][plan[i].size()-1].arrival_time - plan[i][0].arrival_time\
				<< ";" << pairDistanceValue/(plan[i][plan[i].size()-1].arrival_time - plan[i][0].arrival_time)\
                << ";" << instance.getAgents()[i].importance \
                << ";" << optimal_T[i] \
				<< ";" << start_time[i] << ";";
			// std::cout << "All control points: ";
			for (double tmp_point : control_points[i]){
				// std::cout << tmp_point;
				file << tmp_point << ", ";
			}
			// std::cout << std::endl;
			file << '\n';

		}
	}
	else std::cout << "unable to open file";
	file.close();
}

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
    // which stores resultpriority
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
		c += it->back().leaving_time_tail;
	}
	cost = c;
}

double PTNode::calculateDelay(std::shared_ptr<Instance> instance_ptr){
	//std::cout << "calculate cost running\n";
	double delay = 0;
	for (unsigned int i = 0; i < plan.size(); i++) {
		delay += (plan[i].back().leaving_time_tail - instance_ptr->agents[i].earliest_start_time);
	}
	// for(auto it = plan.begin(); it != plan.end(); ++it){
	// 	c += (it->back().leaving_time_tail - it->begin().);
	// }
	return delay;
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

std::tuple<int, int, int> PTNode::getFirstCollision(Instance& instance){
	std::tuple<int, int, int> result = std::make_tuple(-1, -1, -1);
	ReservationTable table(instance.getNumOfVertices());
	std::set<int> s;
	for(int i = 0; i < (signed) plan.size();++i) s.insert(i);
	getRTFromP(instance, table, s);
	//iterating path in plan vector<vector<PathEntry>>
	int aid = 0;
	for(auto it = plan.begin(); it != plan.end(); ++it, ++aid){
		//iterating points in path
		for(auto it2 = it->begin(); it2 != it->end(); ++it2){
			//it2 is path entry
			std::list<TimeInterval> vertexTimeTable = table[it2->conflict_point];
			for(auto it3 = vertexTimeTable.begin(); it3 != vertexTimeTable.end(); ++it3){
				if(aid == it3->agent_id)
					continue;
				//if  collision
				if(!(it2->leaving_time_tail - it3->t_min < EPSILON) && !(it3->t_max - it2->arrival_time < EPSILON)){
					result = std::make_tuple(aid, it3->agent_id, it2->conflict_point);
					return result;
				}
			}
		}
	}
	return result;
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

/**
 * 
*/
void PTNode::getRTFromP(Instance& instance, ReservationTable& rt, std::set<int> p){
	// std::cout<<"running getRTFromP\n";
	for(auto it = p.begin(); it != p.end(); ++it){
		for(auto it2 = plan[*it].begin(); it2 != plan[*it].end(); ++it2){
			//plan: vector<vector<pathentry>>
			TimeInterval newTI;
			
			std::vector<int> CPs = instance.getConflictPoints(it2->conflict_point);
			newTI.t_max = it2->leaving_time_tail;
			newTI.t_min = it2->arrival_time;
			newTI.agent_id = *it;
			rt[it2->conflict_point].push_back(newTI);
			//std::cout << it2->conflict_point << " conflict with ";
			for(auto it3 = CPs.begin(); it3 != CPs.end(); ++it3){
				rt[*it3].push_back(newTI);
			}
		}
	}
}

void PTNode::getRTFromP(Instance& instance,
	ReservationTable& rt, 
	std::set<int> p, 
	int index, 
	std::map<int, std::map<int,std::vector<int> > >& ta)
{
	vector<int> same = ta[instance.agents[index].trajectory[0]][instance.agents[index].trajectory[instance.agents[index].trajectory.size()-1]];

	//TODO: change type
    for(int i = 0; i < (int) same.size(); ++i){
        if(same[i] != index){
            if (instance.getEarliestStartTime(same[i]) < instance.getEarliestStartTime(index)){
                // std::cout <<"agent: " << index << "- pred: " << same[i] << "   ===   \n";
				//TODO: change type
                for(int i2 = 0; i2 < (int) plan[same[i]].size(); ++i2){
                    TimeInterval newTI;
                    std::vector<int> CPs = instance.getConflictPoints(plan[same[i]][i2].conflict_point);
                    newTI.t_max = plan[same[i]][i2].leaving_time_tail;
                    newTI.t_min = 0;
                    newTI.agent_id = same[i];
                    rt[plan[same[i]][i2].conflict_point].push_back(newTI);
                    //std::cout << it2->conflict_point << " conflict with ";
                    // std::cout  << newTI.t_min << " " << newTI.t_max << " " << newTI.agent_id << "===";
                    for(auto it3 = CPs.begin(); it3 != CPs.end(); ++it3)
                        rt[*it3].push_back(newTI);

                    // std::cout << plan[same[i]][i2].conflict_point << ":" << CPs[0] << "----";
                }
                // std::cout << "\n";
                
            }

        }
    }


    // get vehicles having same entry point only
    int conflictPointID = instance.getConflictPoints(instance.agents[index].trajectory[0])[0];
    for(auto it5 = ta.begin(); it5 != ta.end(); ++it5){
        if (conflictPointID == it5->first){
            // std::cout <<  conflictPointID << "--" << it5->first << std::endl;
            for(auto it6 = it5->second.begin(); it6 != it5->second.end(); ++it6){
                for(auto it7 = it6->second.begin(); it7 != it6->second.end(); ++it7){
                    if (instance.getEarliestStartTime(*it7) < instance.getEarliestStartTime(index)){
                        // std::cout << "same entry point agent: " << *it7 << std::endl;

                        // if vehicle is already having a plan
                        if (plan[*it7].size() > 0){
                            TimeInterval newTI;
                            std::vector<int> CPs = instance.getConflictPoints(plan[*it7][0].conflict_point);
                            newTI.t_max = plan[*it7][0].leaving_time_tail;
                            newTI.t_min = 0;
                            newTI.agent_id = *it7;
                            rt[plan[*it7][0].conflict_point].push_back(newTI);
                            //std::cout << it2->conflict_point << " conflict with ";
                            // std::cout  << newTI.t_min << " " << newTI.t_max << " " << newTI.agent_id << "===";
                            // std::cout << plan[*it7][0].conflict_point << ":" << CPs[0] << "----\n";
                            for(auto it8 = CPs.begin(); it8 != CPs.end(); ++it8)
                                rt[*it8].push_back(newTI);
                            
                        }
                        else{
                            TimeInterval newTI;
                            std::vector<int> CPs = instance.getConflictPoints(instance.getAgents()[*it7].start_location);
                            newTI.t_max = instance.getAgents()[*it7].earliest_start_time;
                            newTI.t_min = 0;
                            newTI.agent_id = *it7;
                            //std::cout << it2->conflict_point << " conflict with ";
                            // std::cout  << newTI.t_min << " " << newTI.t_max << " " << newTI.agent_id << "===";
                            // std::cout << instance.getAgents()[*it7].start_location << ":" << CPs[0] << "----\n";
                            for(auto it8 = CPs.begin(); it8 != CPs.end(); ++it8)
                                rt[*it8].push_back(newTI);

                        }
                    }
                }
                // std::cout << "\n";
            }
            
        }
    }

	//std::cout<<"running getRTFromP\n";
    // Vehicle could not arrive before the Earliest arrival time
	TimeInterval beginTI;
	beginTI.t_min = 0;
	beginTI.t_max = instance.getEarliestStartTime(index);
	beginTI.agent_id = index;
	rt[instance.getStartCP(index)].push_back(beginTI);
	std::vector<int> startCPs = instance.getConflictPoints(instance.getStartCP(index));
	for(auto it = startCPs.begin(); it != startCPs.end(); ++it)
		rt[*it].push_back(beginTI);


    // std::cout << "RTP: \n" ;
	for(auto it = p.begin(); it != p.end(); ++it){
        // std::cout << *it << ": ";
		for(auto it2 = plan[*it].begin(); it2 != plan[*it].end(); ++it2){
			//plan: vector<vector<pathentry>>
				TimeInterval newTI;
				std::vector<int> CPs = instance.getConflictPoints(it2->conflict_point);
				newTI.t_max = it2->leaving_time_tail;
				newTI.t_min = it2->arrival_time;
				newTI.agent_id = *it;
				rt[it2->conflict_point].push_back(newTI);
                // std::cout << newTI.t_min << " " << newTI.t_max << " " << newTI.agent_id << ", ===";
				//std::cout << it2->conflict_point << " conflict with ";
				for(auto it3 = CPs.begin(); it3 != CPs.end(); ++it3)
					rt[*it3].push_back(newTI);
                
                // std::cout << "    " << *it << ":" << it2->conflict_point << ":" << CPs[0] << "----";
		}
        // std::cout << "\n";

        
	}
    // std::cout << "\n\n";
	//std::cout<<"rtsize: " << rt.size() << "\n";
}

void PTNode::printPlan(){
	std::cout<<"\n\n_________________________________printing Plans_________________________\n";
	for(int i = 0; i < (signed) plan.size(); ++i){
		std::cout<<"Agent " << i << ": \t";
		for(PathEntry tmp_path : plan[i]){
			printf("Loc: %d, From %f to %f\t", tmp_path.conflict_point, tmp_path.arrival_time, tmp_path.leaving_time_tail);
		}
		std::cout<<"\n";
	}
	std::cout<<"_____________________________________________________________________________________\n\n";
}
