/**
 * Implementation of the PBS class
 * 
 * TODO: add more info
*/

#include "../include/PBS.h"
#include "Common.h"
#include "milp_cache.h"
#include <boost/property_map/property_map.hpp>
#include <chrono>
#include <cmath>
#include <memory>
#include <vector>

/**
 * Class constructor for PBS
 * 
 * @param instance the pointer of instance that is derived from the input files
*/
PBS::PBS(std::shared_ptr<Instance> user_instance_ptr)
{
	instance_ptr = user_instance_ptr;
	cached_milp_ptr = std::make_shared<CacheMILP>();
	sipp_ptr = std::make_shared<SIPP>(user_instance_ptr);
	arrivingVehicles = instance_ptr->getAgents();
	initializeAgents();
}

/**
 * Initialize the configuration of all agents
 * 
 * Load the instance, and generate trajectoryToAgent which maps trajectory to the list of agent. For each trajectory, the agent is listed with its
 * arriving order.
 * 
 * @param NULL
*/
void PBS::initializeAgents(){
	// TODO:single loop is enough, this can also be problemtic when using online version
	bool* visited = new bool[arrivingVehicles.size()];
	for (int i = 0; i < (signed) arrivingVehicles.size(); ++i)
        visited[i] = false;

	for(int i = 0; i < (signed) arrivingVehicles.size(); ++i){
		if(visited[i]) continue;

		std::map<double, int>  samePath;
		samePath[arrivingVehicles[i].earliest_start_time] = i;
		visited[i] = true;
		for(int j = 0; j < (signed) arrivingVehicles.size(); ++j){
			if(visited[j]) continue;
			if((arrivingVehicles[i].trajectory[0] == arrivingVehicles[j].trajectory[0])
				&& ((arrivingVehicles[i].trajectory[arrivingVehicles[i].trajectory.size()-1] == arrivingVehicles[j].trajectory[arrivingVehicles[j].trajectory.size()-1]))){
				samePath[arrivingVehicles[j].earliest_start_time] = j;
				visited[j] = true;
			}
		}
		
		for(auto it = samePath.begin(); it != samePath.end(); ++it){
			trajectoryToAgent[arrivingVehicles[i].trajectory[0]][arrivingVehicles[i].trajectory[arrivingVehicles[i].trajectory.size()-1]].push_back(it->second);
		} 
	}
	delete[] visited;
}

void Showlist(std::list<int>& tmp_list)
{
	for (auto tmp_item: tmp_list){
		std::cout << tmp_item << ", ";
	}
	std::cout << std::endl;
}

/**
 * Update the path that is planned at each node
 * 
 * @param node The node of the priority tree that needs to be modified
 * @param index The index of the agent
 * @return If success return True
*/
bool PBS::UpdatePlan(PTNode& node, int index)
{
	std::list<int> list = node.topologicalSort(index);
	list.reverse();
	// Showlist(list);
	bool replanned = false;
	for(auto it = list.begin(); it != list.end(); ++it){
		//it is the index of agents in list
		if(replanned){
			for(auto it2 = list.begin(); it2 != list.end(); ++it2){
				if (*it2 == *it){
					break;
				}
				for(auto it3 = node.plan[*it].begin(); it3 != node.plan[*it].end(); ++it3){
					//it2 is the path entries of the agent it

					for(auto it4 = node.plan[*it2].begin(); it4 != node.plan[*it2].end(); ++it4){
						//it3 is the path entries of the agent index

						if( instance_ptr->isSamePoint(it3->conflict_point, it4->conflict_point)){
							if(!(it3->leaving_time_tail - it4->arrival_time < EPSILON) && !(it4->leaving_time_tail - it3->arrival_time < EPSILON)){
								ReservationTable rt(instance_ptr->getNumOfVertices());
								std::set<int> rtp;
								node.getRTP(rtp, *it);;
								node.getRTFromP(*instance_ptr, rt, rtp, *it, trajectoryToAgent);
								// std::cout << "In func 1!" << std::endl;

								Path path = sipp_ptr->run(*it, rt, cached_milp_ptr,node.optimal_T[*it], node.start_time[*it],
									node.control_points[*it]);
								// printf("All control points for %d in PBS1 : ", *it);
								// for (double point: node.control_points[*it]){
								// 	std::cout << point << ";";
								// }
								// std::cout << std::endl;
								// std::cout << "In func 2!" << std::endl;
								if(path.empty()) return false;
								node.plan[*it] = path;



								ReservationTable rtdebug(instance_ptr->getNumOfVertices());
								node.getRTFromP(*instance_ptr, rtdebug, rtp, *it, trajectoryToAgent);
								if(!checkValid(rtdebug, path, *it)) {
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
									std::cout << "replanned agent " << *it << '\n';
									std::cout << "rtp: ";
									for(auto itt = rtp.begin(); itt != rtp.end(); ++ itt){
										std::cout << *itt <<" ";
									}
									printRT(rt);
									std::cout <<"agent " <<*it <<'\n';
									printPath(path);
									exit(-1);
									return false;
								}
							}
						}
					}
				}
			}
		}
		if(index == *it){
			ReservationTable rt(instance_ptr->getNumOfVertices());
			std::set<int> rtp;
			node.getRTP(rtp, *it);
			// std::cout << "In func 3!" << std::endl;
			node.getRTFromP(*instance_ptr, rt, rtp, *it, trajectoryToAgent);
			// std::cout << "In func 4!" << std::endl;
			Path path = sipp_ptr->run(*it, rt, cached_milp_ptr,node.optimal_T[*it], node.start_time[*it],
										node.control_points[*it]);
			// printf("All control points for %d in PBS2 : ", *it);
			// for (double point: node.control_points[*it]){
			// 	std::cout << point << ";";
			// }
			// std::cout << std::endl;
			// std::cout << "In func 5!" << std::endl;
			if(path.empty()) return false;
			node.plan[index] = path;
			replanned = true;
			

			ReservationTable rtdebug(instance_ptr->getNumOfVertices());
			node.getRTFromP(*instance_ptr, rtdebug, rtp, *it, trajectoryToAgent);
			if(!checkValid(rtdebug, path, *it)) {
				std::fstream outputRT("ReservationTable.txt", std::ios::out | std::ios::binary);
				for(auto itt = rt.begin(); itt != rt.end(); ++itt){
					for(auto itt2 = itt->begin(); itt2 != itt->end(); ++itt2){
						outputRT.write((char*)&itt2->t_min, sizeof(itt2->t_min));
						outputRT.write((char*)&itt2->t_max, sizeof(itt2->t_max));
						outputRT.write((char*)&itt2->agent_id, sizeof(itt2->agent_id));
					}
					char space = ' ';
					outputRT.write((char*)& space, sizeof(space));
				}
				outputRT.close();
				std::cout << "List: ";
				for(auto it = list.begin(); it != list.end(); ++ it){
					std::cout << *it <<" ";
				}
				printRT(rt);
				std::cout <<"agent " <<*it <<'\n';
				printPath(path);
				exit(-1);
				return false;
			}
		}
	}
	return true;
}

/**
 * The main execution function for PBS tree
 * 
 * @param outputFileName
 * TODO: 1. change the stack to heap
*/
bool PBS::run(const string& outputFileName)
{
	auto pbs_start = Time::now();
	// This should be a priority queue
    std::stack<PTNode> POStack;
    vector<Path> plan((int) arrivingVehicles.size());
    std::map<int, std::set<int>> priority;
    initializePriority(priority);

    // printPriority(priority);
	std::vector<double> init_optimal_T;
	std::vector<double> init_start_time;
	std::vector<std::vector<double>> init_control_points;
	for(int i = 0; i < (int) arrivingVehicles.size(); i++){
		init_optimal_T.push_back(0);
		init_start_time.push_back(0);
		init_control_points.push_back(std::vector<double> {});
	}
	PTNode Root = PTNode(plan, init_control_points, init_optimal_T, init_start_time, priority);

	for(size_t i = 0; i < plan.size(); ++i)
		if(!UpdatePlan(Root, i)) std::cout << "no solution1\n" ;
	Root.calculateCost();
	root_cost = Root.cost;
	POStack.push(Root);
	bool solution_flag = false;

	int test = 0;
	while (POStack.size() != 0){
		// printf("\n[INFO] Run PBS new round!\n\n");
		PTNode N = POStack.top();
		POStack.pop();

		std::tuple<int, int, int> C = N.getFirstCollision(*instance_ptr);
		// printf("collision from %d to %d, at %d\n", get<0> (C), get<1> (C), get<2> (C));
		// No colision founded return the result
		if(std::get<0> (C) == -1){
			N.writeToFile(*instance_ptr, outputFileName);
			solution_cost = N.cost;	
			solution_flag = true;
			solution_delay = N.calculateDelay(instance_ptr);		
			// N.printPlan();
			break;
		}

		// If the collision vehicles are from the same start point
		if( (N.priority[get<0> (C)].find(get<1> (C)) != N.priority[get<0> (C)].end()) || 
			(N.priority[get<1> (C)].find(get<0> (C)) != N.priority[get<1> (C)].end())){
			std::cout <<"!ERROR on iteration " << test << "\n";
			solution_flag = false;
			break;
		}
		std::map<int, std::set<int>> newPriority = N.priority;
		newPriority[std::get<0>(C)].insert(std::get<1>(C));
		PTNode newNode = PTNode(N.plan, N.control_points, N.optimal_T, N.start_time, newPriority);
		// printPriority(newPriority);
		bool n1 = true;
		bool n2 = true;
		if(UpdatePlan(newNode, std::get<1>(C))) {
			newNode.calculateCost();
		} else {
			n1 = false;
		}
		std::map<int, std::set<int> > newPriority2 = N.priority;
		newPriority2[std::get<1>(C)].insert(std::get<0>(C));
		PTNode newNode2 = PTNode(N.plan, N.control_points, N.optimal_T, N.start_time, newPriority2);
		// printPriority(newPriority2);
		if(UpdatePlan(newNode2, std::get<0>(C))){
			newNode2.calculateCost();
		} else {
			n2 = false;
		}

		if(n1 && n2){
			if(newNode.cost >= newNode2.cost){
				POStack.push(newNode);
				POStack.push(newNode2);
			}
			else{
				POStack.push(newNode2);
				POStack.push(newNode);
			}
		} else if (n1){
			POStack.push(newNode);
		} else if (n2){
			POStack.push(newNode2);
		}
		test++;
	}
	if (!solution_flag) {
		std::cout << "no solution2\n";
	}
	std::chrono::duration<float> time_duration = Time::now() - pbs_start;
	runtime = time_duration.count();
	return solution_flag;
}

void PBS::saveResults(const string &fileName, const string &instanceName) const
{
	std::ifstream infile(fileName);
	bool exist = infile.good();
	infile.close();
	if (!exist) {
		std::ofstream addHeads(fileName);
		addHeads << "runtime, #high-level expanded,#high-level generated,#low-level expanded,#low-level generated," <<
			"solution cost,root g value,delay," <<
			"runtime of detecting conflicts,runtime of building constraint tables,runtime of building CATs," <<
			"runtime of path finding,runtime of generating child nodes," <<
			"preprocessing runtime,instance name" << std::endl;
		addHeads.close();
	}
	std::ofstream stats(fileName, std::ios::app);
	stats << runtime << "," <<
		num_HL_expanded << "," << num_HL_generated << "," <<
		num_LL_expanded << "," << num_LL_generated << "," <<
		solution_cost << "," << root_cost << "," << solution_delay << "," <<
		runtime_detect_conflicts << "," << runtime_build_CT << "," << runtime_build_CAT << "," <<
		runtime_path_finding << "," << runtime_generate_child << "," <<
		runtime_preprocessing << "," << instanceName << std::endl;
	stats.close();
}


/**
 * Initialize the priority list for PBS
 * 
 * The initial priority list is generated using the order of the arrive time at each start point
 * 
 * @param p
*/
void PBS::initializePriority(std::map<int, std::set<int> >& p){
	for(auto it = trajectoryToAgent.begin(); it != trajectoryToAgent.end(); ++it){
		for(auto it2 = it->second.begin(); it2 != it->second.end(); ++it2){

			for(auto it3 = it2->second.begin(); it3 != it2->second.end(); ++it3){
				for(auto it4 = next(it3); it4 != it2->second.end(); ++it4){
                    if (instance_ptr->getAgents()[*it3].earliest_start_time < instance_ptr->getAgents()[*it4].earliest_start_time){
                        p[*it3].insert(*it4);
                    }
				}


                int conflictPointID = instance_ptr->getConflictPoints(it->first)[0];
                for(auto it5 = trajectoryToAgent.begin(); it5 != trajectoryToAgent.end(); ++it5){
                    if (conflictPointID == it5->first){
                        // std::cout <<  conflictPointID << "--" << it5->first << std::endl;
                        for(auto it6 = it5->second.begin(); it6 != it5->second.end(); ++it6){
                            for(auto it7 = it6->second.begin(); it7 != it6->second.end(); ++it7){
                                // std::cout << "agent: " << *it3 << "-" << *it7 << std::endl;
                                if (instance_ptr->getAgents()[*it3].earliest_start_time < instance_ptr->getAgents()[*it7].earliest_start_time){
                                    p[*it3].insert(*it7);
                                }
                            }
                        }
                    }
                }


			}
		}
	}
}


void PBS::printPath(Path path){
	std::cout<<"\n\n_________________________________printing Path____________________________________\n";
	std::cout << "path size: " << path.size() <<"\n";
		for(auto itt = path.begin(); itt != path.end(); ++itt){
			std::cout << "cp" << itt->conflict_point << "\tarrival: "  << itt->arrival_time << "\tleave: " << itt->leaving_time_tail << "\n";
	}
	std::cout<<"__________________________________________________________________________________\n\n";
}

void PBS::printRT(ReservationTable rt){
	std::cout<<"\n\n_________________________________printing ReservationTable_________________________\n";
	for(int i = 0; i < (signed) rt.size(); ++i){
		std::cout<<"cp" << i << "\t";
		for(auto ittemp = rt[i].begin(); ittemp != rt[i].end(); ++ittemp){
			std::cout<< ittemp->t_min << "\t" <<ittemp->t_max << "\t#" << ittemp->agent_id << "#\t";
		}
		std::cout<<"\n";
	}
	std::cout<<"_____________________________________________________________________________________\n\n";
}

/**
 * Print the order of the vehicles at each start point
*/
void PBS::printPriority(std::map<int, std::set<int>> p){
	std::cout<<"\n\n_________________________________printing Priority_______________________________\n";
	for(auto it = p.begin(); it != p.end(); ++it){
		std::cout << it->first << ": ";
		for(auto it2 = it->second.begin(); it2 != it->second.end(); ++it2)
			std::cout << *it2 << " ";
		std::cout << "\n";
	}
	std::cout<<"_________________________________________________________________________________\n\n";
}


bool PBS::checkValid(ReservationTable& rt, Path& path, int agent){
	for(auto it = path.begin(); it != path.end(); ++it){
		int cp = 0;
		for(auto it2 = rt.begin(); it2 != rt.end(); ++it2,++cp){
			if(it->conflict_point == cp){
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