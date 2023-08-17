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
	// cached_milp_ptr = std::make_shared<CacheMILP>();
	if (single_agent_solver_name == "sipp-ip") {
		sipp_ip_ptr = std::make_shared<SIPP_IP>(user_instance_ptr);
	} else if (single_agent_solver_name == "bezier") {
		sipp_ptr = std::make_shared<SIPP>(user_instance_ptr);
	} else {
		std::cerr << "Invalid single agent solver!\n";
		exit(-1);
	}
	// std::cout << "Solve single agent using: " << single_agent_solver_name << std::endl;
	single_agent_solver = single_agent_solver_name;
	cutoff_runtime = cutoff_time;
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
		// bezier.optimal_T.push_back(0);
		// bezier.start_time.push_back(0);
		// bezier.control_points.push_back(std::vector<double> {});
		bezier.push_back(empty_node);
	};

}

bool PBS::SolveSingleAgentSIPP_IP(PTNode& node, int agent_id)
{
	ReservationTable rt(instance_ptr->GetNumOfVertices());
	instance_ptr->GetReservationTable(rt);
	std::set<int> rtp;
	node.getRTP(rtp, agent_id);;
	// node.getRTFromP(*instance_ptr, rt, rtp, *it, trajectoryToAgent);
	node.getRTFromP(*instance_ptr, rt, rtp);

	// printRT(rt);
	DEBUG(printf("For agent %d: ", agent_id);)
	if (DEBUG_PRINT) {
		for (int tmp_agent_id: rtp) {
			printf("agent %d, \t", tmp_agent_id);
		}
	}
	DEBUG(printf("\n");)

	// Path path = sipp_ptr->run(agent_id, rt, node.bezier_solution[agent_id]);
	Path path = sipp_ip_ptr->run(agent_id, rt);
	num_LL_generated += sipp_ip_ptr->low_node_generated;
	num_LL_expanded += sipp_ip_ptr->low_node_expanded;
	if(path.empty()) {
		return false;
	} 
	node.plan[agent_id].clear();
	node.plan[agent_id] = path;

	ReservationTable rtdebug(instance_ptr->GetNumOfVertices());
	node.getRTFromP(*instance_ptr, rtdebug, rtp);
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
		printRT(rt);
		std::cout << "agent " << agent_id <<'\n';
		printPath(path);
		// exit(-1);
		return false;
	}
	return true;
}

bool PBS::SolveSingleAgentBezier(PTNode& node, int agent_id)
{
	DEBUG(printf("single agent slover for agent: %d\n", agent_id);)
	ReservationTable rt(instance_ptr->GetNumOfVertices());
	std::set<int> rtp;
	node.getRTP(rtp, agent_id);
	// printf("rtp table\n");
	// for (int val: rtp){
	// 	printf("%d\t", val);
	// }
	// printf("\n");
	node.getWindowRTFromP(*instance_ptr, rt, rtp);
	DEBUG(printf("Get window RT\n");)
	node.InsertInitLocation(agent_id, *instance_ptr, rt);
	// printValidRT(rt);
	// if (DEBUG_PRINT) {
	// 	DEBUG(printf("For agent %d: ", agent_id);)
	// 	for (int tmp_agent_id: rtp) {
	// 		printf("agent %d, \t", tmp_agent_id);
	// 	}
	// 	printf("\n");
	// 	for (auto tmp_priority: node.priority) {
	// 		printf("agent %d: ", tmp_priority.first);
	// 		for (auto item : tmp_priority.second) {
	// 			printf("agent %d\t", item);
	// 		}
	// 		std::cout << std::endl;
	// 	}
	// }
	// printRT(rt);
	Path path;
	bool sipp_success = sipp_ptr->run(agent_id, rt, node.bezier_solution[agent_id], path, cutoff_runtime/3);
	// num_LL_generated += sipp_ptr->low_node_generated;
	// num_LL_expanded += sipp_ptr->low_node_expanded;
	if(!sipp_success) {
		// printf("No solution found!\n");
		return false;
	} 
	// printf("Agent id is: %d, The optimal T is: %f, with control points:\n", agent_id, 
	// 	node.bezier_solution[agent_id].optimal_T);
	// for (auto entry: path) {
	// 	printf("At loc: %d, from %f to %f\n", entry.location, entry.arrival_time, entry.leaving_time_tail);
	// }
	// printf("\n");
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
		// printRT(rt);
		std::cout << "agent " << agent_id <<'\n';
		printPath(path);
		exit(-1);
		return false;
	}
	return true;
}

bool PBS::CheckCollision(PTNode& node, int agent_first, int agent_second)
{
	// int count_plan_step = 0;
	for(auto first_agent_node = node.plan[agent_first].begin(); first_agent_node != node.plan[agent_first].end(); ++first_agent_node){
		// //it2 is the path entries of the agent it
		// // std::cout << "In func 0!" << std::endl;
		// // printf("Acess agent %d at location : %d\n", agent_first, it3->location);
		// if (agent_first == 7) {
		// 	printf("Path size for agent 7 is: %d, with the step %d\n", node.plan[agent_first].size(), count_plan_step);
		// }
		// count_plan_step++;
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
	DEBUG(printf("Update plan for agent %d\n", index);)
	// Showlist(list);
	// TO DO: find out this replan
	// if (DEBUG_PRINT) {
	// 	for (auto tmp_priority: node.priority) {
	// 		printf("agent %d: ", tmp_priority.first);
	// 		for (auto item : tmp_priority.second) {
	// 			printf("agent %d\t", item);
	// 		}
	// 		std::cout << std::endl;
	// 	}
	// }
	bool replanned = false;
	for(auto it = list.begin(); it != list.end(); ++it){
		//it is the index of agents in list
		DEBUG(printf("PBS::UpdatePlan loop for agent %d, is replaned: %d\n", *it, replanned);)
		if(replanned){
			for(auto it2 = list.begin(); it2 != list.end(); ++it2){
				if (*it2 == *it){
					break;
				}
				if (CheckCollision(node, *it, *it2)) {
					// printf("Solve for previous collision!\n");
					auto start_time = Time::now();
					bool solve_success;
					if (single_agent_solver == "sipp-ip") {
						solve_success = SolveSingleAgentSIPP_IP(node, *it);
					} else {
						solve_success = SolveSingleAgentBezier(node, *it);
					}
					auto end_time = Time::now();
					// printf("\n\n\n");
					std::chrono::duration<float> run_time = end_time - start_time;
					runtime_path_finding += run_time.count();
					if (!solve_success) {
						DEBUG(printf("[WARN] Single Agent solver failed in %f s\n\n\n", run_time.count());)
						return false;
					} else {
						DEBUG(printf("[INFO] Single Agent solver solved in %f s\n\n\n", run_time.count());)
					}
				}
			}
		}
		if(index == *it){
			DEBUG(printf("Solve for current collision!\n");)
			replanned = true;
			auto start_time = Time::now();
			bool solve_success;
			if (single_agent_solver == "sipp-ip") {
				solve_success = SolveSingleAgentSIPP_IP(node, *it);
			} else {
				solve_success = SolveSingleAgentBezier(node, *it);
			}
			auto end_time = Time::now();
			// printf("\n\n\n");

			std::chrono::duration<float> run_time = end_time - start_time;
			runtime_path_finding += run_time.count();
			if (!solve_success) {
				DEBUG(printf("[WARN] Single Agent solver failed in %f s\n\n\n", run_time.count());)
				return false;
			} else {
				DEBUG(printf("[INFO] Single Agent solver solved in %f s\n\n\n", run_time.count());)
			}
		}
	}
	DEBUG(printf("[INFO] Finish running update plan\n");)
	return true;
}

/**
 * Get the previous path for the next round
 * 
 * Calculate the path need for agent to reach the next location
*/
bool PBS::GetNextPreviousPath()
{
	// solution_node->bezier_solution[0].speed;
	return false;
}

void PBS::initializePriority(std::map<int, std::set<int>>& priority)
{
	for (int i = 0; i < arrivingVehicles.size(); i++) {
		priority[i] = std::set<int>();
	}
}

/**
 * The main execution function for PBS tree
 * 
 * @param outputFileName
 * TODO: 
*/
bool PBS::run(const string& outputFileName, bool& all_agents_solved)
{
	auto start_time = Time::now();
	num_PBS_called++;
	instance_ptr->GetAgents(arrivingVehicles);
    std::stack<std::shared_ptr<PTNode>> POStack;
    vector<Path> plan((int) arrivingVehicles.size());
    std::map<int, std::set<int>> priority;
    initializePriority(priority);

	BezierInfo init_bezier;
	InitBezier(init_bezier, arrivingVehicles.size());
	
	std::shared_ptr<PTNode> Root = std::make_shared<PTNode>(plan, init_bezier, priority);

	for(size_t i = 0; i < plan.size(); ++i){
		// printf("[INFO] For agent %ld, start location is: %d, goal location is: %d\n", i, instance_ptr->agents[i].start_location,
		// instance_ptr->agents[i].goal_location);
		if (instance_ptr->agents[i].is_solved) {
			continue;
		}
		if(!UpdatePlan(*Root, i)) {
			// std::cout << "no solution for init plan\n";
			// exit(-1);
			all_agents_solved = false;
			return false;
		}
	}
	Root->calculateCost(instance_ptr);
	root_cost = Root->cost;
	POStack.push(Root);

	double entire_runtime = 0;

	// int test = 0;
	while (POStack.size() != 0 and entire_runtime <= cutoff_runtime){
		// printf("\n[INFO] Run PBS new round!\n\n");
		// printf("PoStack size: %ld\n", POStack.size());
		std::shared_ptr<PTNode> N = POStack.top();
		POStack.pop();
		num_HL_expanded++;
		auto start_collision_time = Time::now();
		std::tuple<int, int, int> C;
		if (single_agent_solver == "sipp-ip") {
			C = N->getFirstCollision(*instance_ptr);
		} else {
			C = N->getFirstWindowCollision(*instance_ptr);
		}		
		// N->printPriorityMap();
		// printf("PBS::run detect new collision: from %d to %d\n", std::get<0>(C), std::get<1>(C));
		time_s collison_duration = Time::now() - start_collision_time;
		runtime_detect_conflicts += collison_duration.count();
		// No colision founded return the result
		if(std::get<0> (C) == -1){
			N->writeToFile(*instance_ptr, outputFileName);
			// std::cout << "Solution found\n";	
			solution_found = true;
			// solution_cost = N->cost;
			solution_node = N;	
			break;
		}

		// // If the collision vehicles are from the same start point
		// if( (N.priority[get<0> (C)].find(get<1> (C)) != N.priority[get<0> (C)].end()) || 
		// 	(N.priority[get<1> (C)].find(get<0> (C)) != N.priority[get<1> (C)].end())){
		// 	std::cout <<"!ERROR on iteration " << test << "\n";
		// 	return;
		// }


		auto start_time_generate = Time::now();
		std::map<int, std::set<int>> newPriority = N->priority;
		newPriority[std::get<0>(C)].insert(std::get<1>(C));
		std::shared_ptr<PTNode> newNode = std::make_shared<PTNode> (N->plan, N->bezier_solution, newPriority);
		// printPriority(newPriority);
		bool n1 = true;
		bool n2 = true;
		if(UpdatePlan(*newNode, std::get<1>(C))) {
			newNode->calculateCost(instance_ptr);
		} else {
			n1 = false;
		}
		std::map<int, std::set<int> > newPriority2 = N->priority;
		newPriority2[std::get<1>(C)].insert(std::get<0>(C));
		std::shared_ptr<PTNode> newNode2 = std::make_shared<PTNode> (N->plan, N->bezier_solution, newPriority2);
		// printPriority(newPriority2);
		if(UpdatePlan(*newNode2, std::get<0>(C))){
			newNode2->calculateCost(instance_ptr);
		} else {
			n2 = false;
		}

		if(n1 && n2){
			if(newNode->cost >= newNode2->cost){
				POStack.push(newNode);
				POStack.push(newNode2);
			}
			else{
				POStack.push(newNode2);
				POStack.push(newNode);
			}
			num_HL_generated += 2;
		} else if (n1){
			POStack.push(newNode);
			num_HL_generated++;
		} else if (n2){
			POStack.push(newNode2);
			num_HL_generated++;
		} 
		time_s generate_duration = Time::now() - start_time_generate;
		runtime_generate_child += generate_duration.count();
		// else {
		// 	std::cout << "no solution1\n";
		// 	return;
		// }
		// test++;
		time_s loop_runtime = Time::now() - start_time;
		entire_runtime = loop_runtime.count();
	}
	// auto end_time = Time::now();
	std::chrono::duration<float> time_duration = Time::now() - start_time;
	runtime += time_duration.count();
	if (solution_found) {
		GetNextRoundInit();
	} else {
		// printf("No solution!\n");
		;
	}
	for(size_t i = 0; i < plan.size(); ++i){
		if (!instance_ptr->agents[i].is_solved){
			// DEBUG(printf("Agent %ld is not solved!\n", i);)
			all_agents_solved = false;
		}
	}
	// DEBUG(printf("Finish planning for this PBS round!\n"));
	return solution_found;
}

bool PBS::GetNextRoundInit()
{
	for (unsigned int i = 0; i < instance_ptr->agents.size(); i++) {
		DEBUG(printf("[INFO] Retrive solution for agent %d!\n", i);)

		// double init_time = instance_ptr->agents[i].earliest_start_time;
		BezierNode curr_bezier = solution_node->bezier_solution[i];
		double solution_t = curr_bezier.optimal_T;
		
		// printf("The optimal T is: %f\n", curr_bezier.optimal_T);
		
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
			// printf("solution of agent %d is: The optimal T is: %f, the earliest start time: %f, with control points:\n", i, 
			// 	solution_node->bezier_solution[i].optimal_T, instance_ptr->agents[i].earliest_start_time);
			// for (double point: solution_node->bezier_solution[i].control_points) {
			// 	printf("%f,\t", point);
			// }
			// printf("\n");
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
			// printf("tmp_travel_dist: %f, arr next dist: %f, leave prev_dist: %f, prev_node_dist: %f, next_node_dist: %f\n",
				// tmp_travel_dist, arr_next_dist, leave_prev_dist, prev_node_dist, next_node_dist);

			// double time_arr_next = -1.0;
			double time_leave_prev = -1.0;

			if (tmp_travel_dist < arr_next_dist) {
				// while (tmp_travel_dist < next_node_dist) {
				// 	time_vist_next_node += TIME_STEP_SIZE;
				// 	tmp_travel_dist = tmp_bern.GetVal(time_vist_next_node-instance_ptr->agents[i].earliest_start_time, curr_bezier.control_points);
				// 	printf("At time: %f, tmp dist traveled: %f\n", time_vist_next_node, tmp_travel_dist);
				// 	if (tmp_travel_dist > arr_next_dist and time_arr_next < 0) {
				// 		time_arr_next = time_vist_next_node;
				// 	} else if (tmp_travel_dist > leave_prev_dist and time_leave_prev < 0) {
				// 		time_leave_prev = time_vist_next_node;
				// 	}
				// }
				// PathEntry prev_node;
				// prev_node.arrival_time = 0;
				// prev_node.leaving_time_tail = time_leave_prev - REPLAN_SIZE;
				// prev_node.location = prev_start_loc;
				// PathEntry next_node;
				// next_node.arrival_time = time_arr_next - REPLAN_SIZE;
				// next_node.leaving_time_tail = time_vist_next_node - REPLAN_SIZE;
				// next_node.location = next_start_loc;
				// printf("[11] Time arr: %f, leave: %f\n", 0.0, time_leave_prev);
				// printf("[12] Time arr: %f, leave: %f\n", time_arr_next, time_vist_next_node);


				instance_ptr->agents[i].previous_path.clear();
				next_start_loc = prev_start_loc;
				// instance_ptr->agents[i].previous_path.push_back(prev_node);
				// instance_ptr->agents[i].previous_path.push_back(next_node);

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
				// printf("[21] Time arr: %f, leave: %f\n", 0.0, time_leave_prev);
				// printf("[22] Time arr: %f, leave: %f\n", 0.0, time_vist_next_node);

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
				// printf("[1] Time arr: %f, leave: %f\n", 0.0, time_vist_next_node);

				instance_ptr->agents[i].previous_path.clear();
				instance_ptr->agents[i].previous_path.push_back(next_node);
			}
			double speed = (tmp_travel_dist - tmp_bern.GetVal(time_vist_next_node-instance_ptr->agents[i].earliest_start_time - TIME_STEP_SIZE, 
					curr_bezier.control_points))/TIME_STEP_SIZE;
			speed = round(speed*10)/10;
			instance_ptr->agents[i].init_velocity = max(instance_ptr->agents[i].v_min+EPS, min(instance_ptr->agents[i].v_max, speed));
			instance_ptr->agents[i].start_location = next_start_loc;
			double earliest_start_t = time_vist_next_node - REPLAN_SIZE;
			// earliest_start_t = floor(earliest_start_t/TIME_STEP_SIZE)*TIME_STEP_SIZE;
			instance_ptr->agents[i].earliest_start_time = earliest_start_t;
			instance_ptr->agents[i].total_cost += REPLAN_SIZE;
			// printf("[INFO] Solution for Agent %d, next round start location is: %d, start at time: %f, init speed is: %f\n", 
			// 	i, instance_ptr->agents[i].start_location
			// 	, instance_ptr->agents[i].earliest_start_time,
			// 	instance_ptr->agents[i].init_velocity);
			// printPath(instance_ptr->agents[i].previous_path);
			// printPath(solution_node->plan[i]);
		}
	}
	DEBUG(printf("Get all the agent's path!\n"));
	return true;
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
		addHeads << "runtime, is solved, #high-level expanded,#high-level generated,#low-level expanded,#low-level generated," <<
			"solution cost,root g value,#PBS-called," <<
			"runtime of detecting conflicts,runtime of building constraint tables,runtime of building CATs," <<
			"runtime of path finding,runtime of generating child nodes," <<
			"preprocessing runtime,solver name,instance name,#agents" << endl;
		addHeads.close();
	}
	ofstream stats(fileName, std::ios::app);
	stats << runtime << "," << all_agents_solved << ", " <<
		num_HL_expanded << "," << num_HL_generated << "," <<
		num_LL_expanded << "," << num_LL_generated << "," <<

		solution_cost << "," << root_cost << "," << num_PBS_called << "," <<

		runtime_detect_conflicts << "," << runtime_build_CT << "," << runtime_build_CAT << "," <<
		runtime_path_finding << "," << runtime_generate_child << "," <<

		runtime_preprocessing << "," << getSolverName() << "," << instanceName << "," << instance_ptr->agents.size() << endl;
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