// /**
//  * Implementation of TGP Solver
//  * 
//  * 
// */
// #include "../include/tpg_solver.h"

// /**
//  * TODO: remove it, just for debug
// */
// struct Node
// {
//     int current_point;
//     int next_point;
//     int previous_point;
    
//     double arrival_time_min; // arrival time of the head of the vehicle
//     double arrival_time_max; // arrival time of the head of the vehicle
    
//     double cost_min;


// 	double g;      // 
// 	double f;      // f = h + g;
// 	double h;      // 
// 	int color;  // marking the point to see whether it was visited or not

//     int interval_index;
//     double interval_t_min;
//     double interval_t_max;

//     int index;
//     Node *parent;

//     int parent_index_1;
//     int parent_index_2;
// };

// /**
//  * Transform the TGP to MILP problem
//  * 
//  * @param tgp_node The original TGP problem
//  * @return The success of the operation
// */
// bool TGP_Solver::TGP2MILP(int n_points, double t_length)
// {
//     // variables to be added ********************
//     double dis_init;
//     double v_init;
//     double v_min;
//     double v_max;
//     double a_min;
//     double a_max;
//     vector<Node> result_nodes;
//     vector<int> trajectory;

//     // end of list *******************


//     BernsteinPolynomial bern_poly(n_points, t_length);

//     IloEnv env = IloEnv();
//     IloModel model = IloModel(env);
//     // variable of controls points, TODO: add conditions
//     // n control points and 1 init time
//     IloNumVarArray control_points(env, n_points+1);

//     IloExpr sum_obj = IloExpr(env);
//     IloRangeArray con(env);

//     // Set start condition
//     IloExpr init_place_expr(env);
//     std::vector<double> init_bern_val;
//     bern_poly.CalculateVal(0, init_bern_val);
//     for (int i = 0; i < n_points; i++) {
//         init_place_expr += control_points[i]*init_bern_val[i];
//     }
//     model.add(init_place_expr == dis_init);
//     init_place_expr.end();

//     // Set end condition
//     IloExpr end_place_expr(env);
//     std::vector<double> end_bern_val;
//     bern_poly.CalculateVal(t_length, end_bern_val);
//     for (int i = 0; i < n_points; i++) {
//         end_place_expr += control_points[i]*end_bern_val[i];
//     }
//     model.add(end_place_expr == dis_init);
//     end_place_expr.end();

//     // First order derivative of bernstein polynomial
//     BernsteinPolynomial first_order_deriv = bern_poly.Derivative();


//     // Add init speed condition
//     IloExpr init_speed_expr(env);
//     // For simplicity, we just use 2 here. For any order derivative, use pow(2, first_order_deriv.control_points_deriv.deriv_order) instead
//     for (int i = 0; i < 2; i++) {
//         init_speed_expr += first_order_deriv.control_points_deriv.ratio * first_order_deriv.control_points_deriv.sign_idx[0][i] 
//                             * control_points[first_order_deriv.control_points_deriv.orignal_idx[0][i]];
//     }
//     model.add(init_speed_expr == v_init);
//     init_speed_expr.end();

//     // Speed constraints
//     for (int i = 1; i < first_order_deriv.control_points_deriv.N; i++) {
//         IloExpr speed_expr(env);
//         // For simplicity, we just use 2 here. For any order derivative, use pow(2, first_order_deriv.control_points_deriv.deriv_order) instead
//         for (int j = 0; j < 2; i++) {
//             speed_expr += first_order_deriv.control_points_deriv.ratio * first_order_deriv.control_points_deriv.sign_idx[i][j] 
//                                 * control_points[first_order_deriv.control_points_deriv.orignal_idx[i][j]];
//         }
//         model.add(speed_expr <= v_max);
//         model.add(speed_expr >= v_min);
//         speed_expr.end();
//     }

//     // Acceleration constraints
//     BernsteinPolynomial second_order_deriv = first_order_deriv.Derivative();
//     for (int i = 0; i < second_order_deriv.control_points_deriv.N; i++) {
//         IloExpr acce_expr(env);
//         // For simplicity, we just use 4 here. For any order derivative, use pow(2, second_order_deriv.control_points_deriv.deriv_order) instead
//         for (int j = 0; j < 4; i++) {
//             acce_expr += second_order_deriv.control_points_deriv.ratio * second_order_deriv.control_points_deriv.sign_idx[i][j] 
//                                 * control_points[second_order_deriv.control_points_deriv.orignal_idx[i][j]];
//         }
//         model.add(acce_expr <= a_max);
//         model.add(acce_expr >= a_min);
//         acce_expr.end();
//     }


//     // Vertex time window constraints
//     for (int i =0; i < trajectory.size(); i++){
//         // arrival vertex time
//         IloExpr arr_place_expr(env);
//         std::vector<double> arr_bern_val;
//         bern_poly.CalculateVal(result_nodes[i].arrival_time_min, arr_bern_val);
//         for (int i = 0; i < n_points; i++) {
//             arr_place_expr += control_points[i]*arr_bern_val[i];
//         }
//         model.add(arr_place_expr <= pairDistancesMap[result_nodes[0].current_point][result_nodes[i].current_point]);
//         arr_place_expr.end();

//         // leave vertex time
//         IloExpr leave_place_expr(env);
//         std::vector<double> leave_bern_val;
//         bern_poly.CalculateVal(result_nodes[i].arrival_time_max, leave_bern_val);
//         for (int i = 0; i < n_points; i++) {
//             leave_place_expr += control_points[i]*leave_bern_val[i];
//         }
//         int direction = vNameToDirection[result_nodes[i].current_point];
//         model.add(leave_place_expr >= (pairDistancesMap[result_nodes[0].current_point][result_nodes[i].current_point] + Li(direction, length)) );
//         leave_place_expr.end();
//         // IloExpr node_arrival_time_min = var[1] + pairDistancesMap[result_nodes[0].current_point][result_nodes[i].current_point]*var[0];

//         // int direction = vNameToDirection[result_nodes[i].current_point]; 
//         // IloExpr node_leaving_time_tail = node_arrival_time_min + Li(direction, length)/w + Li(direction, length)*var[0];

//         // if ( i > 0){
//         //     con.add(  node_arrival_time_min - result_nodes[i].arrival_time_min  >=0);

//         // }
//         // con.add(node_leaving_time_tail - result_nodes[i].arrival_time_max  <=0);                
//     }

//     IloExpr init_place_expr(env);
//     std::vector<double> init_bern_val;
//     bern_poly.CalculateVal(0, init_bern_val);
//     for (int i = 0; i < n_points; i++) {
//         init_place_expr += control_points[i]*init_bern_val[i];
//     }
//     model.add(init_place_expr == dis_init);
//     init_place_expr.end();

//     // ****************
//     // //speed
//     // var.add(IloNumVar(env, 1/v_max,1/v_min, ILOFLOAT));  //t
//     // //start time
//     // var.add(IloNumVar(env, result_nodes[0].interval_t_min, result_nodes[0].interval_t_max, ILOFLOAT));  //t

//     // int final_conflict_point_direction = vNameToDirection[result_nodes.back().current_point];
//     // sum_obj = var[1] + pairDistancesMap[result_nodes[0].current_point][result_nodes.back().current_point]*var[0] + Li(final_conflict_point_direction, length)/w + Li(final_conflict_point_direction, length)*var[0];

//     // model.add(IloMinimize(env, sum_obj));

    
//     // for (int i =0; i < trajectory.size(); i++){
//     //     IloExpr node_arrival_time_min = var[1] + pairDistancesMap[result_nodes[0].current_point][result_nodes[i].current_point]*var[0];

//     //     int direction = vNameToDirection[result_nodes[i].current_point]; 
//     //     IloExpr node_leaving_time_tail = node_arrival_time_min + Li(direction, length)/w + Li(direction, length)*var[0];

//     //     if ( i > 0){
//     //         con.add(  node_arrival_time_min - result_nodes[i].arrival_time_min  >=0);

//     //     }
//     //     con.add(node_leaving_time_tail - result_nodes[i].arrival_time_max  <=0);                
//     // }

//     // model.add(con);
//     IloCplex cplex(model);
//     cplex.setOut(env.getNullStream());
//     cplex.setWarning(env.getNullStream());
//     cplex.setError(env.getNullStream());
//     cplex.extract(model);
//     cplex.exportModel("./model/model.lp");
    
    
    
// }

// // bool Optimize()
// // {
// //     if (cplex.solve()){
// //         Path potential_path;
        
// //         cplex.writeSolution("solution");

// //         for (int i =0; i < trajectory.size(); i++){
// //             PathEntry a_path;

// //             a_path.conflict_point = result_nodes[i].current_point;

// //             a_path.arrival_time = cplex.getValue(var[1]) + pairDistancesMap[result_nodes[0].current_point][result_nodes[i].current_point]*cplex.getValue(var[0]);

// //             int direction = vNameToDirection[result_nodes[i].current_point]; 
// //             a_path.leaving_time_tail = a_path.arrival_time + Li(direction, length)/w + Li(direction, length)*cplex.getValue(var[0]);

// //             potential_path.push_back(a_path);

// //         }                
// //         env.end();
// //         sum_obj.end();

        
// //         if (result_path.size() == 0){
// //             copy(potential_path.begin(), potential_path.end(), back_inserter(result_path));
// //         }
// //         else{
// //             if (potential_path.back().leaving_time_tail < result_path.back().leaving_time_tail){
// //                 result_path.clear();
// //                 copy(potential_path.begin(), potential_path.end(), back_inserter(result_path));
// //             }
// //         }

// //     }

           
           

// //         }
// //         else{
// //             node_storage[s.index].push_back(s);

// //             // int current_position = find_point(trajectory.size(), p, open[k].current_point);
// //             Successors successors = get_successors(p, s, trajectory.size(), v_min, v_max, length, rt, first_conflict_point_counter);

// //             for (int j = 0; j < successors.size(); ++j){

// //                 int next_position = successors[j].index;
// //                 successors[j].parent = &(node_storage[s.index].back());

// //                 successors[j].parent_index_1 = s.index;
// //                 successors[j].parent_index_2 = node_storage[s.index].size() -1;

// //                 // cout << "Compare:  " << successors[j].arrival_time_min << "    " << s.arrival_time_min + successors[j].cost_min<< endl;
// //                 successors[j].g = successors[j].arrival_time_min;
// //                 successors[j].f = successors[j].g + successors[j].h;



// //                 open.push_back(successors[j]);

// //             }


// //         }
// //     }
// //     if (result_path.size() == 0){
// //         cout << "finish" << endl;
// //         // return a Path, which is a vector of PathEntry. a PathEntry contains vertex (int), arrivalTime (double) and leavingTime (double)
// //         Path result_empty_path;
// //         return result_empty_path;

// //     }else{
// //         return result_path;
// //     }
// // }


/***********************************************************************/

// bool SearchInterval(double begin_time, double end_time, double& block_start, double& block_end, BernsteinPolynomial& bern)
// {
//     double STEP_SIZE = 0.3;
//     block_start = begin_time;
//     while(true) {
//         if (true) {
//             break;
//         }
//         block_start += STEP_SIZE;
//     }
//     block_start -= STEP_SIZE;
//     block_end = block_start;
//     while(true) {
//         if (true) {
//             break;
//         }
//         block_end += STEP_SIZE;
//     }
//     return true;
// }

// bool SIPP::SolveMILP(int agentID, double& milp_end_time)
// {
//     Agent agent = agents[agentID];
//     vector<int> trajectory = agent.trajectory;
//     double v_min = agent.v_min;
//     double v_max = agent.v_max;
//     double length = agent.length; // length of the vehicle

//     IloEnv env = IloEnv();
//     IloModel model = IloModel(env);
//     IloExpr sum_obj = IloExpr(env);
//     IloNumVarArray var(env);
//     IloRangeArray con(env);
//     bool flag = true;

//     //speed
//     // var.add(IloNumVar(env, 1/v_max,1/v_min, ILOFLOAT));  //t
//     var.add(IloNumVar(env, 1/v_min -0.001,1/v_min, ILOFLOAT));  //t
//     //start time
//     var.add(IloNumVar(env, result_nodes[0].interval_t_min, result_nodes[0].interval_t_max, ILOFLOAT));  //t

//     int final_conflict_point_direction = vNameToDirection[result_nodes.back().current_point];
//     sum_obj = var[1] + pairDistancesMap[result_nodes[0].current_point][result_nodes.back().current_point]*var[0] + Li(final_conflict_point_direction, length)/w + Li(final_conflict_point_direction, length)*var[0];

//     model.add(IloMinimize(env, sum_obj));
//     for (int i =0; i < trajectory.size(); i++){
//         IloExpr node_arrival_time_min = var[1] + pairDistancesMap[result_nodes[0].current_point][result_nodes[i].current_point]*var[0];

//         int direction = vNameToDirection[result_nodes[i].current_point]; 
//         IloExpr node_leaving_time_tail = node_arrival_time_min + Li(direction, length)/w + Li(direction, length)*var[0];

//         if ( i > 0){
//             con.add(node_arrival_time_min - result_nodes[i].arrival_time_min  >=0);

//         }
//         con.add(node_leaving_time_tail - result_nodes[i].arrival_time_max  <=0);                
//     }   
//     model.add(con);

//     IloCplex cplex(model);
//     cplex.setOut(env.getNullStream());
//     cplex.setWarning(env.getNullStream());
//     cplex.setError(env.getNullStream());
//     // cplex.extract(model);
//     // cplex.exportModel("./output/model.lp");
//     sum_obj.end();
//     auto startTime = std::chrono::high_resolution_clock::now();
    
//     milp_end_time = 0;
//     if (cplex.solve()){
//         auto stopTime = std::chrono::high_resolution_clock::now();
//         auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stopTime - startTime); 
//         std::cout << "[INFO] MILP solved with runtime: " << duration.count()/1000000.0 << std::endl;
//         printf("Speed is: %f, and init time is: %f\n", cplex.getValue(var[0]), cplex.getValue(var[1]));
//         Path potential_path;
//         cplex.writeSolution("./output/solution");
//         for (int i =0; i < trajectory.size(); i++){
//             PathEntry a_path;
//             a_path.conflict_point = result_nodes[i].current_point;
//             // Only need to modify this
//             a_path.arrival_time = cplex.getValue(var[1]) + pairDistancesMap[result_nodes[0].current_point][result_nodes[i].current_point]*cplex.getValue(var[0]);
//             // a_path.arrival_time = 0;
            

//             int direction = vNameToDirection[result_nodes[i].current_point]; 
            
//             // Only need to modify this
//             a_path.leaving_time_tail = a_path.arrival_time + Li(direction, length)/w + Li(direction, length)*cplex.getValue(var[0]);

//             potential_path.push_back(a_path);
//         }     
//         milp_end_time = potential_path[trajectory.size()-1].leaving_time_tail;    
//         printf("The solution for MILP is %f.\n", milp_end_time);           

//         // if (result_path.size() == 0){
//         //     copy(potential_path.begin(), potential_path.end(), back_inserter(result_path));
//         // }
//         // else{
//         //     if (potential_path.back().leaving_time_tail < result_path.back().leaving_time_tail){
//         //         result_path.clear();
//         //         copy(potential_path.begin(), potential_path.end(), back_inserter(result_path));
//         //     }
//         // }
//         flag = true;
//     } else {
//         std::cout << "[INFO] MILP Not solved" << std::endl;
//         flag = false;
//     }
//     env.end();
//     return flag;
// }

// bool SIPP::SolveBezier(
//     int agentID,
//     double T_optimal,
//     double& t_length,
//     double& earliest_start_time,
//     std::vector<double>& solution_control_points,
//     Path& result_path)
// {
//     // start own optimization ************************************************************************
//     Agent agent = agents[agentID];
//     vector<int> trajectory = agent.trajectory;

//     // TODO: tune this parameters
//     double init_drift = 1;
//     int n_points = 30;

//     double a_min = agent.a_min;
//     double a_max = agent.a_max;
//     double v_min = agent.v_min;
//     double v_max = agent.v_max;
//     double length = agent.length;
//     earliest_start_time = max(agent.earliest_start_time, result_nodes[0].arrival_time_min);
    
//     int traj_size = trajectory.size();
//     // double t_length = result_nodes[trajectory.size()-1].arrival_time_min;
//     // double milp_end_time = 0;
//     // bool milp_success = SolveMILP(agentID, milp_end_time);
//     // if (milp_success){
//     //     t_length = milp_end_time - earliest_start_time;
//     // } else{
//     //     t_length = result_nodes[trajectory.size()-1].arrival_time_max - earliest_start_time;
//     // }
//     t_length = T_optimal - earliest_start_time;
//     std::cout << "Earliest_start_time is: " << earliest_start_time << std::endl;
//     std::cout << "T value is: " << t_length << std::endl;
//     // std::vector<double> solution_control_points;

//     BernsteinPolynomial bern_poly(n_points, t_length);
//     // First order derivative of bernstein polynomial
//     BernsteinPolynomial first_order_deriv = bern_poly.Derivative();
//     // Second order derivative of bernstein polynomial
//     BernsteinPolynomial second_order_deriv = first_order_deriv.Derivative();

//     IloEnv env = IloEnv();
//     IloModel model = IloModel(env);
//     // variable of controls points, TODO: add conditions
//     // n control points and 1 init time
//     IloNumVarArray control_points(env, n_points, 0.0, 200.0);

//     // Set start condition
//     IloExpr init_place_expr(env);
//     std::vector<double> init_bern_val;
//     bern_poly.CalculateVal(0, init_bern_val);
//     for (int i = 0; i < n_points; i++) {
//         init_place_expr += control_points[i]*init_bern_val[i];
//         // printf("Bernstein value is: %f", init_bern_val[i]);
//     }
//     model.add(init_place_expr == 0);
//     init_place_expr.end();

//     // Set end condition
//     IloExpr end_place_expr(env);
//     std::vector<double> end_bern_val;
//     bern_poly.CalculateVal(t_length, end_bern_val);
//     for (int i = 0; i < n_points; i++) {
//         end_place_expr += control_points[i]*end_bern_val[i];
//     }
//     int direction = vNameToDirection[result_nodes[traj_size-1].current_point];
//     // model.add(end_place_expr == (pairDistancesMap[result_nodes[0].current_point][result_nodes[trajectory.size()-1].current_point] + length + init_drift));
//     model.add(end_place_expr == (pairDistancesMap[result_nodes[0].current_point][result_nodes[traj_size-1].current_point] + 
//                                                                                     Li(direction, length) + init_drift));
//     end_place_expr.end();

//     // Vertex time window constraints
//     for (int i = 0; i < trajectory.size(); i++){
//         // arrival vertex time
//         IloExpr arr_place_expr(env);
//         std::vector<double> arr_bern_val;
//         bern_poly.CalculateVal(result_nodes[i].arrival_time_min - earliest_start_time, arr_bern_val);
//         for (int j = 0; j < n_points; j++) {
//             arr_place_expr += control_points[j]*arr_bern_val[j];
//         }
//         model.add(arr_place_expr <= pairDistancesMap[result_nodes[0].current_point][result_nodes[i].current_point] + init_drift);
//         arr_place_expr.end();

//         // leave vertex time
//         IloExpr leave_place_expr(env);
//         std::vector<double> leave_bern_val;
//         double arr_max = min(result_nodes[i].arrival_time_max - earliest_start_time, t_length);
//         bern_poly.CalculateVal(arr_max, leave_bern_val);
//         // std::cout << "For idx = " << i << ", time to leave is: " << result_nodes[i].arrival_time_max << std::endl;
//         for (int j = 0; j < n_points; j++) {
//             leave_place_expr += control_points[j]*leave_bern_val[j];
//             // std::cout << leave_bern_val[j] << ",\t";
//         }
//         int direction = vNameToDirection[result_nodes[i].current_point];
//         model.add(leave_place_expr >= (pairDistancesMap[result_nodes[0].current_point][result_nodes[i].current_point] + 
//                             Li(direction, length) + init_drift));
//         leave_place_expr.end();  
//         printf("For node %d, the arrive time is: %f, the leave time is: %f\n", i, result_nodes[i].arrival_time_min, arr_max+earliest_start_time);           
//     }

//     // // Add init speed condition
//     // double v_init = 0;
//     // IloExpr init_speed_expr(env);
//     // // For simplicity, we just use 2 here. For any order derivative, use pow(2, first_order_deriv.control_points_deriv.deriv_order) instead
//     // for (int j = 0; j < 2; j++) {
//     //     init_speed_expr += first_order_deriv.control_points_deriv.ratio * first_order_deriv.control_points_deriv.sign_idx[0][j] 
//     //                             * control_points[first_order_deriv.control_points_deriv.orignal_idx[0][j]];
//     // }
//     // model.add(init_speed_expr == v_init);
//     // init_speed_expr.end();

//     // Speed constraints for other conflict points
//     for (int i = 1; i <= first_order_deriv.control_points_deriv.N; i++) {
//         IloExpr speed_expr(env);
//         // For simplicity, we just use 2 here. For any order derivative, use pow(2, first_order_deriv.control_points_deriv.deriv_order) instead
//         for (int j = 0; j < 2; j++) {
//             speed_expr += first_order_deriv.control_points_deriv.ratio * first_order_deriv.control_points_deriv.sign_idx[i][j] 
//                                 * control_points[first_order_deriv.control_points_deriv.orignal_idx[i][j]];
//         }
//         model.add(speed_expr <= v_max);
//         model.add(speed_expr >= v_min);
//         speed_expr.end();
//     }

//     // Acceleration constraints
//     for (int i = 0; i <= second_order_deriv.control_points_deriv.N; i++) {
//         IloExpr acce_expr(env);
//         // For simplicity, we just use 4 here. For any order derivative, use pow(2, second_order_deriv.control_points_deriv.deriv_order) instead
//         for (int j = 0; j < 4; j++) {
//             acce_expr += second_order_deriv.control_points_deriv.ratio * second_order_deriv.control_points_deriv.sign_idx[i][j] 
//                                 * control_points[second_order_deriv.control_points_deriv.orignal_idx[i][j]];
//         }
//         model.add(acce_expr <= a_max);
//         model.add(acce_expr >= a_min);
//         acce_expr.end();
//     }

//     IloCplex cplex(model);
//     cplex.setOut(env.getNullStream());
//     cplex.setWarning(env.getNullStream());
//     cplex.setError(env.getNullStream());
//     // cplex.extract(model);
//     // cplex.exportModel("./output/tpg_model.lp");  

//     auto startTime = std::chrono::high_resolution_clock::now();
//     if(cplex.solve()) {
//         cplex.writeSolution("./output/tpg_solution");
//         auto stopTime = std::chrono::high_resolution_clock::now();
//         auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stopTime - startTime); 
//         std::cout << "[INFO] TPG solved with runtime: " << duration.count()/1000000.0 << std::endl;
//         total_runtime_ += duration.count()/1000000.0;

//         for (int i = 0; i < n_points; i++) {
//             solution_control_points.push_back(cplex.getValue(control_points[i]));
//             // std::cout << solution_control_points[i] << ", ";
//         }

//         Path potential_path;
//         for (int i =0; i < trajectory.size(); i++){
//             PathEntry a_path;
//             // Round all the time slot to x.1
//             int direction = vNameToDirection[result_nodes[i].current_point];
//             double STEP_SIZE = 0.05;
//             double block_start = floor((result_nodes[i].arrival_time_min)/STEP_SIZE) * STEP_SIZE  - earliest_start_time ;
//             while(bern_poly.GetVal(block_start, solution_control_points) - init_drift <= 
//                 pairDistancesMap[result_nodes[0].current_point][result_nodes[i].current_point]) {
//                 block_start += STEP_SIZE;
//             }
//             // block_start -= STEP_SIZE;
//             double block_end = block_start;
//             while(bern_poly.GetVal(block_end, solution_control_points) - init_drift <= 
//                 (pairDistancesMap[result_nodes[0].current_point][result_nodes[i].current_point])+Li(direction, length)) {
//                 block_end += STEP_SIZE;
//             }

//             a_path.conflict_point = result_nodes[i].current_point;
//             // Only need to modify this
//             a_path.arrival_time = block_start + earliest_start_time;
                
//             // Only need to modify this
//             a_path.leaving_time_tail = block_end + earliest_start_time;

//             potential_path.push_back(a_path);
//             printf("TGP: For node %d, the arrive time is: %f, the leave time is: %f\n", i, a_path.arrival_time, a_path.leaving_time_tail);           

//         }   

//         if (result_path.size() == 0){
//             copy(potential_path.begin(), potential_path.end(), back_inserter(result_path));
//         }
//         else{
//             if (potential_path.back().leaving_time_tail < result_path.back().leaving_time_tail){
//                 result_path.clear();
//                 copy(potential_path.begin(), potential_path.end(), back_inserter(result_path));
//             }
//         }
//         return true;
//     } else {
//         std::cout << "[INFO] TPG not solved!" << std::endl;
//         cplex.extract(model);
//         std::string model_name = "./output/tpg_model_agent_" + std::to_string(agentID) + std::to_string(count_called) + ".lp";
//         cplex.exportModel(model_name.c_str());
//         env.end();
//         return false;
//     }
    

//     // exit(-1);

//     // end of  own optimization ************************************************************************
// }


// bool SIPP::SolveSlackBezier(
//     int agentID,
//     double T,
//     std::vector<double>& solution_control_points,
//     Path& result_path,
//     double& slack_var)
// {
//     Agent agent = agents[agentID];
//     vector<int> trajectory = agent.trajectory;

//     // TODO: tune this parameters
//     double init_drift = 1;
//     int n_points = 30;

//     double a_min = agent.a_min;
//     double a_max = agent.a_max;
//     double v_min = agent.v_min;
//     double v_max = agent.v_max;
//     double length = agent.length;
//     double earliest_start_time = max(agent.earliest_start_time, result_nodes[0].arrival_time_min);
//     int traj_size = trajectory.size();
//     // double t_length = result_nodes[trajectory.size()-1].arrival_time_min;

//     std::cout << "[INFO] Solve for agent: " << agentID << std::endl;

//     double t_length = T - earliest_start_time;
//     std::cout << "Earliest_start_time is: " << earliest_start_time << std::endl;
//     std::cout << "T value is: " << t_length << std::endl;
//     // std::vector<double> solution_control_points;

//     BernsteinPolynomial bern_poly(n_points, t_length);
//     // First order derivative of bernstein polynomial
//     BernsteinPolynomial first_order_deriv = bern_poly.Derivative();
//     // Second order derivative of bernstein polynomial
//     BernsteinPolynomial second_order_deriv = first_order_deriv.Derivative();

//     IloEnv env = IloEnv();
//     IloModel model = IloModel(env);
//     IloExpr sum_obj = IloExpr(env);
//     // variable of controls points, TODO: add conditions
//     // n control points and 1 init time
//     IloNumVarArray control_points(env, n_points+1, 0, 2000);
//     sum_obj = control_points[n_points];
//     model.add(IloMinimize(env, sum_obj));
//     // Set start condition
//     IloExpr init_place_expr(env);
//     std::vector<double> init_bern_val;
//     bern_poly.CalculateVal(0, init_bern_val);
//     for (int i = 0; i < n_points; i++) {
//         init_place_expr += control_points[i]*init_bern_val[i];
//         // printf("Bernstein value is: %f", init_bern_val[i]);
//     }
//     model.add(init_place_expr == 0);
//     init_place_expr.end();

//     // Set end condition
//     IloExpr end_place_expr(env);
//     std::vector<double> end_bern_val;
//     bern_poly.CalculateVal(t_length, end_bern_val);
//     for (int i = 0; i < n_points; i++) {
//         end_place_expr += control_points[i]*end_bern_val[i];
//     }
//     int direction = vNameToDirection[result_nodes[traj_size-1].current_point];
//     // model.add(end_place_expr == (pairDistancesMap[result_nodes[0].current_point][result_nodes[trajectory.size()-1].current_point] + length + init_drift));
//     model.add(end_place_expr == (pairDistancesMap[result_nodes[0].current_point][result_nodes[traj_size-1].current_point] + 
//                                     Li(direction, length) + init_drift));
//     end_place_expr.end();

//     // Vertex time window constraints
//     for (int i = 0; i < trajectory.size(); i++){
//         // arrival vertex time
//         IloExpr arr_place_expr(env);
//         std::vector<double> arr_bern_val;
//         bern_poly.CalculateVal(result_nodes[i].arrival_time_min - earliest_start_time, arr_bern_val);
//         for (int j = 0; j < n_points; j++) {
//             arr_place_expr += control_points[j]*arr_bern_val[j];
//         }
//         model.add(arr_place_expr <= pairDistancesMap[result_nodes[0].current_point][result_nodes[i].current_point] + 
//                                         init_drift + control_points[n_points]);
//         arr_place_expr.end();

//         // leave vertex time
//         IloExpr leave_place_expr(env);
//         std::vector<double> leave_bern_val;
//         double arr_max = min(result_nodes[i].arrival_time_max - earliest_start_time, t_length);
//         bern_poly.CalculateVal(arr_max, leave_bern_val);
//         // std::cout << "For idx = " << i << ", time to leave is: " << result_nodes[i].arrival_time_max << std::endl;
//         for (int j = 0; j < n_points; j++) {
//             leave_place_expr += control_points[j]*leave_bern_val[j];
//             // std::cout << leave_bern_val[j] << ",\t";
//         }
//         int direction = vNameToDirection[result_nodes[i].current_point];
//         model.add(leave_place_expr >= (pairDistancesMap[result_nodes[0].current_point][result_nodes[i].current_point] + 
//                                         Li(direction, length) + init_drift - control_points[n_points]));
//         leave_place_expr.end();  
//         // printf("For node %d, the arrive time is: %f, the leave time is: %f\n", i, result_nodes[i].arrival_time_min, arr_max+earliest_start_time);           
//     }

//     // // Add init speed condition
//     // double v_init = 0;
//     // IloExpr init_speed_expr(env);
//     // // For simplicity, we just use 2 here. For any order derivative, use pow(2, first_order_deriv.control_points_deriv.deriv_order) instead
//     // for (int j = 0; j < 2; j++) {
//     //     init_speed_expr += first_order_deriv.control_points_deriv.ratio * first_order_deriv.control_points_deriv.sign_idx[0][j] 
//     //                             * control_points[first_order_deriv.control_points_deriv.orignal_idx[0][j]];
//     // }
//     // model.add(init_speed_expr == v_init);
//     // init_speed_expr.end();

//     // Speed constraints for other conflict points
//     for (int i = 1; i <= first_order_deriv.control_points_deriv.N; i++) {
//         IloExpr speed_expr(env);
//         // For simplicity, we just use 2 here. For any order derivative, use pow(2, first_order_deriv.control_points_deriv.deriv_order) instead
//         for (int j = 0; j < 2; j++) {
//             speed_expr += first_order_deriv.control_points_deriv.ratio * first_order_deriv.control_points_deriv.sign_idx[i][j] 
//                                 * control_points[first_order_deriv.control_points_deriv.orignal_idx[i][j]];
//         }
//         model.add(speed_expr <= v_max + control_points[n_points]);
//         model.add(speed_expr >= v_min - control_points[n_points]);
//         speed_expr.end();
//     }

//     // Acceleration constraints
//     for (int i = 0; i <= second_order_deriv.control_points_deriv.N; i++) {
//         IloExpr acce_expr(env);
//         // For simplicity, we just use 4 here. For any order derivative, use pow(2, second_order_deriv.control_points_deriv.deriv_order) instead
//         for (int j = 0; j < 4; j++) {
//             acce_expr += second_order_deriv.control_points_deriv.ratio * second_order_deriv.control_points_deriv.sign_idx[i][j] 
//                                 * control_points[second_order_deriv.control_points_deriv.orignal_idx[i][j]];
//         }
//         model.add(acce_expr <= a_max + control_points[n_points]);
//         model.add(acce_expr >= a_min - control_points[n_points]);
//         acce_expr.end();
//     }

//     IloCplex cplex(model);
//     cplex.setOut(env.getNullStream());
//     cplex.setWarning(env.getNullStream());
//     cplex.setError(env.getNullStream());
//     // cplex.extract(model);
//     // cplex.exportModel("./output/tpg_model.lp");  

//     auto startTime = std::chrono::high_resolution_clock::now();
//     if(cplex.solve()) {
//         cplex.writeSolution("./output/tpg_solution");
//         auto stopTime = std::chrono::high_resolution_clock::now();
//         auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stopTime - startTime); 
//         std::cout << "[INFO] TPG solved with runtime: " << duration.count()/1000000.0 << std::endl;
//         total_runtime_ += duration.count()/1000000.0;

//         // for (int i = 0; i < n_points; i++) {
//         //     solution_control_points.push_back(cplex.getValue(control_points[i]));
//         //     // std::cout << solution_control_points[i] << ", ";
//         // }
//         slack_var = cplex.getValue(control_points[n_points]);
//         std::cout << "The slack variable is: " << slack_var << std::endl;
//         return true;
//     } else {
//         std::cout << "[INFO] TPG not solved!" << std::endl;
//         cplex.extract(model);
//         std::string model_name = "./output/tpg_model_agent_" + std::to_string(agentID) + std::to_string(count_called) + ".lp";
//         cplex.exportModel(model_name.c_str());
//         env.end();
//         return false;
//     }
// }

// double SIPP::SlackGradient(
//     int agentID,
//     double target_t)
// {
//     double delta = 0.1;
//     std::vector<double> solution_control_points;
//     Path result_path;
//     double slack_t;
//     SolveSlackBezier(agentID, target_t, solution_control_points, result_path, slack_t);
//     double slack_delta;
//     SolveSlackBezier(agentID, target_t+delta, solution_control_points, result_path, slack_delta);
//     double gradient = (slack_delta - slack_t)/delta;
//     printf("Gradient at time %f is: %f\n", target_t, gradient);
//     return gradient;
// }

// bool SIPP::RecurrentSolver(
//     int agentID,
//     double& optimal_T,
//     double& T_length,
//     double& earliest_start_time,
//     std::vector<double>& solution_control_points,
//     Path& result_path)
// {
//     Agent agent = agents[agentID];
//     vector<int> trajectory = agent.trajectory;
//     double lower_bound = result_nodes[trajectory.size()-1].arrival_time_min;
//     double upper_bound = result_nodes[trajectory.size()-1].arrival_time_max;
//     // If the bound is too tight
//     if ((upper_bound-lower_bound) > 10) {
//         double milp_end_time = 0;
//         bool milp_success = SolveMILP(agentID, milp_end_time);
//         if (milp_success){
//             upper_bound = milp_end_time;
//         }
//         std::cout << "milp end time: " << upper_bound << std::endl;
//     }
//     // printf("[INFO] bounds for T: (%f, %f)\n", lower_bound, upper_bound);

//     double mid;
//     double T_thresh = 0.1;
//     double Max_it_thresh = 0.05;
//     double gradient_t_high = SlackGradient(agentID, upper_bound);
//     double gradient_t_low = SlackGradient(agentID, lower_bound);
//     bool t_high_valid = false;
//     if (gradient_t_low == 0) {
//         // TO DO: add the solution to from the solver
//         optimal_T = lower_bound;
//     } else{
//         if (gradient_t_high == 0) {
//             optimal_T = upper_bound;
//             t_high_valid = true;
//         }
//         if (gradient_t_low * gradient_t_high > 0) {
//             std::cout << "No valid solution in the bound!\n";
//             return false;
//         } else {
//             while ((upper_bound - lower_bound) >= Max_it_thresh) {
//                 printf("The upper bound is %f, the lower bound is %f\n", upper_bound, lower_bound);
//                 mid = (upper_bound + lower_bound)/2;
//                 double slack;
//                 SolveSlackBezier(agentID, mid, solution_control_points, result_path, slack);
//                 if (slack == 0) {
//                     upper_bound = mid;
//                     optimal_T = mid;
//                     t_high_valid = true;
//                 } else {
//                     if (t_high_valid){
//                         lower_bound = mid;
//                     } else {
//                         double grad_mid = SlackGradient(agentID, mid);
//                         if (grad_mid > 0) {
//                             upper_bound = mid;
//                         } else {
//                             lower_bound = mid;
//                         }
//                     }
//                 }
//                 if ((upper_bound - lower_bound) <= T_thresh && t_high_valid) {
//                     break;
//                 }
//             }
//         }
//     }
//     if (!t_high_valid) {
//         return false;
//     }
//     solution_control_points.clear();
//     std::cout << "The solved optimal T is: " << optimal_T << std::endl;
//     bool success = SolveBezier(agentID, optimal_T, T_length, earliest_start_time, solution_control_points, result_path);
//     if (success){
//         std::cout << "All control points: ";
//         for (double point: solution_control_points){
//             std::cout << point << ";";
//         }
//         std::cout << std::endl;
//         return true;
//     } else {
//         printf("Unable to solve for the this SIPP\n");
//         return false;
//     }
// }


// bool SIPP::SolveMILP(double& speed, Path& potential_path)
// {
//     double v_min = curr_agent.v_min;
//     double v_max = curr_agent.v_max;
//     double length = curr_agent.length; // length of the vehicle

//     IloEnv env = IloEnv();
//     IloModel model = IloModel(env);
//     IloExpr sum_obj = IloExpr(env);
//     IloNumVarArray var(env);
//     IloRangeArray con(env);
//     bool flag = true;

//     //speed
//     var.add(IloNumVar(env, 1/v_max,1/v_min, ILOFLOAT));  //t
//     //start time
//     // var.add(IloNumVar(env, 0, result_nodes[0].interval_t_max, ILOFLOAT));  //t

//     sum_obj = var[0];

//     model.add(IloMinimize(env, sum_obj));
//     double total_dist = 0;
//     for (unsigned int i = 0; i < result_nodes.size(); i++){
//         IloExpr node_arrival_time_min = total_dist*var[0] + curr_agent.earliest_start_time;
//         IloExpr node_leaving_time_tail = node_arrival_time_min + length/w + length*var[0];
//         if ( i > 0){
//             con.add(node_arrival_time_min - result_nodes[i]->arrival_time_min  >= 0);
//         }
//         con.add(node_leaving_time_tail - result_nodes[i]->arrival_time_max  <= 0);    
//         total_dist += CELL_DIS;
//         printf("Interval for location: %d is: %f -> %f\n", result_nodes[i]->current_point,
//             result_nodes[i]->arrival_time_min, result_nodes[i]->arrival_time_max);            
//     }   
//     model.add(con);

//     IloCplex cplex(model);
//     cplex.setOut(env.getNullStream());
//     cplex.setWarning(env.getNullStream());
//     cplex.setError(env.getNullStream());
//     // cplex.extract(model);
//     // cplex.exportModel("./output/model.lp");
//     sum_obj.end();
//     // auto startTime = std::chrono::high_resolution_clock::now();
//     if (cplex.solve()){
//         // auto stopTime = std::chrono::high_resolution_clock::now();
//         // auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stopTime - startTime); 
//         // std::cout << "[INFO] MILP solved with runtime: " << duration.count()/1000000.0 << std::endl;
//         // printf("Speed is: %f\n", 1/cplex.getValue(var[0]));
//         // Path potential_path;
//         // cplex.writeSolution("./output/solution");
//         double total_dist = 0;
//         // printf("Start location: (%d, %d)\n", instance_ptr->getRowCoordinate(curr_agent.start_location),
//         //          instance_ptr->getColCoordinate(curr_agent.start_location));
//         for (unsigned int i = 0; i < result_nodes.size(); i++){
//             PathEntry a_path;
//             a_path.location = result_nodes[i]->current_point;
//             // Only need to modify this
//             a_path.arrival_time = total_dist*cplex.getValue(var[0]) + curr_agent.earliest_start_time;
//             // a_path.arrival_time = 0;
//             a_path.leaving_time_tail = a_path.arrival_time + length/w + length*cplex.getValue(var[0]);
//             potential_path.push_back(a_path);
//             printf("At location %d: the the arrive time is: %f, the leave time is: %f\n", 
//                 a_path.location, a_path.arrival_time, a_path.leaving_time_tail
//             );
//             total_dist += CELL_DIS;
//         }        
//         // printf("Goal location: (%d, %d)\n", instance_ptr->getRowCoordinate(curr_agent.goal_location),
//         //          instance_ptr->getColCoordinate(curr_agent.goal_location));

//         // if (result_path.size() == 0){
//         //     copy(potential_path.begin(), potential_path.end(), back_inserter(result_path));
//         // }
//         // else{
//         //     if (potential_path.back().leaving_time_tail < result_path.back().leaving_time_tail){
//         //         result_path.clear();
//         //         copy(potential_path.begin(), potential_path.end(), back_inserter(result_path));
//         //     }
//         // }
//         flag = true;
//     } else {
//         // std::cout << "[INFO] MILP Not solved" << std::endl;
//         flag = false;
//     }
//     env.end();
//     return flag;
// }
