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