#include "../include/SIPP.h"
#include "milp_cache.h"
#include <cstdio>
#include <memory>
#include <string>
#include <utility>
#include <vector>


using namespace std;

SIPP::SIPP(std::shared_ptr<Instance> instance)
{
    instance_ptr = instance;
    agents = instance_ptr->agents;
    pairDistancesMap = instance_ptr->getPairDistancesMap();
    vNameToDirection = instance_ptr->getVNameToDirection();
    vNameToID = instance_ptr->getVNameToID();    
}

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
//     var.add(IloNumVar(env, 1/v_max,1/v_min, ILOFLOAT));  //t
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
//             milp_end_time = a_path.leaving_time_tail;
//             // a_path.leaving_time_tail = 1;
//             potential_path.push_back(a_path);
//             // printf("MILP: For node %d, the arrive time is: %f, the leave time is: %f\n", i, a_path.arrival_time, a_path.leaving_time_tail);           

//         }                
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

bool SIPP::SolveMILP(int agentID, double& milp_end_time)
{
    Agent agent = agents[agentID];
    vector<int> trajectory = agent.trajectory;
    double v_min = agent.v_min;
    double length = agent.length; // length of the vehicle

    IloEnv env = IloEnv();
    IloModel model = IloModel(env);
    IloExpr sum_obj = IloExpr(env);
    IloNumVarArray var(env);
    IloRangeArray con(env);
    bool flag = true;

    //speed
    // var.add(IloNumVar(env, 1/v_max,1/v_min, ILOFLOAT));  //t
    var.add(IloNumVar(env, 1/v_min -0.001,1/v_min, ILOFLOAT));  //t
    //start time
    var.add(IloNumVar(env, result_nodes[0].interval_t_min, result_nodes[0].interval_t_max, ILOFLOAT));  //t

    int final_conflict_point_direction = vNameToDirection[result_nodes.back().current_point];
    sum_obj = var[1] + pairDistancesMap[result_nodes[0].current_point][result_nodes.back().current_point]*var[0] + Li(final_conflict_point_direction, length)/w + Li(final_conflict_point_direction, length)*var[0];

    model.add(IloMinimize(env, sum_obj));
    for (int i =0; i < trajectory.size(); i++){
        IloExpr node_arrival_time_min = var[1] + pairDistancesMap[result_nodes[0].current_point][result_nodes[i].current_point]*var[0];

        int direction = vNameToDirection[result_nodes[i].current_point]; 
        IloExpr node_leaving_time_tail = node_arrival_time_min + Li(direction, length)/w + Li(direction, length)*var[0];

        if ( i > 0){
            con.add(node_arrival_time_min - result_nodes[i].arrival_time_min  >=0);

        }
        con.add(node_leaving_time_tail - result_nodes[i].arrival_time_max  <=0);                
    }   
    model.add(con);

    IloCplex cplex(model);
    cplex.setOut(env.getNullStream());
    cplex.setWarning(env.getNullStream());
    cplex.setError(env.getNullStream());
    // cplex.extract(model);
    // cplex.exportModel("./output/model.lp");
    sum_obj.end();
    // auto startTime = std::chrono::high_resolution_clock::now();
    
    milp_end_time = 0;
    if (cplex.solve()){
        // auto stopTime = std::chrono::high_resolution_clock::now();
        // auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stopTime - startTime); 
        // std::cout << "[INFO] MILP solved with runtime: " << duration.count()/1000000.0 << std::endl;
        // printf("Speed is: %f, and init time is: %f\n", cplex.getValue(var[0]), cplex.getValue(var[1]));
        Path potential_path;
        // cplex.writeSolution("./output/solution");
        for (int i =0; i < trajectory.size(); i++){
            PathEntry a_path;
            a_path.conflict_point = result_nodes[i].current_point;
            // Only need to modify this
            a_path.arrival_time = cplex.getValue(var[1]) + pairDistancesMap[result_nodes[0].current_point][result_nodes[i].current_point]*cplex.getValue(var[0]);
            // a_path.arrival_time = 0;
            

            int direction = vNameToDirection[result_nodes[i].current_point]; 
            
            // Only need to modify this
            a_path.leaving_time_tail = a_path.arrival_time + Li(direction, length)/w + Li(direction, length)*cplex.getValue(var[0]);

            potential_path.push_back(a_path);
        }     
        milp_end_time = potential_path[trajectory.size()-1].leaving_time_tail;    
        // printf("The solution for MILP is %f.\n", milp_end_time);           

        // if (result_path.size() == 0){
        //     copy(potential_path.begin(), potential_path.end(), back_inserter(result_path));
        // }
        // else{
        //     if (potential_path.back().leaving_time_tail < result_path.back().leaving_time_tail){
        //         result_path.clear();
        //         copy(potential_path.begin(), potential_path.end(), back_inserter(result_path));
        //     }
        // }
        flag = true;
    } else {
        // std::cout << "[INFO] MILP Not solved" << std::endl;
        flag = false;
    }
    env.end();
    return flag;
}

bool SIPP::SolveBezier(
    int agentID,
    double T_optimal,
    double& t_length,
    double& earliest_start_time,
    std::vector<double>& solution_control_points,
    Path& result_path)
{
    // start own optimization ************************************************************************
    Agent agent = agents[agentID];
    vector<int> trajectory = agent.trajectory;

    // TODO: tune this parameters
    double init_drift = 1;
    int n_points = 30;

    double a_min = agent.a_min;
    double a_max = agent.a_max;
    double v_min = agent.v_min;
    double v_max = agent.v_max;
    double length = agent.length;
    earliest_start_time = max(agent.earliest_start_time, result_nodes[0].arrival_time_min);
    
    int traj_size = trajectory.size();
    // double t_length = result_nodes[trajectory.size()-1].arrival_time_min;
    // double milp_end_time = 0;
    // bool milp_success = SolveMILP(agentID, milp_end_time);
    // if (milp_success){
    //     t_length = milp_end_time - earliest_start_time;
    // } else{
    //     t_length = result_nodes[trajectory.size()-1].arrival_time_max - earliest_start_time;
    // }
    t_length = T_optimal - earliest_start_time;
    DEBUG_PRINT(printf("Earliest_start_time is: %f\n", earliest_start_time);)
    DEBUG_PRINT(printf("T value is: %f\n", t_length);)
    // std::vector<double> solution_control_points;

    BernsteinPolynomial bern_poly(n_points, t_length);
    // First order derivative of bernstein polynomial
    BernsteinPolynomial first_order_deriv = bern_poly.Derivative();
    // Second order derivative of bernstein polynomial
    BernsteinPolynomial second_order_deriv = first_order_deriv.Derivative();

    IloEnv env = IloEnv();
    IloModel model = IloModel(env);
    // variable of controls points, TODO: add conditions
    // n control points and 1 init time
    IloNumVarArray control_points(env, n_points, 0.0, 200.0);

    // Set start condition
    IloExpr init_place_expr(env);
    std::vector<double> init_bern_val;
    bern_poly.CalculateVal(0, init_bern_val);
    for (int i = 0; i < n_points; i++) {
        init_place_expr += control_points[i]*init_bern_val[i];
        // printf("Bernstein value is: %f", init_bern_val[i]);
    }
    model.add(init_place_expr == 0);
    init_place_expr.end();

    // Set end condition
    IloExpr end_place_expr(env);
    std::vector<double> end_bern_val;
    bern_poly.CalculateVal(t_length, end_bern_val);
    for (int i = 0; i < n_points; i++) {
        end_place_expr += control_points[i]*end_bern_val[i];
    }
    int direction = vNameToDirection[result_nodes[traj_size-1].current_point];
    // model.add(end_place_expr == (pairDistancesMap[result_nodes[0].current_point][result_nodes[trajectory.size()-1].current_point] + length + init_drift));
    model.add(end_place_expr == (pairDistancesMap[result_nodes[0].current_point][result_nodes[traj_size-1].current_point] + 
                                                                                    Li(direction, length) + init_drift));
    end_place_expr.end();

    // Vertex time window constraints
    for (int i = 0; i < trajectory.size(); i++){
        // arrival vertex time
        IloExpr arr_place_expr(env);
        std::vector<double> arr_bern_val;
        bern_poly.CalculateVal(result_nodes[i].arrival_time_min - earliest_start_time, arr_bern_val);
        for (int j = 0; j < n_points; j++) {
            arr_place_expr += control_points[j]*arr_bern_val[j];
        }
        model.add(arr_place_expr <= pairDistancesMap[result_nodes[0].current_point][result_nodes[i].current_point] + init_drift);
        arr_place_expr.end();

        // leave vertex time
        IloExpr leave_place_expr(env);
        std::vector<double> leave_bern_val;
        double arr_max = min(result_nodes[i].arrival_time_max - earliest_start_time, t_length);
        // printf("Arr_max is :%f\n", arr_max);
        bern_poly.CalculateVal(arr_max, leave_bern_val);
        // std::cout << "For idx = " << i << ", time to leave is: " << result_nodes[i].arrival_time_max << std::endl;
        for (int j = 0; j < n_points; j++) {
            leave_place_expr += control_points[j]*leave_bern_val[j];
            // std::cout << leave_bern_val[j] << ",\t";
        }
        // std::cout << std::endl;
        int direction = vNameToDirection[result_nodes[i].current_point];
        model.add(leave_place_expr >= (pairDistancesMap[result_nodes[0].current_point][result_nodes[i].current_point] + 
                            Li(direction, length) + init_drift));
        leave_place_expr.end();  
        // printf("For node %d, the arrive time is: %f, the leave time is: %f\n", i, result_nodes[i].arrival_time_min, arr_max+earliest_start_time);           
    }

    // // Add init speed condition
    // double v_init = 0;
    // IloExpr init_speed_expr(env);
    // // For simplicity, we just use 2 here. For any order derivative, use pow(2, first_order_deriv.control_points_deriv.deriv_order) instead
    // for (int j = 0; j < 2; j++) {
    //     init_speed_expr += first_order_deriv.control_points_deriv.ratio * first_order_deriv.control_points_deriv.sign_idx[0][j] 
    //                             * control_points[first_order_deriv.control_points_deriv.orignal_idx[0][j]];
    // }
    // model.add(init_speed_expr == v_init);
    // init_speed_expr.end();

    // Speed constraints for other conflict points
    for (int i = 1; i <= first_order_deriv.control_points_deriv.N; i++) {
        IloExpr speed_expr(env);
        // For simplicity, we just use 2 here. For any order derivative, use pow(2, first_order_deriv.control_points_deriv.deriv_order) instead
        for (int j = 0; j < 2; j++) {
            speed_expr += first_order_deriv.control_points_deriv.ratio * first_order_deriv.control_points_deriv.sign_idx[i][j] 
                                * control_points[first_order_deriv.control_points_deriv.orignal_idx[i][j]];
        }
        model.add(speed_expr <= v_max);
        model.add(speed_expr >= v_min);
        speed_expr.end();
    }

    // Acceleration constraints
    for (int i = 0; i <= second_order_deriv.control_points_deriv.N; i++) {
        IloExpr acce_expr(env);
        // For simplicity, we just use 4 here. For any order derivative, use pow(2, second_order_deriv.control_points_deriv.deriv_order) instead
        for (int j = 0; j < 4; j++) {
            acce_expr += second_order_deriv.control_points_deriv.ratio * second_order_deriv.control_points_deriv.sign_idx[i][j] 
                                * control_points[second_order_deriv.control_points_deriv.orignal_idx[i][j]];
        }
        model.add(acce_expr <= a_max);
        model.add(acce_expr >= a_min);
        acce_expr.end();
    }

    IloCplex cplex(model);
    cplex.setOut(env.getNullStream());
    cplex.setWarning(env.getNullStream());
    cplex.setError(env.getNullStream());
    // cplex.extract(model);
    // cplex.exportModel("./output/tpg_model.lp");  

    auto startTime = std::chrono::high_resolution_clock::now();
    if(cplex.solve()) {
        // cplex.writeSolution("./output/tpg_solution");
        auto stopTime = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stopTime - startTime); 
        // std::cout << "[INFO] TPG solved with runtime: " << duration.count()/1000000.0 << std::endl;
        total_runtime_ += duration.count()/1000000.0;

        for (int i = 0; i < n_points; i++) {
            solution_control_points.push_back(cplex.getValue(control_points[i]));
            // std::cout << solution_control_points[i] << ", ";
        }

        Path potential_path;
        for (int i =0; i < trajectory.size(); i++){
            PathEntry a_path;
            // Round all the time slot to x.1
            int direction = vNameToDirection[result_nodes[i].current_point];
            double STEP_SIZE = 0.05;
            double block_start = floor((result_nodes[i].arrival_time_min)/STEP_SIZE) * STEP_SIZE  - earliest_start_time;
            while(bern_poly.GetVal(block_start, solution_control_points) - init_drift <= 
                pairDistancesMap[result_nodes[0].current_point][result_nodes[i].current_point]
                or
                abs(bern_poly.GetVal(block_start, solution_control_points) - init_drift -
                pairDistancesMap[result_nodes[0].current_point][result_nodes[i].current_point]) < EPS) {
                block_start += STEP_SIZE;
            }
            block_start -= STEP_SIZE;
            // printf("Prev start dist: %f, start start dist: %f, compare dist: %f\n", 
            //     bern_poly.GetVal(block_start + STEP_SIZE, solution_control_points) - init_drift, 
            //     bern_poly.GetVal(block_start, solution_control_points) - init_drift, 
            //     pairDistancesMap[result_nodes[0].current_point][result_nodes[i].current_point]);
            double block_end = block_start;
            
            while((bern_poly.GetVal(block_end, solution_control_points) - init_drift <
                pairDistancesMap[result_nodes[0].current_point][result_nodes[i].current_point] + Li(direction, length))
                and 
                abs(bern_poly.GetVal(block_end, solution_control_points) - init_drift -
                pairDistancesMap[result_nodes[0].current_point][result_nodes[i].current_point] - Li(direction, length)) > EPS) {
                block_end += STEP_SIZE;
            }
            // printf("Prev end dist: %f, end start dist: %f, compare dist: %f\n", 
            //     bern_poly.GetVal(block_end - STEP_SIZE, solution_control_points) - init_drift, 
            //     bern_poly.GetVal(block_end, solution_control_points) - init_drift, 
            //     pairDistancesMap[result_nodes[0].current_point][result_nodes[i].current_point] + Li(direction, length));

            a_path.conflict_point = result_nodes[i].current_point;
            // Only need to modify this
            a_path.arrival_time = block_start + earliest_start_time;
                
            // Only need to modify this
            a_path.leaving_time_tail = block_end + earliest_start_time;

            potential_path.push_back(a_path);
            // printf("TGP: For node %d, the arrive time is: %f, the leave time is: %f\n\n", i, a_path.arrival_time, a_path.leaving_time_tail);           

        }   

        if (result_path.size() == 0){
            copy(potential_path.begin(), potential_path.end(), back_inserter(result_path));
        }
        else{
            if (potential_path.back().leaving_time_tail < result_path.back().leaving_time_tail){
                result_path.clear();
                copy(potential_path.begin(), potential_path.end(), back_inserter(result_path));
            }
        }
        env.end();
        return true;
    } else {
        // std::cout << "[INFO] TPG not solved!" << std::endl;
        // cplex.extract(model);
        // std::string model_name = "./output/tpg_model_agent_" + std::to_string(agentID) + std::to_string(count_called) + ".lp";
        // cplex.exportModel(model_name.c_str());
        env.end();
        return false;
    }
    

    // exit(-1);

    // end of  own optimization ************************************************************************
}


bool SIPP::SolveSlackBezier(
    int agentID,
    double T,
    std::vector<double>& solution_control_points,
    Path& result_path,
    double& slack_var)
{
    Agent agent = agents[agentID];
    vector<int> trajectory = agent.trajectory;

    // TODO: tune this parameters
    double init_drift = 1;
    int n_points = 30;

    double a_min = agent.a_min;
    double a_max = agent.a_max;
    double v_min = agent.v_min;
    double v_max = agent.v_max;
    double length = agent.length;
    double earliest_start_time = max(agent.earliest_start_time, result_nodes[0].arrival_time_min);
    int traj_size = trajectory.size();
    // double t_length = result_nodes[trajectory.size()-1].arrival_time_min;

    DEBUG_PRINT(printf("[INFO] Solve for agent: %d\n", agentID);)

    double t_length = T - earliest_start_time;
    DEBUG_PRINT(printf("Earliest_start_time is: %f\n", earliest_start_time);)
    DEBUG_PRINT(printf("T value is: %f\n", t_length);)

    BernsteinPolynomial bern_poly(n_points, t_length);
    // First order derivative of bernstein polynomial
    BernsteinPolynomial first_order_deriv = bern_poly.Derivative();
    // Second order derivative of bernstein polynomial
    BernsteinPolynomial second_order_deriv = first_order_deriv.Derivative();

    IloEnv env = IloEnv();
    IloModel model = IloModel(env);
    IloExpr sum_obj = IloExpr(env);
    // variable of controls points, TODO: add conditions
    // n control points and 1 init time
    IloNumVarArray control_points(env, n_points+1, 0, 2000);
    sum_obj = control_points[n_points];
    model.add(IloMinimize(env, sum_obj));
    // Set start condition
    IloExpr init_place_expr(env);
    std::vector<double> init_bern_val;
    bern_poly.CalculateVal(0, init_bern_val);
    for (int i = 0; i < n_points; i++) {
        init_place_expr += control_points[i]*init_bern_val[i];
        // printf("Bernstein value is: %f", init_bern_val[i]);
    }
    model.add(init_place_expr == 0);
    init_place_expr.end();

    // Set end condition
    IloExpr end_place_expr(env);
    std::vector<double> end_bern_val;
    bern_poly.CalculateVal(t_length, end_bern_val);
    for (int i = 0; i < n_points; i++) {
        end_place_expr += control_points[i]*end_bern_val[i];
    }
    int direction = vNameToDirection[result_nodes[traj_size-1].current_point];
    // model.add(end_place_expr == (pairDistancesMap[result_nodes[0].current_point][result_nodes[trajectory.size()-1].current_point] + length + init_drift));
    model.add(end_place_expr == (pairDistancesMap[result_nodes[0].current_point][result_nodes[traj_size-1].current_point] + 
                                    Li(direction, length) + init_drift));
    end_place_expr.end();

    // Vertex time window constraints
    for (int i = 0; i < trajectory.size(); i++){
        // arrival vertex time
        IloExpr arr_place_expr(env);
        std::vector<double> arr_bern_val;
        bern_poly.CalculateVal(result_nodes[i].arrival_time_min - earliest_start_time, arr_bern_val);
        for (int j = 0; j < n_points; j++) {
            arr_place_expr += control_points[j]*arr_bern_val[j];
        }
        model.add(arr_place_expr <= pairDistancesMap[result_nodes[0].current_point][result_nodes[i].current_point] + 
                                        init_drift + control_points[n_points]);
        arr_place_expr.end();

        // leave vertex time
        IloExpr leave_place_expr(env);
        std::vector<double> leave_bern_val;
        double arr_max = min(result_nodes[i].arrival_time_max - earliest_start_time, t_length);
        bern_poly.CalculateVal(arr_max, leave_bern_val);
        // std::cout << "For idx = " << i << ", time to leave is: " << result_nodes[i].arrival_time_max << std::endl;
        for (int j = 0; j < n_points; j++) {
            leave_place_expr += control_points[j]*leave_bern_val[j];
            // std::cout << leave_bern_val[j] << ",\t";
        }
        int direction = vNameToDirection[result_nodes[i].current_point];
        model.add(leave_place_expr >= (pairDistancesMap[result_nodes[0].current_point][result_nodes[i].current_point] + 
                                        Li(direction, length) + init_drift - control_points[n_points]));
        leave_place_expr.end();  
        // printf("For node %d, the arrive time is: %f, the leave time is: %f\n", i, result_nodes[i].arrival_time_min, arr_max+earliest_start_time);           
    }

    // // Add init speed condition
    // double v_init = 0;
    // IloExpr init_speed_expr(env);
    // // For simplicity, we just use 2 here. For any order derivative, use pow(2, first_order_deriv.control_points_deriv.deriv_order) instead
    // for (int j = 0; j < 2; j++) {
    //     init_speed_expr += first_order_deriv.control_points_deriv.ratio * first_order_deriv.control_points_deriv.sign_idx[0][j] 
    //                             * control_points[first_order_deriv.control_points_deriv.orignal_idx[0][j]];
    // }
    // model.add(init_speed_expr == v_init);
    // init_speed_expr.end();

    // Speed constraints for other conflict points
    for (int i = 1; i <= first_order_deriv.control_points_deriv.N; i++) {
        IloExpr speed_expr(env);
        // For simplicity, we just use 2 here. For any order derivative, use pow(2, first_order_deriv.control_points_deriv.deriv_order) instead
        for (int j = 0; j < 2; j++) {
            speed_expr += first_order_deriv.control_points_deriv.ratio * first_order_deriv.control_points_deriv.sign_idx[i][j] 
                                * control_points[first_order_deriv.control_points_deriv.orignal_idx[i][j]];
        }
        model.add(speed_expr <= v_max + control_points[n_points]);
        model.add(speed_expr >= v_min - control_points[n_points]);
        speed_expr.end();
    }

    // Acceleration constraints
    for (int i = 0; i <= second_order_deriv.control_points_deriv.N; i++) {
        IloExpr acce_expr(env);
        // For simplicity, we just use 4 here. For any order derivative, use pow(2, second_order_deriv.control_points_deriv.deriv_order) instead
        for (int j = 0; j < 4; j++) {
            acce_expr += second_order_deriv.control_points_deriv.ratio * second_order_deriv.control_points_deriv.sign_idx[i][j] 
                                * control_points[second_order_deriv.control_points_deriv.orignal_idx[i][j]];
        }
        model.add(acce_expr <= a_max + control_points[n_points]);
        model.add(acce_expr >= a_min - control_points[n_points]);
        acce_expr.end();
    }

    IloCplex cplex(model);
    cplex.setOut(env.getNullStream());
    cplex.setWarning(env.getNullStream());
    cplex.setError(env.getNullStream());
    // cplex.extract(model);
    // cplex.exportModel("./output/tpg_model.lp");  

    auto startTime = std::chrono::high_resolution_clock::now();
    if(cplex.solve()) {
        // cplex.writeSolution("./output/tpg_solution");
        auto stopTime = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stopTime - startTime); 
        // std::cout << "[INFO] TPG solved with runtime: " << duration.count()/1000000.0 << std::endl;
        total_runtime_ += duration.count()/1000000.0;

        // for (int i = 0; i < n_points; i++) {
        //     solution_control_points.push_back(cplex.getValue(control_points[i]));
        //     // std::cout << solution_control_points[i] << ", ";
        // }
        slack_var = cplex.getValue(control_points[n_points]);
        DEBUG_PRINT(printf("The slack variable is: %f\n", slack_var);)
        env.end();
        return true;
    } else {
        DEBUG_PRINT(printf("[INFO] TPG not solved!\n");)
        // cplex.extract(model);
        // std::string model_name = "./output/tpg_model_agent_" + std::to_string(agentID) + std::to_string(count_called) + ".lp";
        // cplex.exportModel(model_name.c_str());
        env.end();
        return false;
    }
}

double SIPP::SlackGradient(
    int agentID,
    double target_t)
{
    double delta = 0.1;
    std::vector<double> solution_control_points;
    Path result_path;
    double slack_t;
    SolveSlackBezier(agentID, target_t, solution_control_points, result_path, slack_t);
    double slack_delta;
    SolveSlackBezier(agentID, target_t+delta, solution_control_points, result_path, slack_delta);
    double gradient = (slack_delta - slack_t)/delta;
    DEBUG_PRINT(printf("Gradient at time %f is: %f\n", target_t, gradient);)
    return gradient;
}

bool SIPP::RecurrentSolver(
    int agentID,
    double& optimal_T,
    double& T_length,
    double& earliest_start_time,
    std::vector<double>& solution_control_points,
    Path& result_path)
{
    Agent agent = agents[agentID];
    vector<int> trajectory = agent.trajectory;
    double lower_bound = result_nodes[trajectory.size()-1].arrival_time_min;
    double upper_bound = result_nodes[trajectory.size()-1].arrival_time_max;
    // If the bound is too tight
    if ((upper_bound-lower_bound) > 10) {
        double milp_end_time = 0;
        bool milp_success = SolveMILP(agentID, milp_end_time);
        if (milp_success){
            upper_bound = milp_end_time;
        }
        DEBUG_PRINT(printf("milp end time: %f\n", upper_bound);)
    }
    // printf("[INFO] bounds for T: (%f, %f)\n", lower_bound, upper_bound);

    double mid;
    double T_thresh = 0.1;
    double Max_it_thresh = 0.05;
    double gradient_t_high = SlackGradient(agentID, upper_bound);
    double gradient_t_low = SlackGradient(agentID, lower_bound);
    bool t_high_valid = false;
    if (gradient_t_low == 0) {
        // TO DO: add the solution to from the solver
        optimal_T = lower_bound;
    } else{
        if (gradient_t_high == 0) {
            optimal_T = upper_bound;
            t_high_valid = true;
        }
        if (gradient_t_low * gradient_t_high > 0) {
            DEBUG_PRINT(printf("No valid solution in the bound!\n");)
            return false;
        } else {
            while ((upper_bound - lower_bound) >= Max_it_thresh) {
                DEBUG_PRINT(printf("The upper bound is %f, the lower bound is %f\n", upper_bound, lower_bound);)
                mid = (upper_bound + lower_bound)/2;
                double slack;
                SolveSlackBezier(agentID, mid, solution_control_points, result_path, slack);
                if (slack == 0) {
                    upper_bound = mid;
                    optimal_T = mid;
                    t_high_valid = true;
                } else {
                    if (t_high_valid){
                        lower_bound = mid;
                    } else {
                        double grad_mid = SlackGradient(agentID, mid);
                        if (grad_mid > 0) {
                            upper_bound = mid;
                        } else {
                            lower_bound = mid;
                        }
                    }
                }
                if ((upper_bound - lower_bound) <= T_thresh && t_high_valid) {
                    break;
                }
            }
        }
    }
    if (!t_high_valid) {
        return false;
    }
    solution_control_points.clear();
    DEBUG_PRINT(printf("The solved optimal T is: %f\n", optimal_T);)
    bool success = SolveBezier(agentID, optimal_T, T_length, earliest_start_time, solution_control_points, result_path);
    if (success){
        // std::cout << "All control points: ";
        // for (double point: solution_control_points){
        //     std::cout << point << ";";
        // }
        // std::cout << std::endl;
        return true;
    } else {
        DEBUG_PRINT(printf("Unable to solve for the this SIPP\n");)
        return false;
    }
}

Path SIPP::run(
    int agentID, 
    const ReservationTable& rt,
    std::shared_ptr<CacheMILP> cache_ptr, 
    double& optimal_T, 
    double& start_time,
    std::vector<double>& solution_control_points)
{
    count_called++;
    // std::cout << "Number of sipp called " << count_called << std::endl;
    // // from agent's ID -> get agent's start_location, goal location, earliest_start_time, maximum velocity
    
    // agent details
    Agent agent = agents[agentID];
    int id = agent.id;
    vector<int> trajectory = agent.trajectory;
    std::vector<double> tmp_control_points;
    std::vector<double> min_control_points;
    double tmp_T;
    double min_T = 999999999.0;
    double tmp_T_length = 0;
    double min_T_length = 0;

    double earliest_start_time = agent.earliest_start_time;
    double v_min = agent.v_min;
    double v_max = agent.v_max;
    double length = agent.length; // length of the vehicle

    vector<Node> open;
    // vector<Node> result_nodes;
    vector<Node> p[trajectory.size()];
    vector<Node> node_storage[trajectory.size()];


    list<TimeInterval> safe_intervals = getSafeIntervals( rt[trajectory[0]] );
    int interval_idx = 0;
    for (list<TimeInterval>::const_iterator it = safe_intervals.begin(); it!= safe_intervals.end(); ++it){
        if (earliest_start_time <= it->t_max)
        {
            Node a_new_node;
            a_new_node.current_point = trajectory[0];
            a_new_node.cost_min = 0;
            a_new_node.g = 0;
            a_new_node.h = estimate_cost(trajectory[0], trajectory.back(), v_max);
            a_new_node.f = 0;
            a_new_node.color = 1;
            a_new_node.previous_point = -1;
            a_new_node.arrival_time_min = max(earliest_start_time, it->t_min);
            a_new_node.arrival_time_max = it->t_max;
            a_new_node.next_point = trajectory[1];
            a_new_node.index = 0;            
            a_new_node.interval_index = interval_idx;
            a_new_node.interval_t_min = it->t_min;
            a_new_node.interval_t_max = it->t_max;
            a_new_node.index = 0;
            p[0].push_back(a_new_node);                      
            interval_idx ++; 
        }
  
    }

    for (int i = 1; i < trajectory.size(); i++){
        list<TimeInterval> safe_intervals = getSafeIntervals( rt[trajectory[i]] );
        int interval_idx = 0;
        for (list<TimeInterval>::const_iterator it = safe_intervals.begin(); it!= safe_intervals.end(); ++it){
            Node a_new_node;
            a_new_node.current_point = trajectory[i];
            a_new_node.arrival_time_min = 0;
            a_new_node.arrival_time_max = 0;
            a_new_node.cost_min = 0;
            a_new_node.g = 0;
            a_new_node.h = estimate_cost(trajectory[i], trajectory.back(), v_max);
            a_new_node.f = 0;
            a_new_node.color = 0; 
            a_new_node.previous_point = i-1;
            
            if (i < trajectory.size()-1){
                a_new_node.next_point = trajectory[i+1];
            } else {
                a_new_node.next_point = -1;
            }

            a_new_node.interval_index = interval_idx;
            a_new_node.interval_t_min = it->t_min;
            a_new_node.interval_t_max = it->t_max; 
            a_new_node.index = i;
            p[i].push_back(a_new_node);    
            interval_idx ++;
        }
    }

    for (int i =0; i < p[0].size(); i++){
        open.push_back(p[0][i]);
    }
    
    int first_conflict_point_counter = 0;
    Path result_path;
    // std::pair<double, double> T_bound{-1, -1};
    while (open.size() != 0 ){
        //    remove s with the smallest f-value from OPEN
        int k = find_min(open);
        Node s = open.at(k);
        open.erase(open.begin() + k);
        if (result_path.size() > 0){
            if (s.f >= result_path.back().arrival_time){
                // return result_path;
                break;
            }
        }
        if (s.index == trajectory.size() - 1 ){
            // NOW WE HAVE A VECTOR OF POSSIBLE_SUCCESSOR (SUCCESSORS), WE NEED TO CONVERT IT INTO THE VECTOR OF PATH_ENTRY (PATH)  
            result_nodes.clear();
            Node res = s;
            while (true){
                result_nodes.insert(result_nodes.begin(), res);
                if (res.previous_point  == -1){
                    break;
                }
                res = node_storage[res.parent_index_1][res.parent_index_2];
            }
            // printf("[INFO] boundry for T: (%f, %f)\n", result_nodes[trajectory.size()-1].arrival_time_min, result_nodes[trajectory.size()-1].arrival_time_max);
            // if (cache_ptr->FindEntry(result_nodes)) {
            //     printf("[INFO] Hit the cache!\n");
            // } else {
            //     // SolveMILP(id, result_path, 0);
            //     bool success = SolveBezier(id, T_bound, init_drift, solution_control_points, result_path);
            //     if (!success) {
            //         cache_ptr->InsertEntry(result_nodes);
            //     }
            // }
            if (cache_ptr->FindEntry(result_nodes)) {
                // printf("[INFO] Hit the cache!\n");
                hit_count_++;
            }
            bizer_count_++;
            
            DEBUG_PRINT(printf("[INFO] Solving MILP for agent %d!\n", id);)
            bool success = RecurrentSolver(id, tmp_T, tmp_T_length, start_time, tmp_control_points, result_path);
            
            // std::cout << "success?: " << success;
            if (success) {
                if (tmp_T < min_T) {
                    min_T = tmp_T;
                    min_control_points = tmp_control_points;
                    min_T_length = tmp_T_length;
                }
                cache_ptr->InsertEntry(result_nodes);
                // printf("[INFO] [INFO] Inset new entry!\n");
            } else {
                DEBUG_PRINT(printf("[INFO] Fail for MILP!\n");) 
            }
                
        }
        else{
            node_storage[s.index].push_back(s);
            // int current_position = find_point(trajectory.size(), p, open[k].current_point);
            Successors successors = get_successors(p, s, trajectory.size(), v_min, v_max, length, rt, first_conflict_point_counter);
            for (int j = 0; j < successors.size(); ++j){
                // int next_position = successors[j].index;
                successors[j].parent = &(node_storage[s.index].back());
                successors[j].parent_index_1 = s.index;
                successors[j].parent_index_2 = node_storage[s.index].size() -1;
                // cout << "Compare:  " << successors[j].arrival_time_min << "    " << s.arrival_time_min + successors[j].cost_min<< endl;
                successors[j].g = successors[j].arrival_time_min;
                successors[j].f = successors[j].g + successors[j].h;
                open.push_back(successors[j]);
            }
        }
    }
    if (result_path.size() == 0){
        // cout << "[INFO] No path found" << endl;
        // std::cout << std::endl;
        // std::cout << std::endl;
        // return a Path, which is a vector of PathEntry. a PathEntry contains vertex (int), arrivalTime (double) and leavingTime (double)
        Path result_empty_path;
        return result_empty_path;

    }else{
        // cout << "[INFO] path found" << endl;
        // // for (auto tmp_node: result_path){
        // //     printf("The time, the arrive time is: %f, the leave time is: %f\n", tmp_node.arrival_time, tmp_node.leaving_time_tail);           
        // // }
        // std::cout << std::endl;
        // std::cout << std::endl;
        optimal_T = min_T_length;
        solution_control_points = tmp_control_points;
        return result_path;
    }
    
}

Successors SIPP::get_successors(
        vector<Node> p[],
        Node s,
        int trajectory_size,
        double v_min, 
        double v_max, 
        double length, 
        const ReservationTable& rt,
        int first_conflict_point_counter
    ){
    // empty successors, type vector
    Successors successors;

    // cfg = configuration of m applied to s

    // m = next possible motion -> get from Instance

    // m_time = time to excecute m- using formula t=d/v


    double m_time_min = estimate_cost(s.current_point, s.next_point, v_max);
    double m_time_max = estimate_cost(s.current_point, s.next_point, v_min);

    // start_t = time(s) + m_time
    double start_t = s.arrival_time_min + m_time_min;
    // ent_t = endTime(interval(s)) + m_time + time for vehicle to pass m
    
    int direction = vNameToDirection[s.next_point]; 
    
    double end_time = s.arrival_time_max;
    double end_t = end_time + m_time_max + Li(direction, length)/w + Li(direction, length)/v_min;
    // for each safe interval in cfg:
    // transfer reservation table into safe intervals!!!
    for (int it = 0; it!= p[s.index + 1].size(); ++it){
        Node possible_successor = p[s.index + 1][it];
        possible_successor.color = 1;
        if ((possible_successor.interval_t_min > end_t) || (possible_successor.interval_t_max < start_t)){
            continue;
        }
        double t_min = max(possible_successor.interval_t_min, start_t);
        double t_max = min(possible_successor.interval_t_max, end_t);
        // cout << possible_successor.interval_t_min << " " << end_t << " "<< possible_successor.interval_t_max << " "<< start_t << endl;
        //    t = earliest arrival time at cfg during interval i with no collisions

        //    if t does not exist:

        //        continue

        //    s_successor = start of configuration cfg with interval i and time t

        //    insert s_successor into successors

        possible_successor.arrival_time_min = t_min;
        possible_successor.arrival_time_max = t_max;


        possible_successor.cost_min = t_min - s.arrival_time_min;

        successors.push_back(possible_successor);
    }

   
    return successors;
}


float SIPP::Li(int direction, double agent_length){
    // straight 
    if (direction == 2){
        return agent_length;
    }
    //right, turnRadiusRight
    else if (direction == 1)
    {
        return 4*turn_radius_right*asin(agent_length/(2*turn_radius_right));
    }
    //left, turnRadiusLeft
    else if (direction == 0)
    {
        // cout << "direction: " << direction << "-" << 4*turnRadius*asin(vehicleLength/(2*turnRadius)) << endl; 
        return 4*turn_radius_left*asin(agent_length/(2*turn_radius_left));
    } else {
        return -1;
    }
}

double SIPP::estimate_cost(int start_point, int end_point, double speed){
    return ((pairDistancesMap)[start_point][end_point])/speed;
}


int SIPP::find(vector<Node>& open){
    for (int i = 0; i < open.size(); i++)
        if (open[i].color == 1)
            return i;
    return open.size() - 1;    
}

int SIPP::find_min(vector<Node>& open){
    int min_index = find(open);
    int min = open[min_index].f;
    for (int i = 0; i < open.size(); i++)
    {
        if (open[i].f < min && open[i].color == 1)
        {
            min_index = i;
            min = open[i].f;
        }
    }
    return min_index;

}


int SIPP::find_point(int n, Node* p, int current_point){
    for (int i = 0; i < n; i++)
        if (p[i].current_point == current_point)
            return  i;
    return -1;
}

list<TimeInterval> SIPP::getSafeIntervals(list<TimeInterval> rt)
{
    vector<TimeInterval> safe_intervals;
    if (rt.size() == 0){
        TimeInterval aInterval;
        aInterval.t_min = 0;
        aInterval.t_max = 100000;

        safe_intervals.push_back(aInterval);

        list<TimeInterval> output;

        copy(safe_intervals.begin(), safe_intervals.end(), back_inserter(output));
        return output;
    }

    list<TimeInterval> localRT = rt;
    localRT.sort([](const TimeInterval &f, const TimeInterval &s) { return f.t_min < s.t_min; });



    if (localRT.begin()->t_min > 0){
        TimeInterval aInterval_1;
        aInterval_1.t_min = 0;
        aInterval_1.t_max = localRT.begin()->t_min;
        safe_intervals.push_back(aInterval_1);
    }
    if (localRT.begin()->t_max < 100000){
        TimeInterval aInterval_2;
        aInterval_2.t_min = localRT.begin()->t_max;
        aInterval_2.t_max = 100000;
        safe_intervals.push_back(aInterval_2);
    }
    for (list<TimeInterval>::iterator it = std::next(localRT.begin()); it!= localRT.end(); ++it){
        vector<int> toBeDeleted;
        for (int sfIter = 0; sfIter < safe_intervals.size(); sfIter++){
            if (safe_intervals[sfIter].t_min >= it->t_min && safe_intervals[sfIter].t_min < it->t_max && safe_intervals[sfIter].t_max > it->t_max){
                safe_intervals[sfIter].t_min = it->t_max;
            }
            if (safe_intervals[sfIter].t_min < it->t_min && safe_intervals[sfIter].t_max > it->t_min && safe_intervals[sfIter].t_max <= it->t_max){
                safe_intervals[sfIter].t_max = it->t_min;
            }

            if (safe_intervals[sfIter].t_min < it->t_min && safe_intervals[sfIter].t_max > it->t_max){
                TimeInterval aInterval_3;
                aInterval_3.t_min = it->t_max;
                aInterval_3.t_max = safe_intervals[sfIter].t_max;


                safe_intervals[sfIter].t_max = it->t_min;
                safe_intervals.insert(safe_intervals.begin() + sfIter+1, aInterval_3);
                break;

            }

            if (safe_intervals[sfIter].t_min > it->t_min && safe_intervals[sfIter].t_max < it->t_max){
                toBeDeleted.push_back(sfIter);
            }

        }
        
        if (toBeDeleted.size()>0){
            for (int toBeDeletedIter = toBeDeleted.size()-1; toBeDeletedIter >=0; toBeDeletedIter--){
                safe_intervals.erase(safe_intervals.begin() + toBeDeleted[toBeDeletedIter]);
            }
        }
  
    }


    list<TimeInterval> output;
    copy(safe_intervals.begin(), safe_intervals.end(), back_inserter(output));
    return output;
}
