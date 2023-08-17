#include "SIPP.h"
#include "Instance.h"
#include "common.h"
#include "milp_cache.h"
#include <chrono>
#include <cstddef>
#include <cstdio>
#include <memory>
#include <vector>


using namespace std;

SIPP::SIPP(std::shared_ptr<Instance> instance)
{
    instance_ptr = instance;
    // agents = instance_ptr->agents;
    cache_ptr = std::make_shared<CacheMILP>();
    solution_cache_ptr = std::make_shared<CacheMILP>();
    heuristic_vec.resize(instance_ptr->agents.size());
    // pairDistancesMap = instance_ptr->getPairDistancesMap();
    // vNameToDirection = instance_ptr->getVNameToDirection();
    // vNameToID = instance_ptr->getVNameToID();    
}

bool SearchInterval(double begin_time, double end_time, double& block_start, double& block_end, BernsteinPolynomial& bern)
{
    double STEP_SIZE = 0.3;
    block_start = begin_time;
    while(true) {
        if (true) {
            break;
        }
        block_start += STEP_SIZE;
    }
    block_start -= STEP_SIZE;
    block_end = block_start;
    while(true) {
        if (true) {
            break;
        }
        block_end += STEP_SIZE;
    }
    return true;
}

bool SIPP::SolveBezier(
    double T_optimal,
    std::vector<double>& solution_control_points,
    Path& result_path)
{
    double a_min = curr_agent.a_min;
    double a_max = curr_agent.a_max;
    double v_min = curr_agent.v_min;
    double v_max = curr_agent.v_max;
    double t_length = T_optimal;
    int traj_size = result_nodes.size();
    // printf("debug value for agent: a_min: %f, a_max: %f, v_min: %f, v_max %f, "
    //     "init speed: %f, length_T: %f\n",
    //     a_min, a_max, v_min, v_max, curr_agent.init_velocity, T_optimal);
    // std::cout << "T value is: " << t_length << std::endl;
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
    IloNumVarArray control_points(env, n_points, 0.0, 2000.0);

    // Set start condition
    IloExpr init_place_expr(env);
    std::vector<double> init_bern_val;
    bern_poly.CalculateVal(0, init_bern_val);
    for (int i = 0; i < n_points; i++) {
        init_place_expr += control_points[i]*init_bern_val[i];
        // printf("Bernstein value is: %f", init_bern_val[i]);
    }
    model.add(init_place_expr == 0.0);
    init_place_expr.end();

    // Set end condition
    IloExpr end_place_expr(env);
    std::vector<double> end_bern_val;
    bern_poly.CalculateVal(t_length, end_bern_val);
    for (int i = 0; i < n_points; i++) {
        end_place_expr += control_points[i]*end_bern_val[i];
    }
    model.add(end_place_expr == (traj_size-1)*CELL_DIS);
    end_place_expr.end();

    // Vertex time window constraints
    for (int i = 0; i < traj_size; i++){
        if (i != 0){
            // arrival vertex time
            IloExpr arr_place_expr(env);
            std::vector<double> arr_bern_val;
            bern_poly.CalculateVal(result_nodes[i]->arrival_time_min, arr_bern_val);
            for (int j = 0; j < n_points; j++) {
                arr_place_expr += control_points[j]*arr_bern_val[j];
            }
            model.add(arr_place_expr <= (i*CELL_DIS - CELL_DIS/2 - curr_agent.length));
            arr_place_expr.end();
        }
        // leave vertex time
        if (i != (traj_size-1)) {
            IloExpr leave_place_expr(env);
            std::vector<double> leave_bern_val;
            double arr_max = min(result_nodes[i]->arrival_time_max, t_length);
            // printf("[INFO] arr_max is: %f\n", arr_max);
            bern_poly.CalculateVal(arr_max, leave_bern_val);
            // std::cout << "For idx = " << i << ", time to leave is: " << result_nodes[i].arrival_time_max << std::endl;
            for (int j = 0; j < n_points; j++) {
                leave_place_expr += control_points[j]*leave_bern_val[j];
                // std::cout << leave_bern_val[j] << ",\t";
            }
            // std::cout << std::endl;
            model.add(leave_place_expr >= (i*CELL_DIS + CELL_DIS/2 + curr_agent.length));
            leave_place_expr.end();
        } 
        // else {
        //     IloExpr leave_place_expr(env);
        //     std::vector<double> leave_bern_val;
        //     double arr_max = min(result_nodes[i]->arrival_time_max, t_length);
        //     // printf("[INFO] arr_max is: %f\n", arr_max);
        //     bern_poly.CalculateVal(arr_max, leave_bern_val);
        //     // std::cout << "For idx = " << i << ", time to leave is: " << result_nodes[i].arrival_time_max << std::endl;
        //     for (int j = 0; j < n_points; j++) {
        //         leave_place_expr += control_points[j]*leave_bern_val[j];
        //         // std::cout << leave_bern_val[j] << ",\t";
        //     }
        //     // std::cout << std::endl;
        //     model.add(leave_place_expr >= (i*CELL_DIS));
        //     leave_place_expr.end();
        // }
    }

    // Add init speed condition
    IloExpr init_speed_expr(env);
    // For simplicity, we just use 2 here. For any order derivative, use pow(2, first_order_deriv.control_points_deriv.deriv_order) instead
    for (int j = 0; j < 2; j++) {
        init_speed_expr += first_order_deriv.control_points_deriv.ratio * first_order_deriv.control_points_deriv.sign_idx[0][j] 
                                * control_points[first_order_deriv.control_points_deriv.orignal_idx[0][j]];
    }
    model.add(init_speed_expr == curr_agent.init_velocity);
    init_speed_expr.end();

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
    // printf("Finish model!\n");
    // cplex.extract(model);
    // cplex.exportModel("./output/tpg_model.lp");  
    auto startTime = std::chrono::high_resolution_clock::now();
    if(cplex.solve()) {
        // printf("solved cplex model\n");
        // cplex.writeSolution("./output/tpg_solution");
        auto stopTime = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stopTime - startTime); 
        if (DEBUG_BEZIER) {
            std::cout << "[INFO] TPG solved with runtime: " << duration.count()/1000000.0 << std::endl;
        }
        
        total_runtime_ += duration.count()/1000000.0;

        for (int i = 0; i < n_points; i++) {
            solution_control_points.push_back(cplex.getValue(control_points[i]));
            // std::cout << solution_control_points[i] << ", ";
        }

        Path potential_path;
        for (int i = 0; i < traj_size; i++){
            PathEntry a_path;
            // Round all the time slot to x.1
            
            
            a_path.location = result_nodes[i]->current_point;
            // Only need to modify this
            if (i == 0) {
                a_path.arrival_time = 0.0;
            } else {
                double block_start = floor((result_nodes[i]->arrival_time_min)/TIME_STEP_SIZE) * TIME_STEP_SIZE  ;
                while((bern_poly.GetVal(block_start, solution_control_points) - EPS) <= (i * CELL_DIS - CELL_DIS/2 - curr_agent.length) 
                    and block_start < INF) {
                    block_start += TIME_STEP_SIZE;
                }
                block_start -= TIME_STEP_SIZE;
                a_path.arrival_time = block_start;
            }
                
            // Only need to modify this
            if (i == traj_size - 1) {
                a_path.leaving_time_tail = floor(T_optimal/TIME_STEP_SIZE)*TIME_STEP_SIZE;
            } else {
                double block_end = a_path.arrival_time;
                while((bern_poly.GetVal(block_end, solution_control_points) + EPS
                < (i * CELL_DIS + CELL_DIS/2 + curr_agent.length)) 
                and block_end < INF) {
                    block_end += TIME_STEP_SIZE;
                }
                if (block_end >= INF) {
                    // printf("block end bigger than INF, potential bug!\n");
                    exit(-1);
                }
                a_path.leaving_time_tail = block_end;
            }

            potential_path.push_back(a_path);
            // printf("TGP: For node %d, the arrive time is: %f, dist: %f, dist2: %f, the leave time is: %f, dist: %f, dist2: %f\n", 
            //     i, 
            //     a_path.arrival_time, 
            //     bern_poly.GetVal(a_path.arrival_time, solution_control_points), 
            //     bern_poly.GetVal(a_path.arrival_time + TIME_STEP_SIZE, solution_control_points), 
            //     a_path.leaving_time_tail, 
            //     bern_poly.GetVal(a_path.leaving_time_tail - TIME_STEP_SIZE, solution_control_points),
            //     bern_poly.GetVal(a_path.leaving_time_tail, solution_control_points));           
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
        // std::string model_name = "./output/tpg_model_agent_" + std::to_string(curr_agent.id) + std::to_string(count_called) + ".lp";
        // cplex.exportModel(model_name.c_str());
        env.end();
        return false;
    }
}


bool SIPP::SolveSlackBezier(
    double T,
    std::vector<double>& solution_control_points,
    Path& result_path,
    double& slack_var)
{
    // TODO: tune this parameters
    double a_min = curr_agent.a_min;
    double a_max = curr_agent.a_max;
    double v_min = curr_agent.v_min;
    double v_max = curr_agent.v_max;
    int traj_size = result_nodes.size();

    // std::cout << "[INFO] Solve for agent: " << curr_agent.id << std::endl;

    double t_length = T;
    // printf("Slack debug value for agent: a_min: %f, a_max: %f, v_min: %f, v_max %f, "
    //     "init speed: %f. T length is: %f\n",
    //     a_min, a_max, v_min, v_max, curr_agent.init_velocity, t_length);

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
    IloNumVarArray control_points(env, n_points+1, 0, 2000.0);
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
    model.add(init_place_expr == 0.0);
    init_place_expr.end();

    // Set end condition
    IloExpr end_place_expr(env);
    std::vector<double> end_bern_val;
    bern_poly.CalculateVal(t_length, end_bern_val);
    for (int i = 0; i < n_points; i++) {
        end_place_expr += control_points[i]*end_bern_val[i];
    }
    double final_dist = (traj_size - 1)*CELL_DIS;
    // model.add(end_place_expr == (pairDistancesMap[result_nodes[0].current_point][result_nodes[trajectory.size()-1].current_point] + length + init_drift));
    model.add(end_place_expr == final_dist);
    end_place_expr.end();
    // Vertex time window constraints
    for (int i = 0; i < traj_size; i++){
        if (i != 0){
            // arrival vertex time
            IloExpr arr_place_expr(env);
            std::vector<double> arr_bern_val;
            bern_poly.CalculateVal(result_nodes[i]->arrival_time_min, arr_bern_val);
            for (int j = 0; j < n_points; j++) {
                arr_place_expr += control_points[j]*arr_bern_val[j];
            }
            model.add(arr_place_expr <= (i*CELL_DIS - CELL_DIS/2 - curr_agent.length + control_points[n_points]));
            arr_place_expr.end();
        }
        // leave vertex time
        if (i != (traj_size-1)) {
            IloExpr leave_place_expr(env);
            std::vector<double> leave_bern_val;
            double arr_max = min(result_nodes[i]->arrival_time_max, t_length);
            // printf("[INFO] arr_max is: %f\n", arr_max);
            bern_poly.CalculateVal(arr_max, leave_bern_val);
            // std::cout << "For idx = " << i << ", time to leave is: " << result_nodes[i].arrival_time_max << std::endl;
            for (int j = 0; j < n_points; j++) {
                leave_place_expr += control_points[j]*leave_bern_val[j];
                // std::cout << leave_bern_val[j] << ",\t";
            }
            // std::cout << std::endl;
            model.add(leave_place_expr >= (i*CELL_DIS + CELL_DIS/2 + curr_agent.length - control_points[n_points]));
            leave_place_expr.end();
        } 
        // else {
        //     IloExpr leave_place_expr(env);
        //     std::vector<double> leave_bern_val;
        //     double arr_max = min(result_nodes[i]->arrival_time_max, t_length);
        //     // printf("[INFO] arr_max is: %f\n", arr_max);
        //     bern_poly.CalculateVal(arr_max, leave_bern_val);
        //     // std::cout << "For idx = " << i << ", time to leave is: " << result_nodes[i].arrival_time_max << std::endl;
        //     for (int j = 0; j < n_points; j++) {
        //         leave_place_expr += control_points[j]*leave_bern_val[j];
        //         // std::cout << leave_bern_val[j] << ",\t";
        //     }
        //     // std::cout << std::endl;
        //     model.add(leave_place_expr >= (i*CELL_DIS - control_points[n_points]));
        //     leave_place_expr.end();
        // }
    }

    // Add init speed condition
    IloExpr init_speed_expr(env);
    // For simplicity, we just use 2 here. For any order derivative, use pow(2, first_order_deriv.control_points_deriv.deriv_order) instead
    for (int j = 0; j < 2; j++) {
        init_speed_expr += first_order_deriv.control_points_deriv.ratio * first_order_deriv.control_points_deriv.sign_idx[0][j] 
                                * control_points[first_order_deriv.control_points_deriv.orignal_idx[0][j]];
    }
    model.add(init_speed_expr == curr_agent.init_velocity);
    init_speed_expr.end();

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
    // cplex.exportModel("./output/tpg_slack_model.lp"); 
    // cplex.extract(model);
    // std::string model_name = "./output/tpg_model_agent_" + std::to_string(T) + ".lp";
    // cplex.exportModel(model_name.c_str()); 

    auto startTime = std::chrono::high_resolution_clock::now();
    if(cplex.solve()) {
        // cplex.writeSolution("./output/tpg_solution");
        auto stopTime = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stopTime - startTime); 
        
        total_runtime_ += duration.count()/1000000.0;

        // for (int i = 0; i < n_points; i++) {
        //     solution_control_points.push_back(cplex.getValue(control_points[i]));
        //     // std::cout << solution_control_points[i] << ", ";
        // }
        slack_var = cplex.getValue(control_points[n_points]);
        if (DEBUG_BEZIER){
            std::cout << "[INFO] TPG solved with runtime: " << duration.count()/1000000.0 << std::endl;
            printf("The slack variable is: %f\n\n\n", slack_var);
        }
        env.end();
        return true;
    } else {
        // printf("[INFO] TPG not solved!\n\n\n");

        // cplex.extract(model);
        // std::string model_name = "./output/tpg_model_agent_" + std::to_string(curr_agent.id) + std::to_string(T) + ".lp";
        // cplex.exportModel(model_name.c_str());
        env.end();
        return false;
    }
}

bool SIPP::SlackGradient(
    double target_t, double& gradient)
{
    double delta = 0.1;
    std::vector<double> solution_control_points;
    Path result_path;
    double slack_t;
    // printf("Solve Slack gradient!\n");
    SolveSlackBezier(target_t, solution_control_points, result_path, slack_t);
    double slack_delta;
    SolveSlackBezier(target_t+delta, solution_control_points, result_path, slack_delta);
    gradient = (slack_delta - slack_t)/delta;
    if (DEBUG_BEZIER){
        printf("Gradient at time %f is: %f\n", target_t, gradient);

    }
    if (slack_t == 0){
        return true;
    } else {
        return false;
    }
}

bool SIPP::RecurrentSolver(BezierNode& solution, Path& result_path)
{
    // printf("Call recurrent solver, the path length is: %ld!\n", result_nodes.size());
    double lower_bound = result_nodes[result_nodes.size()-1]->arrival_time_min;
    double upper_bound = lower_bound;
    double mid;
    double Max_it_thresh = 0.2;
    double gradient_t_low = 0;
    double gradient_t_high = 0;
    double grad_mid = 0;
    bool t_high_valid = false;
    bool upper_in_bould = false;
    double optimal_T;

    bool low_valid_solution = SlackGradient(lower_bound, gradient_t_low);
    bool upper_valid_solution = false;

    if (low_valid_solution) {
        // TO DO: add the solution to from the solver
        optimal_T = lower_bound;
    } else{
        while (upper_bound < INF) {
            upper_bound = 2 * upper_bound;
            // printf("[WARN] current upper bound is: %f\n", upper_bound);
            upper_valid_solution = SlackGradient(upper_bound, gradient_t_high);
            if (gradient_t_high > 0 or upper_valid_solution) {
                optimal_T = upper_bound;
                upper_in_bould = true;
                // printf("[WARN] The upper bound found: %f, gradient_t_high: %f, upper valid: %d\n", 
                //     upper_bound, gradient_t_high, upper_valid_solution);
                if (upper_valid_solution) {
                    t_high_valid = true;
                }
                break;
            }
        }

        if (!upper_in_bould) {
            return false;
        }

        while ((upper_bound - lower_bound) >= Max_it_thresh) {
            if (DEBUG_BEZIER){
                printf("[INFO] The upper bound is %f, the lower bound is %f\n", upper_bound, lower_bound);
            }

            mid = (upper_bound + lower_bound)/2;
            double slack;
            SolveSlackBezier(mid, solution.control_points, result_path, slack);
            if (slack == 0) {
                // printf("find new optimal T: %f\n", mid);
                upper_bound = mid;
                optimal_T = mid;
                t_high_valid = true;
            } else {
                // printf("The slack variable is: %.9f\n", slack);
                if (t_high_valid){
                    lower_bound = mid;
                } else {
                    SlackGradient(mid, grad_mid);
                    if (grad_mid > 0) {
                        upper_bound = mid;
                    } else {
                        lower_bound = mid;
                    }
                }
            }
            // if ((upper_bound - lower_bound) <= Max_it_thresh && t_high_valid) {
            //     break;
            // }
        }
    }

    if (!t_high_valid) {
        // printf("Unable to solve for the this SIPP\n");
        return false;
    }
    
    solution.control_points.clear();
    if (DEBUG_BEZIER){
        std::cout << "The solved optimal T is: " << optimal_T << std::endl;
    }
    bool success = SolveBezier(optimal_T, solution.control_points, result_path);
    if (success){
        // printf("Success solve for the this SIPP with Optimal_T: %f\n", optimal_T);
        solution.optimal_T = optimal_T;
        if (DEBUG_BEZIER) {
            std::cout << "All control points: ";
            for (double point: solution.control_points){
                std::cout << point << ";";
            }
            std::cout << std::endl;
        }
        return true;
    } else {
        // printf("Unable to solve for the this SIPP\n");
        // printf("Optimla t is: %f\n", solution.optimal_T);
        // exit(-1);
        return false;
    }
}

void SIPP::PrintNonzeroRT(const ReservationTable& rt)
{
    for (unsigned int i = 0; i < rt.size(); i++) {
        if (rt[i].size() > 0) {
            printf("Agent start: %d : interval at location: %d:\t", curr_agent.start_location, i);
            for (TimeInterval tmp_interval: rt[i]) {
                printf("%f -> %f,\t", tmp_interval.t_min, tmp_interval.t_max);
            }
            printf("\n");
        }
    }
}

bool SIPP::GetFirstInterval(const ReservationTable& rt, std::shared_ptr<Node>& first_interval)
{
    // printf("At start location: %d : interval at location: %d:\t", curr_agent.start_location, curr_agent.start_location);
    // for (TimeInterval tmp_interval: rt[curr_agent.start_location]) {
    //     printf("%f -> %f,\t", tmp_interval.t_min, tmp_interval.t_max);
    // }
    // printf("\n");
    std::shared_ptr<Node> init_null_interval = std::make_shared<Node> ();
    init_null_interval->current_point = -1;   
    first_interval = std::make_shared<Node> ();
    vector<TimeInterval> safe_intervals;
    list<TimeInterval> localRT = rt[curr_agent.start_location];
    
    localRT.sort([](const TimeInterval &f, const TimeInterval &s) { return f.t_min < s.t_min; });
    if (DEBUG_SIPP){
        for (auto tmp_iterval: localRT) {
            printf("Interval in RT: %f -> %f\n", tmp_iterval.t_min, tmp_iterval.t_max);
        }
    }
    list<TimeInterval>::iterator rt_it;
    for (rt_it = localRT.begin(); rt_it != localRT.end(); rt_it++) {
        if (rt_it->t_max <= curr_agent.earliest_start_time + EPS) {
            continue;
        } else {
            if (rt_it->t_min > curr_agent.earliest_start_time){
                first_interval->current_point = curr_agent.start_location;
                first_interval->arrival_time_min = curr_agent.earliest_start_time;
                first_interval->arrival_time_max = rt_it->t_min;
                first_interval->parent = init_null_interval;
                break;
            } else {
                // printf("[WARN] Initial position if blocked, invalid time interval: %f to %f, while the earliest start time is: %f\n", 
                // rt_it->t_min, rt_it->t_max, curr_agent.earliest_start_time);
                // exit(-2);
                return false;
            }
        }
        
        
    }
    if (rt_it == localRT.end()){
        first_interval->current_point = curr_agent.start_location;
        first_interval->arrival_time_min = curr_agent.earliest_start_time;
        first_interval->arrival_time_max = INF;
        first_interval->parent = init_null_interval;
    }
    // printf("Generate init interval: %f to %f!\n", first_interval->arrival_time_min, first_interval->arrival_time_max);

    return true;
}

void SIPP::Reset()
{
    all_nodes_table.clear();
    result_nodes.clear();
    open = std::priority_queue<std::shared_ptr<Node>, std::vector<std::shared_ptr<Node>>, NodeCompare>();
}

void SIPP::updateResultNodes(std::shared_ptr<Node> s)
{
    result_nodes.clear();
    std::shared_ptr<Node> res = s;
    std::shared_ptr<Node> tmp_node;

    while (true){
        tmp_node = std::make_shared<Node>();
        tmp_node->arrival_time_min = res->arrival_time_min - curr_agent.earliest_start_time;
        double arr_max_t = INF;
        if (res->arrival_time_max != INF) {
            arr_max_t = res->arrival_time_max - curr_agent.earliest_start_time;
        }
        tmp_node->arrival_time_max = arr_max_t;
        tmp_node->current_point = res->current_point;
        tmp_node->interval_index = -1;
        if (tmp_node->arrival_time_min <= WINDOWS_SIZE){
            result_nodes.insert(result_nodes.begin(), tmp_node);
        } else {
            result_nodes.clear();
            result_nodes.insert(result_nodes.begin(), tmp_node);
        }

        if (res->parent->current_point  == -1){
            break;
        }
        res = res->parent;
    }
}

bool SIPP::run(
    int agentID, 
    const ReservationTable& rt,
    BezierNode& solution,
    Path& path,
    double cutoff_time)
{
    // agent details
    curr_agent = instance_ptr->agents[agentID];
    curr_agent.id = agentID;
    if (DEBUG_SIPP){
        printf("Number of sipp called %d. Solve for agent: %d: start from: %d, goal: %d, "
        "with the earliest start time at: %f, init_spped: %f\n", 
        count_called, agentID, curr_agent.start_location, curr_agent.goal_location, 
        curr_agent.earliest_start_time, curr_agent.init_velocity);
    }
    

    if (heuristic_vec[agentID].empty()) {
        // printf("call heuristic generator!\n");
        Dijstra(curr_agent.goal_location);
        // printf("Finish dij heuristic generator!\n");

    }
    if (!duplicate_table.empty()) {
        for (auto tmp_cache: duplicate_table) {
            tmp_cache.reset();
        }
    }
    
    duplicate_table.clear();
    duplicate_table.resize(instance_ptr->GetNumOfVertices());

    
    // // from agent's ID -> get agent's start_location, goal location, earliest_start_time, maximum velocity
    Reset();
    

    count_called++;

    if (curr_agent.goal_location == curr_agent.start_location) {
        path.clear();
        return true;
    }

    solution.optimal_T = INF;
    std::vector<double> tmp_control_points;

    // int interval_idx = 0;
    std::shared_ptr<Node> init_node;
    bool first_interval_free = GetFirstInterval(rt, init_node);
    if (!first_interval_free) {
        path.clear();
        return false;
    }
    all_nodes_table[init_node->current_point].push_back(init_node);

    // printf("INIT_node::Time interval is: %f to %f\n", init_node->arrival_time_min, init_node->arrival_time_max);

    open.push(init_node);

    Path optimal_result_path;
    optimal_result_path.clear();
    Path result_path;
    BezierNode tmp_solution;
    // if ((init_node->arrival_time_max - init_node->arrival_time_min) < 1.35) {
    //     return false;
    // }
    // std::pair<double, double> T_bound{-1, -1};
    auto start_time = Time::now();
    while (!open.empty()){
        //    remove s with the smallest f-value from OPEN
        // assert();
        // printf("Size of open set is: %ld\n", open.size());
        auto tmp_end_time = Time::now();
        std::chrono::duration<float> tmp_duration = tmp_end_time - start_time;
        if (tmp_duration.count() > cutoff_time){
            break;
        }
        std::shared_ptr<Node> s = open.top();
        open.pop();
        // if (!earlyPruning(s)){
        //     continue;
        // }
        if (DEBUG_SIPP){
            printf("[INFO] Expand node at location: %d, the coordinate: (%d, %d). with parent: %d, h value is: %f, g value is: %f, f value is: %f\n", 
            s->current_point, instance_ptr->getCoordinate(s->current_point).first, instance_ptr->getCoordinate(s->current_point).second,
            s->parent->current_point, s->h, s->g, s->f);
        }
       

        // printf("SIPP_loop::Time interval is: %f to %f\n", s.arrival_time_min, s.arrival_time_max);

        if (optimal_result_path.size() > 0){
            if (s->f > optimal_result_path.back().arrival_time){
                // return result_path;
                break;
            }
        }
        // printf("The current point is: %d, the goal location is: %d\n", s->current_point, curr_agent.goal_location);
        // If the reach the goal, try to solve the path using the Bezier curve
        if (s->current_point == curr_agent.goal_location){
            if (s->arrival_time_max == INF) {
                // NOW WE HAVE A VECTOR OF POSSIBLE_SUCCESSOR (SUCCESSORS), WE NEED TO CONVERT IT INTO THE VECTOR OF PATH_ENTRY (PATH)  
                // printf("Found path to the goal!\n");

                updateResultNodes(s);
                // bool success = RecurrentSolver(tmp_solution, result_path);                
                // if (success and tmp_solution.optimal_T < solution.optimal_T) {
                //     solution.optimal_T = tmp_solution.optimal_T;
                //     solution.control_points = tmp_solution.control_points;
                //     for (unsigned int i = 0; i < result_path.size(); i++) {
                //         // printf("The found path at location: %d, from %f to %f\n", result_nodes[i]->current_point, result_path[i].arrival_time, result_path[i].leaving_time_tail);

                //         result_path[i].arrival_time = result_path[i].arrival_time + curr_agent.earliest_start_time;
                //         result_path[i].leaving_time_tail = result_path[i].leaving_time_tail + curr_agent.earliest_start_time;
                //         result_path[i].location = result_nodes[i]->current_point;
                //     }
                //     optimal_result_path = result_path;
                //     instance_ptr->agents[curr_agent.id].tmp_cost = result_path.back().leaving_time_tail 
                //         + heuristic_vec[agentID][result_path.back().location]->arrival_time_min;
                //     // min_T_length = tmp_T_length;
                // }

                // printf("Find new path the the goal with length of %ld!\n", result_nodes.size());
                // for (std::shared_ptr<Node> tmp_n: result_nodes) {
                //     printf("The entry at location: %d, from %f to %f\n", tmp_n->current_point, tmp_n->arrival_time_min, tmp_n->arrival_time_max);
                // }
                // printf("Traj length is: %ld\n", result_nodes.size());
                // PrintNonzeroRT(rt);
                // exit(-1);
                bool find_entry_success = cache_ptr->FindEntry(result_nodes, curr_agent.init_velocity);
                result_path.clear();
                bool find_solution_success = solution_cache_ptr->RetriveEntry(result_nodes, curr_agent.init_velocity, tmp_solution, result_path);
                // printf("[INFO] find fail: %d, find solution: %d\n", find_entry_success, find_solution_success);
                if (find_entry_success) {
                    // printf("[INFO] Hit the failed cache!\n");
                    continue;
                    hit_count_++;
                } else if(find_solution_success) {
                    ;
                    // printf("[INFO] Found solution in the cache with optimal_T: %f!\n", tmp_solution.optimal_T);
                } else {
                    bizer_count_++;
                    // printf("[INFO] Solving Bezier for agent %d!\n", agentID);
                    bool success = RecurrentSolver(tmp_solution, result_path);                
                    // printf("Recurrent solver: success?: %d, result path length: %ld\n", success, result_path.size());
                    if (success) {
                        solution_cache_ptr->InsertEntry(result_nodes, curr_agent.init_velocity, tmp_solution, result_path);
                    } else {
                        // printf("[INFO] [INFO] Inset new entry!\n");
                        cache_ptr->InsertEntry(result_nodes, curr_agent.init_velocity);
                        // printf("[INFO] Fail for MILP!\n");
                    }
                }

                if (tmp_solution.optimal_T < solution.optimal_T) {
                    solution.optimal_T = tmp_solution.optimal_T;
                    solution.control_points = tmp_solution.control_points;
                    for (unsigned int i = 0; i < result_path.size(); i++) {
                        // printf("The found path at location: %d, from %f to %f\n", result_nodes[i]->current_point, result_path[i].arrival_time, result_path[i].leaving_time_tail);

                        result_path[i].arrival_time = result_path[i].arrival_time + curr_agent.earliest_start_time;
                        result_path[i].leaving_time_tail = result_path[i].leaving_time_tail + curr_agent.earliest_start_time;
                        result_path[i].location = result_nodes[i]->current_point;
                    }
                    optimal_result_path = result_path;
                    instance_ptr->agents[curr_agent.id].tmp_cost = result_path.back().leaving_time_tail 
                        + heuristic_vec[agentID][result_path.back().location]->arrival_time_min;
                    // min_T_length = tmp_T_length;
                }
            }
            // else {
            //     printf("At location: %d, Time interval to the end path with arr_max: %f\n", 
            //     s->current_point, s->arrival_time_max);
            // }
        }
        else{
            // printf("Expand node!\n");
            // For all the neighbor location, all need to do this operation
            std::list<int> neighbors = instance_ptr->getNeighbors(s->current_point);
            for (int neighbor: neighbors) {
                if (neighbor == s->parent->current_point) {
                    // omit the back and forth case
                    continue;
                }
                if (DEBUG_SIPP){
                    std::cout << "visit neighbor: " << neighbor << std::endl;
                }

                if (s->arrival_time_min > WINDOWS_SIZE) {
                    // AStarSearch(s);
                    // printf("Search for nodes out of the window!\n");
                    NodeSearch(s);
                } else {
                    // node_storage[s.index].push_back(s);
                    // int current_position = find_point(trajectory.size(), p, open[k].current_point);
                    // To Do: need to change this get successors function
                    // Successors successors = get_successors(p, s, trajectory.size(), v_min, v_max, length, rt, first_conflict_point_counter);
                    std::list<TimeInterval> neighbor_successors = get_successors(s, neighbor, rt);
                    if (DEBUG_SIPP){
                        if (neighbor_successors.size() > 1){
                            // std::cout << "visit neighbor: " << neighbor << std::endl;
                            // printf("Neighbor successor size: %ld\n", neighbor_successors.size());
                            for (TimeInterval tmp_interval: neighbor_successors) {
                                printf("At location: %d, interval from %f to %f\n", neighbor, tmp_interval.t_min, tmp_interval.t_max);
                            }
                        }
                    }
                    for (TimeInterval neighbor_interval: neighbor_successors){
                        std::shared_ptr<Node> successor_node = std::make_shared<Node>();
                        successor_node->current_point = neighbor;
                        successor_node->arrival_time_min = neighbor_interval.t_min;
                        successor_node->arrival_time_max = neighbor_interval.t_max;
                        successor_node->interval_index = neighbor_interval.id;
                        successor_node->g = neighbor_interval.t_min;
                        // printf("At %d, heuristic val: %f\n", neighbor, heuristic_vec[agentID][neighbor]);
                        successor_node->h = heuristic_vec[agentID][neighbor]->arrival_time_min;
                        successor_node->f = successor_node->g + successor_node->h;

                        // printf("[INFO] At location: %d, h value is: %f, g value is: %f, f value is: %f. The coordinate: (%d, %d)\n", 
                        //     neighbor, successor_node->h, successor_node->g, successor_node->f,
                        //     instance_ptr->getCoordinate(neighbor).first, instance_ptr->getCoordinate(neighbor).second);

                        successor_node->parent = s;
                        // successor_node->color = 1;
                        // if (dominanceCheck(successor_node)){
                        //     // printf("[INFO] At location: %d, h value is: %f, g value is: %f, f value is: %f. The coordinate: (%d, %d)\n", 
                        //     // neighbor, successor_node->h, successor_node->g, successor_node->f,
                        //     // instance_ptr->getCoordinate(neighbor).first, instance_ptr->getCoordinate(neighbor).second);
                        //     open.push(successor_node);
                        //     all_nodes_table[successor_node->current_point].push_back(successor_node);
                        // }
                        open.push(successor_node);
                        all_nodes_table[successor_node->current_point].push_back(successor_node);
                    }
                }
            }
        }
    }
    // printf("[WARN] Hit count is: %d, total call is: %d\n", hit_count_, bizer_count_);
    if (optimal_result_path.size() == 0){
        // printf("The optimal T is: %f\n", solution.optimal_T);
        // cout << "[INFO] No path found" << endl;
        // return a Path, which is a vector of PathEntry. a PathEntry contains vertex (int), arrivalTime (double) and leavingTime (double)
        path.clear();
        return false;
    }else{
        // cout << "[INFO] path found" << endl;
        
        path.clear();
        path = optimal_result_path;
        // printf("The optimal T is: %f, with control points:\n", solution.optimal_T);
        // for (double point: solution.control_points) {
        //     printf("%f,\t", point);
        // }
        // printf("\n");
        if (solution.optimal_T < REPLAN_SIZE) {
            path.back().leaving_time_tail = INF;
        }
        // for (auto tmp_node: path){
        //     printf("The location %d, the arrive time is: %f, the leave time is: %f\n", tmp_node.location, 
        //         tmp_node.arrival_time, tmp_node.leaving_time_tail);           
        // }
        // printf("\n");
        // exit(0);
        return true;
    }
    
}

bool SIPP::Dijstra(int start_loc)
{
    std::vector<std::shared_ptr<Node>> curr_agent_heuristic;
    curr_agent_heuristic.resize(instance_ptr->map_size);
    // printf("curr_agent_heuristic size is: %ld\n", curr_agent_heuristic.size());
    std::priority_queue<std::shared_ptr<Node>, std::vector<std::shared_ptr<Node>>, NodeCompare> dij_open;
    std::unordered_map<int, std::shared_ptr<Node>> close_set;
    std::shared_ptr<Node> root_node = std::make_shared<Node>();
    root_node->arrival_time_min = 0.0;
    root_node->arrival_time_max = INF;
    root_node->current_point = start_loc;
    root_node->parent = NULL;
    dij_open.push(root_node);
    while (!dij_open.empty()){
        std::shared_ptr<Node> s = dij_open.top();
        dij_open.pop();
        std::unordered_map<int, std::shared_ptr<Node>>::const_iterator close_item_it;
        close_item_it = close_set.find(s->current_point);
        if (close_item_it != close_set.end()) {
            // printf("find duplicate node!\n");
            if (close_item_it->second->f <= s->f){
                continue;
            } else {
                close_set.erase(close_item_it);
                curr_agent_heuristic[s->current_point] = s;
                close_set.insert(std::pair<int, std::shared_ptr<Node>>(s->current_point, s));
            }
        } else {
            curr_agent_heuristic[s->current_point] = s;
            close_set.insert(std::pair<int, std::shared_ptr<Node>>(s->current_point, s));
        }
        // printf("pop node loc: %d, with f: %f\n", s->current_point, s->f);

        // For all the neighbor location, all need to do this operation
        std::list<int> neighbors = instance_ptr->getNeighbors(s->current_point);
        for (int neighbor: neighbors) {
            if (neighbor == s->current_point) {
                continue;
            }
            std::shared_ptr<Node> successor_node = std::make_shared<Node>();
            successor_node->current_point = neighbor;
            successor_node->arrival_time_min = s->arrival_time_min + CELL_DIS/curr_agent.v_max;
            successor_node->arrival_time_max = INF;
            successor_node->interval_index = -1;
            successor_node->g = s->arrival_time_min;
            successor_node->h = 0;
            successor_node->f = successor_node->g + successor_node->h;
            successor_node->parent = s;
            dij_open.push(successor_node);
        }

    }
    // printf("finish Dij for agent %d\n", curr_agent.id);
    heuristic_vec[curr_agent.id] = curr_agent_heuristic;
    return false;
}

bool SIPP::NodeSearch(std::shared_ptr<Node> start_node)
{
    assert(start_node->current_point != curr_agent.goal_location);
    std::shared_ptr<Node> node_it = heuristic_vec[curr_agent.id][start_node->current_point];
    std::shared_ptr<Node> successor_node;
    std::shared_ptr<Node> prev_node = start_node;
    while(node_it->parent != NULL) {
        node_it = node_it->parent;
        successor_node = std::make_shared<Node>();
        successor_node->current_point = node_it->current_point;
        successor_node->arrival_time_min = prev_node->arrival_time_min + CELL_DIS/curr_agent.v_max;
        successor_node->arrival_time_max = INF;
        successor_node->interval_index = -1;
        successor_node->g = successor_node->arrival_time_min;
        successor_node->h = node_it->arrival_time_min;
        successor_node->f = successor_node->g + successor_node->h;
        successor_node->parent = prev_node;
        prev_node = successor_node;
    }
    open.push(successor_node);
    return true;
}

bool SIPP::AStarSearch(std::shared_ptr<Node> root_node)
{
    std::priority_queue<std::shared_ptr<Node>, std::vector<std::shared_ptr<Node>>, NodeCompare> astar_open;
    std::unordered_map<int, std::shared_ptr<Node>> close_set;
    astar_open.push(root_node);
    while (!astar_open.empty()){
        std::shared_ptr<Node> s = astar_open.top();
        astar_open.pop();
        std::unordered_map<int, std::shared_ptr<Node>>::const_iterator close_item_it;
        if (close_item_it != close_set.end()) {
            if (close_item_it->second->f <= s->f){
                continue;
            } else {
                close_set.erase(close_item_it);
                close_set.insert(std::pair<int, std::shared_ptr<Node>>(s->current_point, s));
            }
        } else {
            close_set.insert(std::pair<int, std::shared_ptr<Node>>(s->current_point, s));
        }

        if (s->current_point == curr_agent.goal_location){
            open.push(s);
            return true;
        }
        else{
            // For all the neighbor location, all need to do this operation
            std::list<int> neighbors = instance_ptr->getNeighbors(s->current_point);
            for (int neighbor: neighbors) {
                std::shared_ptr<Node> successor_node = std::make_shared<Node>();
                successor_node->current_point = neighbor;
                successor_node->arrival_time_min = s->arrival_time_min + CELL_DIS/curr_agent.v_max;
                successor_node->arrival_time_max = INF;
                successor_node->interval_index = -1;
                successor_node->g = s->arrival_time_min;
                successor_node->h = DistHeuristic(neighbor);
                successor_node->f = successor_node->g + successor_node->h;
                successor_node->parent = s;
                astar_open.push(successor_node);
            }
        }
    }
    return false;
}

// TO DO: change this structure to unordered map to improve efficiency
bool SIPP::RemoveOpenNode(std::shared_ptr<Node> rm_node)
{
    // for (unsigned int i = 0; i < open.size(); i++){
    //     if (open[i] == rm_node) {
    //         open.erase(open.begin() + i);
    //         return true;
    //     }
    // }
    return false;
}

bool SIPP::RemoveAllTable(int location, std::shared_ptr<Node> rm_node)
{
    for (unsigned int i = 0; i < all_nodes_table[location].size(); i++){
        if (all_nodes_table[location][i] == rm_node) {
            all_nodes_table[location].erase(all_nodes_table[location].begin() + i);
            return true;
        }
    }
    return false;
}

void PrintAllNodes(std::shared_ptr<Node> tmp_node)
{
    while(tmp_node->current_point!=-1){
        printf("[INFO] Visit node at location %d, with time interval id: %d\n", tmp_node->current_point, tmp_node->interval_index);
        tmp_node = tmp_node->parent;
    }
}

bool SIPP::earlyPruning(std::shared_ptr<Node> res)
{
    result_nodes.clear();
    while (true){
        result_nodes.insert(result_nodes.begin(), res);
        if (res->parent->current_point  == -1){
            break;
        }
        res = res->parent;
    }

    bool find_entry_success = cache_ptr->FindEntry(result_nodes, curr_agent.init_velocity);
    if (find_entry_success) {
        // printf("[INFO] Hit the cache!\n");
        return false;
    } else {
        BezierNode tmp_bezier_node;
        Path result_path;
        bool success = RecurrentSolver(tmp_bezier_node, result_path);
        if (!success) {
            cache_ptr->InsertEntry(result_nodes, curr_agent.init_velocity);
        }
        return success;
    }
}

bool SIPP::checkDuplicate(std::shared_ptr<Node> new_node)
{
    vector<std::shared_ptr<Node>> tmp_nodes;
    std::shared_ptr<Node> res = new_node;
    while (true){
        tmp_nodes.insert(tmp_nodes.begin(), res);
        if (res->parent->current_point  == -1){
            break;
        }
        res = res->parent;
    }
    bool find_dup = duplicate_table[new_node->current_point].FindEntry(tmp_nodes, curr_agent.init_velocity);
    // printf("Find duplicate: %d\n", find_dup);
    if (find_dup) {
        return false;
    } else {
        duplicate_table[new_node->current_point].InsertEntry(tmp_nodes, curr_agent.init_velocity);
        return true;
    }
}

bool SIPP::dominanceCheck(std::shared_ptr<Node> new_node)
{
    // PrintAllNodes(new_node);
    // int curr_interval_id = new_node->interval_index;
    // int curr_loc = new_node->current_point;
    // std::shared_ptr<Node> tmp_node = new_node->parent;
    // while (tmp_node->current_point != -1) {
    //     if (curr_loc == tmp_node->current_point and curr_interval_id == tmp_node->interval_index) {
    //         // printf("SIPP::dominanceCheck False\n");
    //         return false;
    //     }
    //     tmp_node = tmp_node->parent;
    // }
    if (!checkDuplicate(new_node)) {
        return false;
    }
    // if (!earlyPruning(new_node)){
    //     return false;
    // }
    // printf("SIPP::dominanceCheck True\n");
    return true;
}

bool SIPP::outWindowDominanceCheck(std::shared_ptr<Node> new_node)
{
    std::vector<std::shared_ptr<Node>> old_nodes;
    // find_point(new_node->current_point, old_nodes);
    old_nodes = all_nodes_table[new_node->current_point];
    if (old_nodes.size() == 0) {
        // printf("[INFO] No old node from open list\n");
        return true;
    } else{
        for (std::shared_ptr<Node> old_node: old_nodes) {
            if (new_node->arrival_time_min >= old_node->arrival_time_min and 
                new_node->arrival_time_max <= old_node->arrival_time_max) {
                return false;
            } 
            else if (
                new_node->arrival_time_min <= old_node->arrival_time_min and 
                new_node->arrival_time_max >= old_node->arrival_time_max
            ) {
                // if (old_node->color == 1) {
                    //To do: remove it from open
                    // old_node->color = 0;
                RemoveAllTable(new_node->current_point, old_node);
                RemoveOpenNode(old_node);
                // }
                // printf("[WARN] Need to remove the old node from open list\n");
                // return true;
            }
            //  else if (
            //     new_node->parent->current_point == old_node->parent->current_point and
            //     new_node->arrival_time_min <= old_node->arrival_time_max and 
            //     new_node->arrival_time_max >= old_node->arrival_time_min
            // ) {
            //     if (old_node->arrival_time_min < new_node->arrival_time_min) {
            //         old_node->arrival_time_max = new_node->arrival_time_min;
            //         old_node->arrival_time_max = new_node->arrival_time_min;

            //     } else {
            //         new_node->arrival_time_max = old_node->arrival_time_min;

            //     }
            //     // printf("[INFO] Valid overlap nodes\n");
            //     // return true;
            // }
            // else {
            //     // printf("Valid new point!!!\n");
            //     // printf("Old_Node: The arr time is %f, the leave time is %f\n", old_node->arrival_time_min,
            //     // old_node->arrival_time_max);
            //     // printf("New_Node: The arr time is %f, the leave time is %f\n", new_node->arrival_time_min,
            //     // new_node->arrival_time_max);
            //     // return true;
            //     ;
            // }
        }
    }
    return true;
}

// bool SIPP::dominanceCheck(std::shared_ptr<Node> new_node)
// {
//     std::vector<std::shared_ptr<Node>> old_nodes;
//     // find_point(new_node->current_point, old_nodes);
//     old_nodes = all_nodes_table[new_node->current_point];
//     if (old_nodes.size() == 0) {
//         // printf("[INFO] No old node from open list\n");
//         return true;
//     } else{
//         for (std::shared_ptr<Node> old_node: old_nodes) {
//             if (new_node->interval_index == old_node->interval_index and
//                 new_node->arrival_time_min >= old_node->arrival_time_min and 
//                 new_node->arrival_time_max <= old_node->arrival_time_max) {
//                 return false;
//             } 
//             // else if (
//             //     new_node->parent->current_point == old_node->parent->current_point and
//             //     new_node->arrival_time_min <= old_node->arrival_time_min and 
//             //     new_node->arrival_time_max >= old_node->arrival_time_max
//             // ) {
//             //     // if (old_node->color == 1) {
//             //         //To do: remove it from open
//             //         // old_node->color = 0;
//             //     RemoveAllTable(new_node->current_point, old_node);
//             //     RemoveOpenNode(old_node);
//             //     // }
//             //     // printf("[WARN] Need to remove the old node from open list\n");
//             //     // return true;
//             // } else if (
//             //     new_node->parent->current_point == old_node->parent->current_point and
//             //     new_node->arrival_time_min <= old_node->arrival_time_max and 
//             //     new_node->arrival_time_max >= old_node->arrival_time_min
//             // ) {
//             //     if (old_node->arrival_time_min < new_node->arrival_time_min) {
//             //         old_node->arrival_time_max = new_node->arrival_time_min;
//             //         old_node->arrival_time_max = new_node->arrival_time_min;

//             //     } else {
//             //         new_node->arrival_time_max = old_node->arrival_time_min;

//             //     }
//             //     // printf("[INFO] Valid overlap nodes\n");
//             //     // return true;
//             // }
//             // else {
//             //     // printf("Valid new point!!!\n");
//             //     // printf("Old_Node: The arr time is %f, the leave time is %f\n", old_node->arrival_time_min,
//             //     // old_node->arrival_time_max);
//             //     // printf("New_Node: The arr time is %f, the leave time is %f\n", new_node->arrival_time_min,
//             //     // new_node->arrival_time_max);
//             //     // return true;
//             //     ;
//             // }
//         }
//     }
//     return true;
// }

std::list<TimeInterval> SIPP::get_successors( 
        std::shared_ptr<Node> s,
        int to_location,
        const ReservationTable& rt)
{
    debug_rt_location_ = to_location;
    std::list<TimeInterval> rst;
    double m_time_min = (CELL_DIS/2 - curr_agent.length)/curr_agent.v_max;
    // printf("get_successors::Time current max time: %f, CELL distance is %f\n", curr_agent.v_max, CELL_DIS);
    // start_t = time(s) + m_time
    double lower_bound;
    if (s->arrival_time_min == curr_agent.earliest_start_time) {
        lower_bound = s->arrival_time_min + m_time_min;
    } else {
        lower_bound = s->arrival_time_min + CELL_DIS/curr_agent.v_max;
    }
    // ent_t = endTime(interval(s)) + m_time + time for vehicle to pass m
    // printf("get_successors::Time bound for NODE s is: %f to %f\n", s.arrival_time_min, s.arrival_time_max);
    
    double upper_bound = INF;
    // Check if the safe interval comes in time order (not ordered)
    std::list<TimeInterval> interval_list = getSafeIntervals(rt[to_location]);
    // bool get_interval_success = getSafeIntervals(rt[to_location], interval_list);

    if (debug_rt_location_ == DEBUG_LOC)
        std::cout << "get_successors::getSafeInterval size: " << interval_list.size() << std::endl;
    // printf("Time bound is: %f to %f\n", lower_bound, upper_bound);
    int interval_id = 0;
    for (TimeInterval interval: interval_list) {
        interval_id++;
        if (debug_rt_location_ == DEBUG_LOC)
            printf("SIPP::GetsafeInterval Time interval is: %f to %f\n", interval.t_min, interval.t_max);
        if (lower_bound >= interval.t_max)
            continue;
        else if (upper_bound <= interval.t_min)
            continue;
        // the interval overlaps with [lower_bound, upper_bound)
        TimeInterval tmp_interval;
        tmp_interval.t_min = max(lower_bound, interval.t_min);
        tmp_interval.t_max = min(upper_bound, interval.t_max);
        tmp_interval.id = interval_id;
        rst.emplace_back(tmp_interval);
    }
    return rst;
}

int SIPP::find_min(vector<std::shared_ptr<Node>>& open){
    int min_index = 0;
    double min = open[min_index]->f;
    for (unsigned int i = 0; i < open.size(); i++)
    {
        if (open[i]->f < min)
        {
            min_index = i;
            min = open[i]->f;
        }
    }
    // printf("Return the min index\n");
    return min_index;
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
        for (unsigned int sfIter = 0; sfIter < safe_intervals.size(); sfIter++){
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

bool SIPP::getSafeIntervals(list<TimeInterval> localRT, vector<TimeInterval>& safe_intervals)
{
    if (localRT.size() == 0){
        TimeInterval aInterval;
        aInterval.t_min = 0;
        aInterval.t_max = INF;
        safe_intervals.push_back(aInterval);
        if (debug_rt_location_ == DEBUG_LOC) {
            printf("No entry found!\n");
        }

        return true;
    }
    localRT.sort([](const TimeInterval &f, const TimeInterval &s) {return f.t_min < s.t_min; });
    list<TimeInterval>::iterator prev_it = localRT.begin();
    if (prev_it->t_min > 0){
        TimeInterval aInterval_1;
        aInterval_1.t_min = 0;
        aInterval_1.t_max = prev_it->t_min;
        safe_intervals.push_back(aInterval_1);
    }
    for (list<TimeInterval>::iterator curr_it = std::next(localRT.begin()); curr_it!= localRT.end(); ++curr_it){
        assert(prev_it->t_max <= curr_it->t_min);
        TimeInterval tmp_interval;
        tmp_interval.t_min = prev_it->t_max;
        tmp_interval.t_max = curr_it->t_min;
        safe_intervals.push_back(tmp_interval);
        prev_it = curr_it;
    }
    if (localRT.back().t_max < INF){
        TimeInterval aInterval_2;
        aInterval_2.t_min = localRT.back().t_max;
        aInterval_2.t_max = INF;
        safe_intervals.push_back(aInterval_2);
    }
    if (debug_rt_location_ == DEBUG_LOC) {
        printf("Found multiple entry!\n");
    }
    return true;
}
