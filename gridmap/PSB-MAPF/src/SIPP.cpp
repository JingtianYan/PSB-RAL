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
    cache_ptr = std::make_shared<CacheMILP>();
    solution_cache_ptr = std::make_shared<CacheMILP>();
    heuristic_vec.resize(instance_ptr->agents.size());
}

bool SearchInterval(double begin_time, double end_time, double& block_start, double& block_end, BernsteinPolynomial& bern)
{
    double STEP_SIZE = 0.3;
    block_start = begin_time;
    while(true) {
        if (true) {
            break;
        }
    }
    block_start -= STEP_SIZE;
    block_end = block_start;
    while(true) {
        if (true) {
            break;
        }
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
            bern_poly.CalculateVal(arr_max, leave_bern_val);
            for (int j = 0; j < n_points; j++) {
                leave_place_expr += control_points[j]*leave_bern_val[j];
            }
            model.add(leave_place_expr >= (i*CELL_DIS + CELL_DIS/2 + curr_agent.length));
            leave_place_expr.end();
        }
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
    auto startTime = std::chrono::high_resolution_clock::now();
    if(cplex.solve()) {
        auto stopTime = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stopTime - startTime);
        total_runtime_ += duration.count()/1000000.0;
        for (int i = 0; i < n_points; i++) {
            solution_control_points.push_back(cplex.getValue(control_points[i]));
        }

        Path potential_path;
        for (int i = 0; i < traj_size; i++){
            PathEntry a_path;
            // Round all the time slot to x.1
            a_path.location = result_nodes[i]->current_point;
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
                    exit(-1);
                }
                a_path.leaving_time_tail = block_end;
            }

            potential_path.push_back(a_path);
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
    double t_length = T;

    BernsteinPolynomial bern_poly(n_points, t_length);
    // First order derivative of bernstein polynomial
    BernsteinPolynomial first_order_deriv = bern_poly.Derivative();
    // Second order derivative of bernstein polynomial
    BernsteinPolynomial second_order_deriv = first_order_deriv.Derivative();

    IloEnv env = IloEnv();
    IloModel model = IloModel(env);
    IloExpr sum_obj = IloExpr(env);
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
            bern_poly.CalculateVal(arr_max, leave_bern_val);
            for (int j = 0; j < n_points; j++) {
                leave_place_expr += control_points[j]*leave_bern_val[j];
            }
            // std::cout << std::endl;
            model.add(leave_place_expr >= (i*CELL_DIS + CELL_DIS/2 + curr_agent.length - control_points[n_points]));
            leave_place_expr.end();
        }
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

    auto startTime = std::chrono::high_resolution_clock::now();
    if(cplex.solve()) {
        auto stopTime = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stopTime - startTime); 
        
        total_runtime_ += duration.count()/1000000.0;
        slack_var = cplex.getValue(control_points[n_points]);
        env.end();
        return true;
    } else {
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
    SolveSlackBezier(target_t, solution_control_points, result_path, slack_t);
    double slack_delta;
    SolveSlackBezier(target_t+delta, solution_control_points, result_path, slack_delta);
    gradient = (slack_delta - slack_t)/delta;
    if (slack_t == 0){
        return true;
    } else {
        return false;
    }
}

bool SIPP::RecurrentSolver(BezierNode& solution, Path& result_path)
{
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
        optimal_T = lower_bound;
    } else{
        while (upper_bound < INF) {
            upper_bound = 2 * upper_bound;
            upper_valid_solution = SlackGradient(upper_bound, gradient_t_high);
            if (gradient_t_high > 0 or upper_valid_solution) {
                optimal_T = upper_bound;
                upper_in_bould = true;
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
            mid = (upper_bound + lower_bound)/2;
            double slack;
            SolveSlackBezier(mid, solution.control_points, result_path, slack);
            if (slack == 0) {
                upper_bound = mid;
                optimal_T = mid;
                t_high_valid = true;
            } else {
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
        }
    }

    if (!t_high_valid) {
        return false;
    }
    
    solution.control_points.clear();
    bool success = SolveBezier(optimal_T, solution.control_points, result_path);
    if (success){
        solution.optimal_T = optimal_T;
        return true;
    } else {
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
    std::shared_ptr<Node> init_null_interval = std::make_shared<Node> ();
    init_null_interval->current_point = -1;   
    first_interval = std::make_shared<Node> ();
    vector<TimeInterval> safe_intervals;
    list<TimeInterval> localRT = rt[curr_agent.start_location];
    
    localRT.sort([](const TimeInterval &f, const TimeInterval &s) { return f.t_min < s.t_min; });
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
    if (heuristic_vec[agentID].empty()) {
        Dijstra(curr_agent.goal_location);
    }
    if (!duplicate_table.empty()) {
        for (auto tmp_cache: duplicate_table) {
            tmp_cache.reset();
        }
    }
    
    duplicate_table.clear();
    duplicate_table.resize(instance_ptr->GetNumOfVertices());

    Reset();
    

    count_called++;

    if (curr_agent.goal_location == curr_agent.start_location) {
        path.clear();
        return true;
    }

    solution.optimal_T = INF;
    std::vector<double> tmp_control_points;

    std::shared_ptr<Node> init_node;
    bool first_interval_free = GetFirstInterval(rt, init_node);
    if (!first_interval_free) {
        path.clear();
        return false;
    }
    all_nodes_table[init_node->current_point].push_back(init_node);

    open.push(init_node);

    Path optimal_result_path;
    optimal_result_path.clear();
    Path result_path;
    BezierNode tmp_solution;
    auto start_time = Time::now();
    while (!open.empty()){
        //    remove s with the smallest f-value from OPEN
        auto tmp_end_time = Time::now();
        std::chrono::duration<float> tmp_duration = tmp_end_time - start_time;
        if (tmp_duration.count() > cutoff_time){
            break;
        }
        std::shared_ptr<Node> s = open.top();
        open.pop();
        if (optimal_result_path.size() > 0){
            if (s->f > optimal_result_path.back().arrival_time){
                // return result_path;
                break;
            }
        }
        if (s->current_point == curr_agent.goal_location){
            if (s->arrival_time_max == INF) {
                updateResultNodes(s);
                bool find_entry_success = cache_ptr->FindEntry(result_nodes, curr_agent.init_velocity);
                result_path.clear();
                bool find_solution_success = solution_cache_ptr->RetriveEntry(result_nodes, curr_agent.init_velocity, tmp_solution, result_path);
                if (find_entry_success) {
                    continue;
                } else if(find_solution_success) {
                    ;
                } else {
                    bizer_count_++;
                    bool success = RecurrentSolver(tmp_solution, result_path);
                    if (success) {
                        solution_cache_ptr->InsertEntry(result_nodes, curr_agent.init_velocity, tmp_solution, result_path);
                    } else {
                        cache_ptr->InsertEntry(result_nodes, curr_agent.init_velocity);
                    }
                }

                if (tmp_solution.optimal_T < solution.optimal_T) {
                    solution.optimal_T = tmp_solution.optimal_T;
                    solution.control_points = tmp_solution.control_points;
                    for (unsigned int i = 0; i < result_path.size(); i++) {
                        result_path[i].arrival_time = result_path[i].arrival_time + curr_agent.earliest_start_time;
                        result_path[i].leaving_time_tail = result_path[i].leaving_time_tail + curr_agent.earliest_start_time;
                        result_path[i].location = result_nodes[i]->current_point;
                    }
                    optimal_result_path = result_path;
                    instance_ptr->agents[curr_agent.id].tmp_cost = result_path.back().leaving_time_tail 
                        + heuristic_vec[agentID][result_path.back().location]->arrival_time_min;
                }
            }
        }
        else{
            // For all the neighbor location, all need to do this operation
            std::list<int> neighbors = instance_ptr->getNeighbors(s->current_point);
            for (int neighbor: neighbors) {
                if (neighbor == s->parent->current_point) {
                    // omit the back and forth case
                    continue;
                }

                if (s->arrival_time_min > WINDOWS_SIZE) {
                    NodeSearch(s);
                } else {
                    std::list<TimeInterval> neighbor_successors = get_successors(s, neighbor, rt);
                    for (TimeInterval neighbor_interval: neighbor_successors){
                        std::shared_ptr<Node> successor_node = std::make_shared<Node>();
                        successor_node->current_point = neighbor;
                        successor_node->arrival_time_min = neighbor_interval.t_min;
                        successor_node->arrival_time_max = neighbor_interval.t_max;
                        successor_node->interval_index = neighbor_interval.id;
                        successor_node->g = neighbor_interval.t_min;
                        successor_node->h = heuristic_vec[agentID][neighbor]->arrival_time_min;
                        successor_node->f = successor_node->g + successor_node->h;
                        successor_node->parent = s;

                        open.push(successor_node);
                        all_nodes_table[successor_node->current_point].push_back(successor_node);
                    }
                }
            }
        }
    }
    if (optimal_result_path.size() == 0){
        path.clear();
        return false;
    }else{
        path.clear();
        path = optimal_result_path;
        if (solution.optimal_T < REPLAN_SIZE) {
            path.back().leaving_time_tail = INF;
        }
        return true;
    }
    
}

bool SIPP::Dijstra(int start_loc)
{
    std::vector<std::shared_ptr<Node>> curr_agent_heuristic;
    curr_agent_heuristic.resize(instance_ptr->map_size);
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
    if (find_dup) {
        return false;
    } else {
        duplicate_table[new_node->current_point].InsertEntry(tmp_nodes, curr_agent.init_velocity);
        return true;
    }
}

bool SIPP::dominanceCheck(std::shared_ptr<Node> new_node)
{
    if (!checkDuplicate(new_node)) {
        return false;
    }
    return true;
}

std::list<TimeInterval> SIPP::get_successors( 
        std::shared_ptr<Node> s,
        int to_location,
        const ReservationTable& rt)
{
    debug_rt_location_ = to_location;
    std::list<TimeInterval> rst;
    double m_time_min = (CELL_DIS/2 - curr_agent.length)/curr_agent.v_max;
    double lower_bound;
    if (s->arrival_time_min == curr_agent.earliest_start_time) {
        lower_bound = s->arrival_time_min + m_time_min;
    } else {
        lower_bound = s->arrival_time_min + CELL_DIS/curr_agent.v_max;
    }
    double upper_bound = INF;
    // Check if the safe interval comes in time order (not ordered)
    std::list<TimeInterval> interval_list = getSafeIntervals(rt[to_location]);
    if (debug_rt_location_ == DEBUG_LOC)
        std::cout << "get_successors::getSafeInterval size: " << interval_list.size() << std::endl;
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
