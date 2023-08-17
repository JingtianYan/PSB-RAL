/**
 * Implementation of the cache for the MILP solver
 * Author: Jingtian Yan
*/
#pragma once

#include <iostream>
#include <vector>
#include <utility>

#include "Instance.h"

// #define EntryKey std::vector<std::pair<double, double>>

struct EntryKey{
    std::vector<int> traj;
    std::vector<std::pair<double, double>> interval;
    double init_v;
};

struct CacheEntry{
    EntryKey keys;
    int traj_length;
    bool IsWithinKey(EntryKey query_key);
    bool IsEqual(EntryKey query_key);
    BezierNode bezier_val;
    Path path_val;
};

class CacheMILP{
public:
    CacheMILP();
    bool InsertEntry(std::vector<std::shared_ptr<Node>> &new_path, double init_speed);
    bool InsertEntry(std::vector<std::shared_ptr<Node>> &new_path, double init_speed, BezierNode& bezier_solution,Path& solution_path);
    bool RetriveEntry(std::vector<std::shared_ptr<Node>> &new_path, double init_speed, BezierNode& tmp_solution, Path& result_path);
    bool RemoveEntry(EntryKey rm_entry);
    bool FindEntry(std::vector<std::shared_ptr<Node>> &new_path, double init_speed);
    // bool FindEqualEntry(std::vector<std::shared_ptr<Node>> &new_path);
    bool PathToKey(std::vector<std::shared_ptr<Node>> &path, double start_speed, EntryKey &key);
    void ShowEntry(CacheEntry entry);
    bool reset();


private:
    std::deque<CacheEntry> cache_table;
    unsigned long long cache_size;
};