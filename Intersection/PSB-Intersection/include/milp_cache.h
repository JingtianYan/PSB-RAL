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
};

struct CacheEntry{
    EntryKey keys;
    int traj_length;
    bool IsWithinKey(EntryKey query_key);
};

class CacheMILP{
public:
    CacheMILP();
    bool InsertEntry(std::vector<Node> &new_path);
    bool RemoveEntry(EntryKey rm_entry);
    bool FindEntry(std::vector<Node> &new_path);
    bool PathToKey(std::vector<Node> &path, EntryKey &key);
    void ShowEntry(CacheEntry entry);

private:
    std::vector<CacheEntry> cache_table;
    long long cache_size;
};