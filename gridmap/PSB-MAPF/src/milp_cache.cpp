#include "milp_cache.h"
#include <boost/lexical_cast/try_lexical_convert.hpp>
#include <utility>
#include <vector>

bool CacheEntry::IsWithinKey(EntryKey query_key)
{
    bool flag = true;
    if (query_key.traj.size() != keys.traj.size() or query_key.init_v != keys.init_v) {
        flag = false;
    } else {
        for(unsigned int i = 0; i < query_key.traj.size(); i++) {
            if (query_key.interval[i].first < keys.interval[i].first ||
                query_key.interval[i].second > keys.interval[i].second) {
                    // printf("CacheEntry::IsWithinKey out of the range. Query: %f, %f; Key: %f, %f!\n",
                    //     query_key.interval[i].first, query_key.interval[i].second, keys.interval[i].first,
                    //     keys.interval[i].second);
                    flag = false;
                    break;
                }
        }
    }
    return flag;
}

bool CacheEntry::IsEqual(EntryKey query_key)
{
    bool flag = true;
    if (query_key.traj.size() != keys.traj.size() or query_key.init_v != keys.init_v) {
        flag = false;
    } else {
        for(unsigned int i = 0; i < query_key.traj.size(); i++) {
            if (query_key.interval[i].first != keys.interval[i].first ||
                query_key.interval[i].second != keys.interval[i].second) {
                    // printf("CacheEntry::IsWithinKey out of the range. Query: %f, %f; Key: %f, %f!\n",
                    //     query_key.interval[i].first, query_key.interval[i].second, keys.interval[i].first,
                    //     keys.interval[i].second);
                    flag = false;
                    break;
                }
        }
    }
    return flag;
}

CacheMILP::CacheMILP()
{
    cache_size = 9999;
}

bool CacheMILP::reset()
{
    cache_table.clear();
    return true;
}

bool CacheMILP::InsertEntry(
    std::vector<std::shared_ptr<Node>> &new_path, 
    double init_speed,
    BezierNode& bezier_solution,
    Path& solution_path)
{
    
    CacheEntry new_entry;
    new_entry.traj_length = new_path.size();
    new_entry.bezier_val = bezier_solution;
    new_entry.path_val = solution_path;
    PathToKey(new_path, init_speed, new_entry.keys);
    cache_table.push_back(new_entry);
    if (cache_table.size() >= cache_size) {
        cache_table.pop_front();
    }
    // printf("Success Buffer size is: %ld\n", cache_table.size());
    
    // for (int i = 0; i < new_path.size(); i++) {
    //     ;
    // }
    // std::pair<double, double> val;
    // new_entry.keys.push_back(val);
    return true;
}

bool CacheMILP::InsertEntry(std::vector<std::shared_ptr<Node>> &new_path, double init_speed)
{
    
    CacheEntry new_entry;
    new_entry.traj_length = new_path.size();
    PathToKey(new_path, init_speed, new_entry.keys);
    cache_table.push_back(new_entry);
    if (cache_table.size() >= cache_size) {
        cache_table.pop_front();
    }
    // printf("Fail Buffer size is: %ld\n", cache_table.size());
    // for (int i = 0; i < new_path.size(); i++) {
    //     ;
    // }
    // std::pair<double, double> val;
    // new_entry.keys.push_back(val);
    return true;
}

bool CacheMILP::RemoveEntry(EntryKey rm_entry)
{
    return true;
}

void CacheMILP::ShowEntry(CacheEntry entry)
{
    for (int i = 0; i < entry.traj_length; i++) {
        printf("For traj point %d, the time interval is (%f -> %f)\n", 
            entry.keys.traj[i], entry.keys.interval[i].first, entry.keys.interval[i].second);
    }
    printf("\n");
}

bool CacheMILP::FindEntry(std::vector<std::shared_ptr<Node>> &new_path, double init_speed)
{

    EntryKey query_key;
    PathToKey(new_path, init_speed, query_key);
    for(CacheEntry tmp_entry: cache_table) {
        if (tmp_entry.IsWithinKey(query_key)) {
            // printf("Find the fail key:\n");
            // ShowEntry(tmp_entry);
            return true;
        }
    }
    return false;
}

bool CacheMILP::RetriveEntry(
    std::vector<std::shared_ptr<Node>> &new_path,
    double init_speed,
    BezierNode& tmp_solution,
    Path& result_path)
{

    EntryKey query_key;
    PathToKey(new_path, init_speed, query_key);
    for(CacheEntry tmp_entry: cache_table) {
        if (tmp_entry.IsEqual(query_key)) {
            // printf("Find the success key:\n");
            // ShowEntry(tmp_entry);
            tmp_solution = tmp_entry.bezier_val;
            result_path = tmp_entry.path_val;
            return true;
        }
    }
    tmp_solution.optimal_T = INF;
    return false;
}

// bool CacheMILP::FindEqualEntry(std::vector<std::shared_ptr<Node>> &new_path)
// {

//     EntryKey query_key;
//     PathToKey(new_path, query_key);
//     for(CacheEntry tmp_entry: cache_table) {
//         if (tmp_entry.IsEqual(query_key)) {
//             // printf("Find the key:\n");
//             // ShowEntry(tmp_entry);
//             return true;
//         }
//     }
//     return false;
// }

bool CacheMILP::PathToKey(std::vector<std::shared_ptr<Node>> &path, double start_speed, EntryKey &key)
{
    for (unsigned int i = 0; i < path.size(); i++) {
        key.traj.push_back(path[i]->current_point);
        key.interval.push_back(std::pair<double, double> (path[i]->arrival_time_min, path[i]->arrival_time_max));
        key.init_v = start_speed;
    }
    return false;
}


// std::vector<CacheEntry> cache_table;
