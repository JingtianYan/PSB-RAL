#include "milp_cache.h"
#include <boost/lexical_cast/try_lexical_convert.hpp>
#include <utility>
#include <vector>

bool CacheEntry::IsWithinKey(EntryKey query_key)
{
    bool flag = true;
    if (query_key.traj.size() != keys.traj.size()) {
        flag = false;
    } else {
        for(int i = 0; i < query_key.traj.size(); i++) {
            if (query_key.traj[i] != keys.traj[i] || 
                query_key.interval[i].first < keys.interval[i].first ||
                query_key.interval[i].second > keys.interval[i].second) {
                    flag = false;
                    break;
                }
        }
    }
    return flag;
}

CacheMILP::CacheMILP()
{
    cache_size = 99999;
}


bool CacheMILP::InsertEntry(std::vector<Node> &new_path)
{
    
    CacheEntry new_entry;
    new_entry.traj_length = new_path.size();
    PathToKey(new_path, new_entry.keys);
    cache_table.push_back(new_entry);
    return true;
}

bool CacheMILP::RemoveEntry(EntryKey rm_entry)
{
    return true;
}

void CacheMILP::ShowEntry(CacheEntry entry)
{
    for (int i = 0; i < entry.traj_length; i++) {
        printf("For traj point %d, the time interval is (%f -> %f)\t", 
            entry.keys.traj[i], entry.keys.interval[i].first, entry.keys.interval[i].second);
    }
    printf("\n");
}

bool CacheMILP::FindEntry(std::vector<Node> &new_path)
{

    EntryKey query_key;
    PathToKey(new_path, query_key);
    for(CacheEntry tmp_entry: cache_table) {
        if (tmp_entry.IsWithinKey(query_key)) {
            return true;
        }
    }
    return false;
}

bool CacheMILP::PathToKey(std::vector<Node> &path, EntryKey &key)
{
    for (int i = 0; i < path.size(); i++) {
        key.traj.push_back(path[i].current_point);
        key.interval.push_back(std::pair<double, double> (path[i].arrival_time_min, path[i].arrival_time_max));
    }
    return false;
}