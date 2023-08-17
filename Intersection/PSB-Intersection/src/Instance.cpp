/**
 * This file is part of the PBS with TPG
 * 
 * In this file, the instance class is defined which load the data and configuration from the file
 */
#include "../include/Instance.h"

/**
 * The constructor of Instance
 * 
 * Load the search graph, sample points and order of vehicles from the given file
 * 
 * @param step
 * @param map_file
 * @param pair_distance_file
 * @param vehicle_file
*/
Instance::Instance(int step, const string& map_file, const string& pair_distance_file, const string& vehicle_file)
{
    _graph_name = map_file;
    _pair_distance_filename = pair_distance_file;
    _vehicle_arrival_filename = vehicle_file;
    _step = step;

    loadSearchGraph();
    // std::cout << "Instance: graph loaded\n";
    loadSamePoint();
    // std::cout << "Instance: sample points loaded\n";
    loadVehicles();
    // std::cout << "Instance: vehicle loaded\n";
}

int Instance::getNumOfVertices() const
{
    return vNameToID.size();
}

/**
 * Load the order and time of the vehicles
 * 
 * @param NULL
*/
void Instance::loadVehicles()
{
    // non-Windows use "r"
    FILE* fp = fopen(_vehicle_arrival_filename.c_str(), "r");
    char readBuffer[65536];
    rapidjson::FileReadStream is(fp, readBuffer, sizeof(readBuffer));
    rapidjson::Document doc;
    doc.ParseStream(is);
    fclose(fp);

    agents.clear();
    int agentId = 0;
    for (int i = 0; i <= _step; i++){
        std::string IDString = std::to_string(i);
        Agent ag = {};
        ag.id = agentId;
        ag.name = std::to_string(doc[IDString.c_str()]["id"].GetInt());
        ag.importance = (doc[IDString.c_str()]["importance"].GetInt());
        ag.earliest_start_time = doc[IDString.c_str()]["arrivalTime"].GetDouble();
        auto trajectoryArray = doc[IDString.c_str()]["trajectory"].GetArray();
        // printf("The length of trajectory: %d\n", trajectoryArray.Size());
        ag.length = 6;
        ag.v_min = 3;
        
        ag.a_min = -2;
        ag.a_max = 5;
        if (trajectoryArray.Size() == 2) {
            ag.v_max = 5;
        } else {
            ag.v_max = 15;            
        }
        for (rapidjson::Value::ConstValueIterator itr2 = trajectoryArray.Begin(); itr2 != trajectoryArray.End(); ++itr2){
            ag.trajectory.push_back(vNameToID.find((*itr2).GetString())->second);
        }
        agents.push_back(ag);
        agentId++;
    }
}

/**
 * Load the search graph from the given file
 * 
 * Load the structure of the intersection and the pair distance of the intersection. Save those
 * information into the search graph and distance graph.
 * 
 * @param NULL
*/
void Instance::loadSearchGraph()
{
    FILE* fp = fopen(_graph_name.c_str(), "r"); // non-Windows use "r"
    char readBuffer[65536];

    rapidjson::FileReadStream is(fp, readBuffer, sizeof(readBuffer));
    rapidjson::Document doc;
    doc.ParseStream(is);
    fclose(fp);
    
    int vertex_index = 0;
    // add vertices
    for (rapidjson::Value::ConstValueIterator itr = doc["vertices"].Begin(); itr != doc["vertices"].End(); ++itr) {
        if (itr->HasMember("name")) {
            position_t pos = nodeAsPos((*itr)["pos"][0].GetDouble(),(*itr)["pos"][1].GetDouble());
            std::string name = (*itr)["name"].GetString();
            auto v = boost::add_vertex(searchGraph);
            searchGraph[v].name = name;
            searchGraph[v].pos = pos;
            searchGraph[v].id = vertex_index;
            vNameToID[name] = vertex_index;
            vIDToName[vertex_index] = name;
            vNameToV.push_back(v);

            // find the direction, where 2 mean straight direction, 1 mean turning right direction, and 0 mean turning left
            if ((name.find("EW")!= std::string::npos) || (name.find("WE")!= std::string::npos) || 
                    (name.find("NS")!= std::string::npos) || (name.find("SN")!= std::string::npos)) {
                vNameToDirection.push_back(2);
            } else if ((name.find("EN")!= std::string::npos) || (name.find("WS")!= std::string::npos) || 
                    (name.find("NW")!= std::string::npos) || (name.find("SE")!= std::string::npos)) {
                vNameToDirection.push_back(1);
            } else {
                vNameToDirection.push_back(0);
            }
            vertex_index++;
        }
    }

    //add vertex conflicts
    for (rapidjson::Value::ConstValueIterator itr = doc["vertices"].Begin(); itr != doc["vertices"].End(); ++itr) {
        if (itr->HasMember("name")) {
            std::string name = (*itr)["name"].GetString();
            auto iter = vNameToID.find(name);
            int v_idx = iter->second;
            vertex_t v = vNameToV[v_idx];

            for (rapidjson::Value::ConstValueIterator node = (*itr)["vertex_conflicts"].Begin(); node != (*itr)["vertex_conflicts"].End(); ++node) 
            {
                std::string cName = node->GetString();
                auto cIter = vNameToID.find(cName);
                if (cIter == vNameToID.end()) 
                {
                    std::cerr << "(2) ERROR: Could not find vertex " << cName << std::endl;
                    continue;
                }
                int u = cIter->second;
                searchGraph[v].generalizedVertexConflicts.insert(u);
                vIDToConflictPoints[v_idx].push_back(u);
            }
        }
    }


    //read pair distance
    FILE* pdfile = fopen(_pair_distance_filename.c_str(), "r"); // non-Windows use "r"
    char readBuffer2[65536];
    rapidjson::FileReadStream is2(pdfile, readBuffer2, sizeof(readBuffer2));
    pairDistances.ParseStream(is2);
    fclose(pdfile);

    for (rapidjson::Value::ConstMemberIterator itr = pairDistances.MemberBegin(); itr != pairDistances.MemberEnd(); ++itr)
    {
        std::string first_vertex_string = itr->name.GetString();
        auto first_vertex = vNameToID.find(first_vertex_string);
        int first_vertex_id = first_vertex->second;        
        auto second_vertex_list = itr->value.GetObject();
        for (rapidjson::Value::ConstMemberIterator itr2 = second_vertex_list.MemberBegin();
                    itr2 != second_vertex_list.MemberEnd(); ++itr2)
        {
            std::string second_vertex_string = itr2->name.GetString();
            auto second_vertex = vNameToID.find(second_vertex_string);
            int second_vertex_id = second_vertex->second;
            pairDistancesMap[first_vertex_id][second_vertex_id] = itr2->value.GetDouble();      
        }
    }
}

/**
 * Find the points in the graph that at the same position
 * 
 * @param NULL
*/
void Instance::loadSamePoint(){
    for(int i = 0; i < 104; i ++){
        samePoint[i][i] = true;
        std::vector<int> cps = getConflictPoints(i);
        for(auto it = cps.begin(); it != cps.end(); ++it){
            samePoint[i][*it] = true;
        }
    }
}

/**
 * Determine if two points are at the same position
 * 
 * @param a index of the first point
 * @param b index of the second point
 * @return bool value of is this is a same point
*/
bool Instance::isSamePoint(int a, int b){
    return samePoint[a][b];
}

std::vector<int> Instance::getConflictPoints(int aid){
    return vIDToConflictPoints[aid];
}

position_t Instance::nodeAsPos(const float x, const float y)
{  
    return position_t(x,y);
}

std::map<int, std::map<int, double> > Instance::getPairDistancesMap(){
    return pairDistancesMap;
}

std::vector<int> Instance::getVNameToDirection(){
    return vNameToDirection;
}

std::unordered_map<std::string, int> Instance::getVNameToID(){
    return vNameToID;
}

std::vector<Agent> Instance::getAgents(){
    return agents;
}

std::unordered_map<int, std::vector<int> > Instance::getVIDToConflictPoints(){
    return vIDToConflictPoints;
}

std::unordered_map<int, std::string> Instance::getVIDToName(){
    return vIDToName;
}

double Instance::getEarliestStartTime(int index){
    return agents[index].earliest_start_time;
}

int Instance::getStartCP(int index){
    return agents[index].trajectory[0];
}