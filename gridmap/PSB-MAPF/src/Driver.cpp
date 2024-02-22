/* Copyright (C) Jiaoyang Li
* Unauthorized copying of this file, via any medium is strictly prohibited
* Confidential
* Written by Jiaoyang Li <jiaoyanl@usc.edu>, March 2021
*/

/*driver.cpp
* Solve a MAPF instance on 2D grids.
*/
#include <boost/program_options.hpp>
#include <boost/tokenizer.hpp>
#include <memory>
#include <string>
#include "Instance.h"
#include "PBS.h"
#include "common.h"


/* Main function */
int main(int argc, char** argv)
{
	namespace po = boost::program_options;
	// Declare the supported options.
	po::options_description desc("Allowed options");
	desc.add_options()
		("help", "produce help message")

		// params for the input instance and experiment settings
		("map,m", po::value<string>()->default_value("./data/Boston_0_256/Boston_0_256.map"), "input file for map")
		("agents,a", po::value<string>()->default_value("./data/Boston_0_256/scen-random/Boston_0_256-random-1.scen"), 
			"input file for agents")
		

		("output,o", po::value<string>()->default_value("./output.txt"), "output file for schedule")
		("statistic,c", po::value<string>()->default_value("./output.csv"), "output file for statistic result")

		("agentNum,k", po::value<int>()->default_value(3), "number of agents")
		("agentIdx", po::value<string>()->default_value(""), "customize the indices of the agents (e.g., \"0,1\")")
		("seed,d", po::value<int>()->default_value(0), "random seed")

		("solver,s", po::value<string>()->default_value("bezier"), "single agent solver for PBS, choose from sipp-ip and bezier")


		("cutoffTime,t", po::value<double>()->default_value(300), "cutoff time (seconds)")

		// params for instance generators
		("rows", po::value<int>()->default_value(0), "number of rows")
		("cols", po::value<int>()->default_value(0), "number of columns")
		("obs", po::value<int>()->default_value(0), "number of obstacles")
		("warehouseWidth", po::value<int>()->default_value(0), "width of working stations on both sides, for generating instances")

		// params for CBS
		("sipp", po::value<bool>()->default_value(false), "using sipp as the single agent solver")
		;

	po::variables_map vm;
	po::store(po::parse_command_line(argc, argv, desc), vm);

	if (vm.count("help"))
	{
		cout << desc << endl;
		return 1;
	}

	po::notify(vm);
	/////////////////////////////////////////////////////////////////////////
	/// check the correctness and consistence of params
    //////////////////////////////////////////////////////////////////////
	

	///////////////////////////////////////////////////////////////////////////
	/// load the instance
    //////////////////////////////////////////////////////////////////////
	std::shared_ptr<Instance> instance_ptr = std::make_shared<Instance>(vm["map"].as<string>(), 
		vm["agents"].as<string>(), vm["agentNum"].as<int>(), vm["agentIdx"].as<string>(),
		vm["rows"].as<int>(), vm["cols"].as<int>(), vm["obs"].as<int>(), vm["warehouseWidth"].as<int>());

	srand(vm["seed"].as<int>());

	//////////////////////////////////////////////////////////////////////
	/// initialize the solver
    //////////////////////////////////////////////////////////////////////
	PBS pbs(instance_ptr, vm["solver"].as<string>(), vm["cutoffTime"].as<double>());
	bool all_agents_solved = false;
	auto global_start_time = Time::now();
	while(!all_agents_solved) {
		all_agents_solved = true;
		bool pbs_success = pbs.solve(vm["output"].as<string>(), all_agents_solved);
		if (!pbs_success) {
			break;
		}
		auto last_time = Time::now();
		std::chrono::duration<float> duration_time = last_time - global_start_time;
		if (duration_time.count() >= vm["cutoffTime"].as<double>()) {
			break;
		}
	}
	auto global_end_time = Time::now();
	std::chrono::duration<float> global_run_time = global_end_time - global_start_time;
	string traj_name = vm["agents"].as<string>();
	pbs.updateCost();
	pbs.saveResults(all_agents_solved, vm["statistic"].as<string>(), vm["map"].as<string>());
}