#include "../include/PBS.h"
#include "../include/Instance.h"
#include <boost/program_options.hpp>
#include <boost/tokenizer.hpp>
#include <cstdlib>

#include <chrono>
using namespace std::chrono;

int main(int argc, char** argv)
{
    namespace po = boost::program_options;
	// Declare the supported options.
	po::options_description desc("Allowed options");
	desc.add_options()
		("help", "produce help message")
		// params for the input instance and experiment settings
		("map,m", po::value<string>()->default_value("./data/intro_graph_3.json"), "input file for map")
		("agents,a", po::value<string>()->default_value("./data/500vphpl/vehicleArrival_500vphpl_random1.json"), 
			"input file for agents")
		("output,o", po::value<string>()->default_value("./output.txt"), "output file for schedule")
		("statistic,c", po::value<string>()->default_value("./output.csv"), "output file for statistic result")
		("agentNum,k", po::value<int>()->default_value(3), "number of agents")
		;

	po::variables_map vm;
	po::store(po::parse_command_line(argc, argv, desc), vm);

	if (vm.count("help"))
	{
		cout << desc << endl;
		return 1;
	}

	po::notify(vm);

    // Load the configuration and data
    int step = vm["agentNum"].as<int>;
    std::string arr_file = vm["agents"].as<std::string>;
    std::string result_file = vm["statistic"].as<std::string>;


    std::string CONFIG_PATH = "./data/";
    std::string graph_file = CONFIG_PATH + "intro_graph_3.json";
    std::string pdf_ile = CONFIG_PATH + "pairDistance_3.json";
    // std::string arr_file = CONFIG_PATH + "vehicleArrival500vphpl.json";

    std::shared_ptr<Instance> instance_ptr = std::make_shared<Instance>(step, graph_file, pdf_ile, arr_file);

    // Start planning
    auto startTime = high_resolution_clock::now();
    PBS pbs(instance_ptr);
    pbs.run("output.txt");
    auto stopTime = high_resolution_clock::now();
    auto duration = duration_cast<microseconds>(stopTime - startTime); 
    std::cout << duration.count()/1000000.0 << std::endl;
    pbs.saveResults(result_file, "PBS-Bezier");
    return 0;
}