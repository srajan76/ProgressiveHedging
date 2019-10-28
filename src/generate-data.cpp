#include "instance.hpp"
#include "optionParser.hpp"

using namespace std;



int main(int argc, char* argv[]) {

    // create a OptionParser with options
    op::OptionParser opt;
    opt.add_option("h", "help", "shows option help" ); 
    opt.add_option("t", "num_targets", "number of targets", "10" );
    opt.add_option("s", "num_satellites_per_target", "number of satellites per target", "3" );
    opt.add_option("p", "instance_path", "instance_path", "../data/" );
    opt.add_option("n", "num_instances", "number of instances", "1" );
    opt.add_option("r", "radius", "satellite radius", "5");
    opt.add_option("u", "turn_radius", "vehicle turn radius", "5");

    // parse the options and verify that all went well
    bool correct_parsing = opt.parse_options(argc, argv);
    
    if(!correct_parsing) return EXIT_FAILURE;
    if(op::str2bool(opt["h"])) { 
        opt.show_help();
        return 0;
    }
    
    int numInstances = op::str2int(opt["n"]);
    int numTargets = op::str2int(opt["t"]);
    int numSatellitesPerTarget = op::str2int(opt["s"]);
    std::string path = opt["p"];
    float radius = op::str2float(opt["r"]);
    double turnRadius = op::str2double(opt["u"]);

    std::vector<Instance> instances(numInstances, Instance());
    int count = 0;

    std::random_device rd;     
    std::mt19937 rng(rd());    
    std::uniform_int_distribution<int> uniform(2000, 3000);


    for(auto & instance : instances) {
        instance.setPath(path);
        instance.setNumTargets(numTargets);
        instance.setNumSatellitesPerTarget(numSatellitesPerTarget);
        instance.setNumSatellites();
        instance.setRadius(radius);
        instance.setTurnRadius(turnRadius);

        std::string name = std::to_string(numTargets) + "-" 
            + to_string(numSatellitesPerTarget) + "-" 
            + to_string(int(radius)) + "-"
            + to_string(count) + ".txt";
        count += 1;
        instance.setName(name);
        instance.setSeed(uniform(rng));

        instance.createData();
        instance.writeData();

    }
    


    return 0;
}
