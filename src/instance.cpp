#include "instance.hpp"

Instance::Instance() : 
    _name(), _path(), _seed(2018), 
    _grid(std::make_tuple(100.0, 100.0)),
    _source(), _destination(), 
    _numTargets(10), 
    _targetCoords(), 
    _numSatellitesPerTarget(3), 
    _numSatellites(24), _radius(5), 
    _satelliteMap(), _satelliteCoords(), 
    _turnRadius(5.0), _targetToSatelliteMap()
    {};

void Instance::createData() {
    std::mt19937 generator(getSeed());
    setGrid(std::make_tuple(100.0, 100.0));

    // creating target coordinate generator engine 
    double minCoord = 0.0 + getRadius();
    double maxXCoord = getXGridSize() - getRadius();
    double maxYCoord = getYGridSize() - getRadius();
    std::uniform_real_distribution<> xCoordinateGenerator(minCoord, maxXCoord);
    std::uniform_real_distribution<> yCoordinateGenerator(minCoord, maxYCoord);
    std::uniform_real_distribution<> thetaGenerator(0, 2*M_PI);

    // creating satellite coordinate generator engine
    std::uniform_real_distribution<> angleGenerator(0, 2*M_PI);
    std::uniform_real_distribution<> radiusGenerator(0, getRadius());

    std::unordered_map<int, std::tuple<double, double, double>> targetCoords;
    std::unordered_map<int, int> satelliteMap; 
    std::unordered_map<int, std::tuple<double, double, double>> satelliteCoords; 

    for (auto i=0; i<getNumTargets()-2; ++i) {
        std::tuple<double, double, double> coord = std::make_tuple(
            xCoordinateGenerator(generator), 
            yCoordinateGenerator(generator), 
            thetaGenerator(generator)
        );
        targetCoords.insert({i, coord});
    }

    targetCoords.insert({getNumTargets()-1, std::make_tuple(5.0, 5.0, 0.0)});
    targetCoords.insert({getNumTargets()-2, std::make_tuple(95.0, 95.0, 0.0)});
    setSource(getNumTargets()-1);
    setDestination(getNumTargets()-2);

    int satelliteCount = 0;

    for (auto const & targetCoord : targetCoords) {
        int targetIndex = targetCoord.first;
        if (targetIndex == getSource() || targetIndex == getDestination()) continue;
        double targetXCoord = std::get<0>(targetCoord.second);
        double targetYCoord = std::get<1>(targetCoord.second);
        for (auto i=0; i<getNumSatellitesPerTarget(); ++i) {
            double angle = angleGenerator(generator);
            double radius = radiusGenerator(generator);
            std::tuple<double, double, double> coord = std::make_tuple(
                targetXCoord + radius * std::cos(angle), 
                targetYCoord + radius * std::sin(angle), 
                thetaGenerator(generator));
            satelliteCoords.insert({satelliteCount, coord});
            satelliteMap.insert({satelliteCount, targetIndex});
            satelliteCount += 1;
        }
    }

    setTargetCoords(targetCoords);
    setSatelliteCoords(satelliteCoords);
    setSatelliteMap(satelliteMap);

    return;
}

void Instance::writeData() {
    std::ofstream outfile;
    std::string file = getPath() + getName();
    outfile.open(file);

    auto targetCoords = getTargetCoords(); 
    auto satelliteCoords = getSatelliteCoords(); 
    auto satelliteMap = getSatelliteMap(); 
    
    outfile << getSeed() << std::endl << 
        getNumTargets() << std::endl << 
        getNumSatellitesPerTarget() << std::endl << 
        getRadius() << std::endl << getTurnRadius()  << 
        std::endl << std::endl << 
        getSource() << std::endl << getDestination() << 
        std::endl << std::endl;

    for (auto i=0; i<getNumTargets(); ++i) { 
        int targetIndex = i;
        std::tuple<double, double, double> coord = targetCoords.at(i);
        outfile << targetIndex << " " << 
            std::setprecision(4) << std::fixed << std::get<0>(coord) << " " << 
            std::setprecision(4) << std::fixed << std::get<1>(coord) << " " << 
            std::setprecision(4) << std::fixed << std::get<2>(coord) << std::endl;
    }
    outfile << std::endl;

    for (auto i=0; i<getNumSatellites(); ++i) { 
        int targetIndex = satelliteMap.at(i);
        std::tuple<double, double, double> coord = satelliteCoords.at(i);
        outfile << targetIndex << " " << 
            std::setprecision(4) << std::fixed << std::get<0>(coord) << " " << 
            std::setprecision(4) << std::fixed << std::get<1>(coord) << " " << 
            std::setprecision(4) << std::fixed << std::get<2>(coord) << std::endl;
    }
    
    outfile.close();
    return;
}




void Instance::readData() {
    std::ifstream infile;
    std::string file = getPath() + getName();
    infile.open(file);
    if (!infile) {
        std::cerr << "file does not exist, quitting program" << std::endl;
        exit(1);
    }

    int numTargets, numSatellitesPerTarget, numSatellites;
    int seed, source, destination; 
    float radius;
    double turnRadius;
    std::unordered_map<int, std::tuple<double, double, double>> targetCoords;
    std::unordered_map<int, int> satelliteMap; 
    std::unordered_map<int, std::tuple<double, double, double>> satelliteCoords; 
    std::unordered_map<int, std::vector<int>> targetToSatelliteMap;

    infile >> seed >> numTargets >> 
        numSatellitesPerTarget >> radius >> turnRadius >>
        source >> destination; 
    numSatellites = (numTargets - 2) * numSatellitesPerTarget;

    for (auto i=0; i<numTargets; ++i) {
        int index; 
        double x, y, theta;
        infile >> index >> x >> y >> theta;
        targetCoords.insert({index, std::make_tuple(x, y, theta)});
        targetToSatelliteMap[index] = {};
    }

    for (auto i=0; i<numSatellites; ++i) {
        int targetIndex; 
        double x, y, theta;
        infile >> targetIndex >> x >> y >> theta;
        satelliteCoords.insert({i, std::make_tuple(x, y, theta)});
        satelliteMap.insert({i, targetIndex});
        targetToSatelliteMap[targetIndex].push_back(i);
    }

    infile.close();
    setNumTargets(numTargets);
    setSource(source);
    setDestination(destination);
    setNumSatellitesPerTarget(numSatellitesPerTarget);
    setNumSatellites();
    setSeed(seed);
    setRadius(radius);
    setTurnRadius(turnRadius);

    setTargetCoords(targetCoords);
    setSatelliteCoords(satelliteCoords);
    setTargetToSatelliteMap(targetToSatelliteMap);
    setSatelliteMap(satelliteMap);
    
    return;
}
