#ifndef INSTANCE_HPP
#define INSTANCE_HPP

#pragma once

#include <iostream>
#include <fstream> 
#include <string>
#include <unordered_map>
#include <tuple>
#include <set>
#include <sstream>
#include <vector>
#include <random>
#include <cmath>
#include <iomanip>
#include <limits>

#define _USE_MATH_DEFINES


class Instance {
    protected:
        std::string _name = "data.txt";
        std::string _path = "./data/";
        int _seed = 2018;
        std::tuple<double, double> _grid;
        int _source;
        int _destination;
        int _numTargets = 12;
        std::unordered_map<int, std::tuple<double, double, double>> _targetCoords;
        int _numSatellitesPerTarget = 3;
        int _numSatellites = 30;
        float _radius = 5;
        std::unordered_map<int, int> _satelliteMap; // satellite id -> target id
        std::unordered_map<int, std::vector<int>> _targetToSatelliteMap;
        std::unordered_map<int, std::tuple<double, double, double>> _satelliteCoords; 
        double _turnRadius = 5;
        

    public:
        Instance();

        void setName(std::string name) { _name = name; }; 
        void setPath(std::string path) { _path = path; };
        void setSeed(int seed) { _seed = seed; };
        void setGrid(std::tuple<double, double> grid) { _grid = grid; };
        void setSource(int source) { _source = source; };
        void setDestination(int destination) { _destination = destination; }; 
        void setNumTargets(int numTargets) { _numTargets = numTargets; };
        void setNumSatellites() { _numSatellites = (_numTargets - 2) * _numSatellitesPerTarget; };
        void setNumSatellitesPerTarget(int numSatellitesPerTargets) { _numSatellitesPerTarget = numSatellitesPerTargets; };
        void setRadius(float radius) { _radius = radius; };
        void setTargetCoords(const std::unordered_map<int, std::tuple<double, double, double>> & targetCoords) { 
            _targetCoords = targetCoords; 
        }; 
        void setSatelliteCoords(const std::unordered_map<int, std::tuple<double, double, double>> & satelliteCoords) { 
            _satelliteCoords = satelliteCoords; 
        }; 
        void setSatelliteMap(const std::unordered_map<int, int> & satelliteMap) { 
            _satelliteMap = satelliteMap; 
        };
        void setTargetToSatelliteMap(const std::unordered_map<int, std::vector<int>> targetToSatelliteMap) {
            _targetToSatelliteMap = targetToSatelliteMap;
        };
        void setTurnRadius(double turnRadius) { _turnRadius = turnRadius; };

        std::string getName() const { return _name; }; 
        std::string getPath() const { return _path; }; 
        int getSeed() const { return _seed; };
        double getXGridSize() const { return std::get<0>(_grid); };
        double getYGridSize() const { return std::get<1>(_grid); };
        int getSource() const { return _source; };
        int getDestination() const { return _destination; };
        int getNumTargets() const { return _numTargets; }; 
        int getNumSatellitesPerTarget() const { return _numSatellitesPerTarget; };
        int getNumSatellites() const { return _numSatellites; }
        float getRadius() const { return _radius; };
        const std::unordered_map<int, std::tuple<double, double, double>> & getTargetCoords() const { return _targetCoords; }; 
        const std::unordered_map<int, std::tuple<double, double, double>> & getSatelliteCoords() const { return _satelliteCoords; };
        const std::vector<int> & getSatellitesAtTarget(int i) const { return _targetToSatelliteMap.at(i); }
        const std::unordered_map<int, int> & getSatelliteMap() const { return _satelliteMap; };
        double getTurnRadius() const { return _turnRadius; };

        void createData(); 
        void writeData(); 
        void readData(); 
};





#endif
