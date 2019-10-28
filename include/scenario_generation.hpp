#ifndef SCENARIO_GENERATION_HPP
#define SCENARIO_GENERATION_HPP

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
class Instance;
class HamiltonianPath;
class scenario_generation {
    protected:
        std::string _name = "scen.txt";
        std::string _path = "./data/";
        int _seed = 2018;
        int _scenarioCount =1000;
        std::unordered_map<int, double> _probAction;
        std::unordered_map<int, std::vector<int>> _scenarios;
        std::vector<int> _targetList;
                

    public:
     scenario_generation(): _name(), _path(), _seed(2018), 
    _scenarioCount(1000), _probAction(), _scenarios(), _targetList() {};
    
    scenario_generation(const std:: vector<int> &i): _name(), _path(), _seed(2018), 
    _scenarioCount(1000), _probAction(), _scenarios(), _targetList(i){for (int j = 0; j <_targetList.size(); ++j)
            _probAction[j] = 0.5;};

        void setName(std::string name) { _name = name; }; 
        void setPath(std::string path) { _path = path; };
        void setSeed(int seed) { _seed = seed; };
        void setscenarioCount(int scenarioCount) { _scenarioCount = scenarioCount; };
        void setprobAction(const std::unordered_map<int, double> & probAction){_probAction = probAction;}; 
        void setScenarios(const std::unordered_map<int, std::vector<int>> & scenarios){_scenarios = scenarios;}; 
        void setTargetList(const std::vector<int> & targetList){ _targetList = targetList;};

        std::string getName() const { return _name; }; 
        std::string getPath() const { return _path; }; 
        int getSeed() const { return _seed; };
        int getscenarioCount() const { return _scenarioCount;};
        const std::unordered_map<int, double> & getprobAction() const { return _probAction; };
        const std::unordered_map<int, std::vector<int>> & getScenarios() const { return _scenarios; }; 
        const std::vector<int> & getTargetList() const { return _targetList; };

        void generateScenarios(){
            std::mt19937 generator(getSeed());
            std::uniform_real_distribution<> prob(0.0,1.0);
            for (int n =0; n< _targetList.size(); ++n ) {
                for (int m = 0; m < _scenarioCount; ++m){
                    if (prob(generator) <= _probAction[n])
                        _scenarios[n].push_back(1);
                    else               
                        _scenarios[n].push_back(0);

            }
            }

        }; 

        void writeScenario(){




        }; 
        void readScenario(){




        }; 


};

#endif