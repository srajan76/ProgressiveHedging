#ifndef TWOSTAGE_HPP
#define TWOSTAGE_HPP

#pragma once

#include <ilcplex/ilocplex.h>
#include <vector>
#include <tuple>
#include <unordered_map>
#include <limits>
#include <string>
#include <algorithm>
#include "util.hpp"


class Instance;
class Model;
class Edge;
class Scenarios;


class TwoStage {
    protected: 
        const Instance & _instance;
        Model _model;
        const Scenarios _scenarios;
        const std::unordered_map<int, std::tuple<double, double, double>> & _coords; 
        int _source;
        int _destination;
        std::vector<Edge> _firstStageEdges;
        std::vector<Edge> _recourseEdges;
        std::vector<std::vector<int>> _recoursePaths;
        std::unordered_map<std::tuple<int, int>, int> _edgeMap;
        std::vector<int> _path;
        double _firstStageCost;
        double _secondStageCost;
        double _pathCost;
        std::vector<std::tuple<int,int>> _detEdges;
        std::unordered_map<int, std::tuple<int,int>> _satEdgeMap;
        std::vector<int> _firstStageX;
    
    private: 
        void setSource()  { _source = _instance.getSource(); };
        void setDestination() { _destination = _instance.getDestination(); };
        void populateConstraintsPH(std::vector<int> w);
        void computePath(std::vector<std::tuple<int, int>> &);
        void populateConstraintsDet(std::tuple<int,int>&);
        double detCostCompute(std::vector<Edge>&v, std::tuple<int, int> &t);
        void solvePHscenarioDet(std::tuple<int,int> t);

    public: 
        TwoStage(const Instance & instance, const Scenarios & scenarios);

        const Instance & getInstance() const { return _instance; };
        Model & getModel() { return _model; };
        const Scenarios & getScenarios() const { return _scenarios; }
        const std::unordered_map<int, std::tuple<double, double, double>> & getCoords() const { return _coords; };
        const std::tuple<double, double, double> & getCoord(int i) const { return _coords.at(i); };
        const int getNumVertices() const { return _coords.size(); };
        int getSource() const { return _source; };
        int getDestination() const { return _destination; };
        const std::vector<Edge> & getFirstStageEdges() const { return _firstStageEdges; };
        const std::vector<Edge> & getSecondStageEdges() const { return _recourseEdges; };
        const std::unordered_map<std::tuple<int, int>, int> & getEdgeMap() const { 
            return _edgeMap; 
        };
        const std::vector<int> & getPath() const { return _path; };
        double getFirstStageCost() const { return _firstStageCost; };
        double getSecondStageCost() const { return _secondStageCost; };
        double getPathCost() const { return _pathCost; };
        void setModel(Model & model) { _model = model; };
        std::vector<int> getfirstStageScenarioSolution() {return _firstStageX;};


        void initialize();
        void calcDetCostPath(std::vector<int> &,std::vector<int> &);
        void populateEdgesPH();
        void solvePHscenario(std::vector<int> omega);
        void solvePHscenario(std::vector<int> omega, std::vector<double> w, std::vector<double> Xbar, double Rho);
        
    
         
};

#endif
