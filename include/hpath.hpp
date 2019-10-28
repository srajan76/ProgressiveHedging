#ifndef HPATH_HPP
#define HPATH_HPP

#pragma once

#include <vector>
#include <tuple>
#include <lemon/list_graph.h>
#include <lemon/hao_orlin.h>
#include <lemon/nagamochi_ibaraki.h>
#include <lemon/connectivity.h>

class Instance;
class Model;
class Edge;

/*  Specializing the std::hash and std::equal_to functions 
    to hash a std::tuple<int, int> and compare two std::tuple<int, int>;
    implementation for hash function flicked from boost's hash function and
    hence, do not completely understand how the hash is being performed.
    Implementation of std::equal_to<std::tuple<int, int>> is self-explanatory. 
    This is used to construct the _edgeMap in the HamilotonianPath class. */
namespace std {
    template <> 
    struct hash<tuple<int, int>> {
        size_t operator() (const tuple<int, int>& p) const {
            size_t seed = 0;
            std::hash<int> h;
            seed ^= h(std::get<0>(p)) + 0x9e3779b9 + (seed << 6) + (seed >> 2); 
            seed ^= h(std::get<1>(p)) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
            return seed;
        }
    };

    template <>
    struct equal_to<tuple<int, int>> {
        bool operator() (const tuple<int, int>& x, const tuple<int, int>& y) const {
            return (get<0>(x) == get<0>(y) && get<1>(x) == get<1>(y));
        }
    };
}

class HamiltonianPath {
    protected: 
        const Instance & _instance;
        Model _model;
        std::unordered_map<int, std::tuple<double, double>> _vertexCoords; 
        std::unordered_map<int, bool> _isTarget;
        std::unordered_map<int, bool> _isSatellite;
        std::unordered_map<int, int> _vertexToTargetMap;
        std::unordered_map<int, int> _vertexToSatelliteMap;
        int _source;
        int _destination;
        std::vector<Edge> _edges;
        std::unordered_map<std::tuple<int, int>, int> _edgeMap;
        std::vector<int> _path;
        double _pathCost;
    
    public: 
        HamiltonianPath(const Instance & instance);

        const Instance & getInstance() const { return _instance; };
        Model & getModel() { return _model; };
        const std::unordered_map<int, std::tuple<double, double>> & getVertexCoords() const { 
            return _vertexCoords; 
        };
        int getNumVertices() const { return _vertexCoords.size(); };
        const std::unordered_map<int, bool> & getIsTarget() const { 
            return _isTarget; 
        };
        const std::unordered_map<int, bool> & getIsSatellite() const { 
            return _isSatellite; 
        };
        const std::unordered_map<int, int> & getVertexToTargetMap() const { 
            return _vertexToTargetMap; 
        };
        const std::unordered_map<int, int> & getVertexToSatelliteMap() const { 
            return _vertexToSatelliteMap; 
        };
        int getSource() const { return _source; };
        int getDestination() const { return _destination; };
        const std::vector<Edge> & getEdges() const { return _edges; };
        const std::unordered_map<std::tuple<int, int>, int> & getEdgeMap() const { 
            return _edgeMap; 
        };
        const std::vector<int> & getPath() const { return _path; };
        double getPathCost() const { return _pathCost; };

        void setVertexCoords(const std::unordered_map<int, std::tuple<double, double>> & vertexCoords) { 
            _vertexCoords = vertexCoords; 
        };
        void setIsTarget(const std::unordered_map<int, bool> & isTarget) { 
            _isTarget = isTarget; 
        };
        void setIsSatellite(const std::unordered_map<int, bool> & isSatellite) { 
            _isSatellite = isSatellite; 
        };
        void setVertexToTargetMap(const std::unordered_map<int, int> & vertexToTargetMap) { 
            _vertexToTargetMap = vertexToTargetMap;
        };
        void setVertexToSatelliteMap(const std::unordered_map<int, int> & vertexToSatellitetMap) {
            _vertexToSatelliteMap = vertexToSatellitetMap; 
        };
        void setSource(int source) { _source = source; };
        void setDestination(int destination) { _destination = destination; };
        void setEdges(const std::vector<Edge> & edges) { _edges = edges; };
        void setEdgeMap(const std::unordered_map<std::tuple<int, int>, int> & edgeMap) { 
            _edgeMap = edgeMap; 
        };
        void setPath(const std::vector<int> & path) { _path = path; };
        void setPathCost(double pathCost) { _pathCost = pathCost; };

        void populatePathData(std::vector<int> & targets, 
            std::vector<int> satellites, 
            int sourceTarget, 
            int destinationTarget);
        void createEdges();
        
        Model addVariablesandConstraints();
        void solve();
        void populateSolution();
        
};

        


#endif
