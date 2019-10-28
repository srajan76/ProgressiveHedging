#ifndef CALLBACKS_HPP
#define CALLBACKS_HPP

#pragma once

#include <ilcplex/ilocplex.h>
#include <vector>
#include <tuple>
#include <unordered_map>
#include <string>
#include <lemon/list_graph.h>
#include <lemon/connectivity.h>
#include <set>
#include "util.hpp"

typedef const std::unordered_map<std::tuple<int, int>, int> & edgeMapType;
std::vector<IloRange> generateLazyConstraintsPH(
    Model & model,  
    const std::vector<Edge> & firststageedges, 
    const std::vector<Edge> & recourseedges, 
    const int numVertices,
    const int numsatellites,
    const int source, 
    const int destination,
    std::unordered_map<std::string, IloNumArray> & variableValues) {

    std::vector<IloRange> constraints;
    lemon::ListDigraph supportGraph;

    for (int i=0; i<numVertices + numsatellites; ++i) 
        supportGraph.addNode();

    IloNumArray xVals = variableValues.at("x");
    IloNumArray yVals = variableValues.at("y");

    for (auto i=0; i<xVals.getSize(); ++i)
        if (xVals[i] > 1E-5) {
            int from = firststageedges[i].from();
            int to = firststageedges[i].to();
            supportGraph.addArc(supportGraph.nodeFromId(from), supportGraph.nodeFromId(to));
        }
    for (auto i=0; i<yVals.getSize(); ++i)
        if (yVals[i] > 1E-5) {
            int from = recourseedges[i].from();
            int to = recourseedges[i].to();
            supportGraph.addArc(supportGraph.nodeFromId(from), supportGraph.nodeFromId(to));
        }

    lemon::ListDigraph::NodeMap<int> componentMap(supportGraph);
    int numComponents = stronglyConnectedComponents(supportGraph, componentMap);
    
    std::vector<std::set<int>> components(numComponents);
	for (lemon::ListDigraph::NodeIt n(supportGraph); n!=lemon::INVALID; ++n)
		components[componentMap[n]].insert(supportGraph.id(n));

    for (auto const & component : components) {
        if (component.size() <= 1) continue;
        std::set<int> vertexIds;
        for (auto const & vertexId : component) 
            vertexIds.insert(vertexId);
        std::vector<int> constraintindices;
        for (auto const & from : vertexIds){
            for(auto const & to : vertexIds){
                if (from ==to) continue;
                for( int m =0; m <recourseedges.size(); ++m){
                    if ( from == recourseedges[m].from() && to == recourseedges[m].to()){
                        constraintindices.push_back(m);
                        break;
                    }
                }
            }
        }
         IloExpr expr(model.getEnv());
         for (auto const &m : constraintindices)
            expr+= model.getVariables().at("y")[m];
         IloRange constr(model.getEnv(), expr, vertexIds.size()-1);
         constraints.push_back(constr);
         expr.end();
    }

    return constraints;
};
std::vector<IloRange> generateLazyConstraints(
    Model & model,  
    const std::vector<Edge> & edges, 
    const std::unordered_map<std::tuple<int, int>, int> & edgeMap,
    const int numVertices,
    const int source, 
    const int destination,
    std::unordered_map<std::string, IloNumArray> & variableValues) {

    std::vector<IloRange> constraints;
    lemon::ListDigraph supportGraph;

    for (int i=0; i<numVertices; ++i) 
        supportGraph.addNode();

    IloNumArray xVals = variableValues.at("x");

    for (auto i=0; i<xVals.getSize(); ++i)
        if (xVals[i] > 1E-5) {
            int from = edges[i].from();
            int to = edges[i].to();
            supportGraph.addArc(supportGraph.nodeFromId(from), supportGraph.nodeFromId(to));
        }

    lemon::ListDigraph::NodeMap<int> componentMap(supportGraph);
    int numComponents = stronglyConnectedComponents(supportGraph, componentMap);

    std::vector<std::set<int>> components(numComponents);
	for (lemon::ListDigraph::NodeIt n(supportGraph); n!=lemon::INVALID; ++n)
		components[componentMap[n]].insert(supportGraph.id(n));

    for (auto const & component : components) {
        if (component.size() <= 1) continue;
        std::set<int> vertexIds;
        for (auto const & vertexId : component) 
            vertexIds.insert(vertexId);
        
        const bool sourceIn = vertexIds.find(source) != vertexIds.end();
        const bool destinationIn = vertexIds.find(destination) != vertexIds.end();
        if (sourceIn || destinationIn) continue;
        
        std::vector<int> edgeIds;
        for (auto const & from : vertexIds) {
            for (auto const & to : vertexIds) {
                if (from == to) continue;
                auto itr = edgeMap.find(std::make_tuple(from, to));
                if (itr != edgeMap.end()) 
                    edgeIds.push_back(itr->second);
            }
        }

        IloExpr expr(model.getEnv());
        for (auto const & edgeId : edgeIds)
            expr += model.getVariables().at("x")[edgeId];
        IloRange constr(model.getEnv(), expr, vertexIds.size()-1);
        constraints.push_back(constr);
        expr.end();
    }

    return constraints;
};
ILOLAZYCONSTRAINTCALLBACK7(addLazyCallback1, 
    Model&, model, 
    std::vector<Edge>&, firststageedges, 
    std::vector<Edge>&, recourseedges,
    int, numVertices,
    int, numsatellites,
    int, source, 
    int, destination) {
    
    IloEnv env = getEnv();
    std::unordered_map<std::string, IloNumArray> variableValues;
    variableValues.insert(std::make_pair("x", IloNumArray(env, model.getVariables().at("x").getSize())));
    variableValues.insert(std::make_pair("y", IloNumArray(env, model.getVariables().at("y").getSize())));
    getValues(variableValues.at("x"), model.getVariables().at("x"));
    getValues(variableValues.at("y"), model.getVariables().at("y"));
    std::vector<IloRange> constraints = generateLazyConstraintsPH(
        model, 
        firststageedges, 
        recourseedges,
        numVertices, 
        numsatellites,
        source, 
        destination, 
        variableValues);

    for (IloRange constr : constraints)
        add(constr);

    variableValues.at("x").end();
    variableValues.at("y").end();
    constraints.clear();
};
ILOLAZYCONSTRAINTCALLBACK6(addLazyCallback, 
    Model&, model, 
    std::vector<Edge>&, edges, 
    edgeMapType, edgeMap,
    int, numVertices,
    int, source, 
    int, destination) {
    
    IloEnv env = getEnv();
    std::unordered_map<std::string, IloNumArray> variableValues;
    variableValues.insert(std::make_pair("x", IloNumArray(env, model.getVariables().at("x").getSize())));
    getValues(variableValues.at("x"), model.getVariables().at("x"));

    std::vector<IloRange> constraints = generateLazyConstraints(
        model, 
        edges, 
        edgeMap,
        numVertices, 
        source, 
        destination, 
        variableValues);

    for (IloRange constr : constraints)
        add(constr);

    variableValues.at("x").end();
    constraints.clear();
};


#endif