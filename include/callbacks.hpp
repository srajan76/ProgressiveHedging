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

typedef const std::vector<std::tuple<int, int>> & detEdgeType;
std::vector<IloRange> generateLazyConstraintsDet(
    Model & model,
    detEdgeType & detEdges,
    const std::vector<int> & satTargetsIn,
    const std::vector<int> & satTargetsOut,
    const int targets,
    const int numsatellites,
    std::unordered_map<std::string, IloNumArray>& variableValues){

    std::vector<IloRange> constraints;
    lemon::ListDigraph supportGraph;
    for (int i=0; i<=numsatellites+1; ++i) 
        supportGraph.addNode();
        
    IloNumArray xVals = variableValues.at("f");
    for (auto i=0; i<xVals.getSize(); ++i)
        if (xVals[i] > 1E-5) {
            int from = std::get<0>(detEdges[i]);
            int to = std::get<1>(detEdges[i]);
            if (from < targets)
                supportGraph.addArc(supportGraph.nodeFromId(0), supportGraph.nodeFromId((to- targets)%numsatellites +1));
            else if(from >= targets && to>= targets)
                supportGraph.addArc(supportGraph.nodeFromId((from- targets)%numsatellites +1), supportGraph.nodeFromId((to- targets)%numsatellites +1));
            else
                supportGraph.addArc(supportGraph.nodeFromId((from- targets)%numsatellites +1), supportGraph.nodeFromId(numsatellites+1));           
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
                for( int m =0; m <detEdges.size(); ++m){
                    auto out= std::get<0>(detEdges[m]) ;
                    auto in = std::get<1>(detEdges[m]) ;
                    out = (out-targets)%numsatellites +1;
                    in = (in-targets)%numsatellites +1;
                    if ( out <=0)
                        out =0;
                    if ( in <=0)
                        in= numsatellites+1;
                    if ( from == out && to == in){
                        constraintindices.push_back(m);
                        break;
                    }
                }
            }
        }
         IloExpr expr(model.getEnv());
         for (auto const &m : constraintindices)
            expr+= model.getVariables().at("f")[m];
         IloRange constr(model.getEnv(), expr, vertexIds.size()-1);
         constraints.push_back(constr);
         expr.end();
    }
    
    return constraints;
};   
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
    detEdgeType&, detEdges,
    std::vector<int>&, satTargetsIn,
    std::vector<int>&, satTargetsOut,
    int, targets, 
    int, numsatellites) {
    
    IloEnv env = getEnv();
    std::unordered_map<std::string, IloNumArray> variableValues;
    variableValues.insert(std::make_pair("f", IloNumArray(env, model.getVariables().at("f").getSize())));
    getValues(variableValues.at("f"), model.getVariables().at("f"));
    std::vector<IloRange> constraints = generateLazyConstraintsDet(
        model, 
        detEdges,
        satTargetsIn,
        satTargetsOut,
        targets,
        numsatellites,
        variableValues);
    
    for (IloRange constr : constraints)
        add(constr);
        
        
    
    variableValues.at("f").end();
    constraints.clear();
};

#endif