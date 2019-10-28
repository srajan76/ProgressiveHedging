#include "instance.hpp"
#include "model.hpp"
#include "edge.hpp"
#include "scenarios.hpp"
#include "twostage.hpp"
#include "dubins.hpp"
#include "callbacks.hpp"

#include <cassert>

TwoStage::TwoStage(const Instance & instance,
    const Scenarios & scenarios) :
    _instance(instance), 
    _model(), 
    _scenarios(scenarios),
    _coords(instance.getTargetCoords()), 
    _source(), 
    _destination(), 
    _firstStageEdges(), 
    _recourseEdges(),
    _hasRecourseEdge(),
    _recoursePaths(), 
    _edgeMap(), 
    _satEdgeMap(),
    _path(), 
    _firstStageCost(), 
    _secondStageCost(), 
    _pathCost(), 
    _ub(), 
    _firstStageX(),
    _ubRange() {};

void TwoStage::initialize() { 
    setSource(); 
    setDestination();
    return;
};



void TwoStage::populateEdgesPH(){
    auto coords = getCoords();
    auto satelliteCoords = getInstance().getSatelliteCoords(); // returns a map <int, tuple> with satellite coords
    auto satelliteMap = getInstance().getSatelliteMap(); //returns map<int, int> where key is index of satellite and its value is the target index
    std::unordered_map<int,std::tuple< int,int>> satEdgeMap;
    std::vector<Edge> edges;
    std::vector<Edge> recourseEdges;


    int i = getSource();
    auto ith = coords.at(i);
    std::vector<double> qInitial = {std::get<0>(ith), std::get<1>(ith), std::get<2>(ith)};
    for (int j=0; j<getNumVertices(); ++j) {
            if (j == i ||j == getDestination())  continue;
            auto jth = coords.at(j);
            std::vector<double> qFinal = {std::get<0>(jth), std::get<1>(jth), std::get<2>(jth)};
            DubinsPath path = getDubinsShortestPath(qInitial, qFinal, getInstance().getTurnRadius());
            double directCost = path.getLength();
            edges.push_back(Edge(i,j,directCost));
    }
    
    int idx = getNumVertices();
    for (int i=0; i<getNumVertices(); ++i) {
        if ( i ==getSource() || i == getDestination()) continue;
        auto ith = coords.at(i);
        std::vector<double> qInitial = {std::get<0>(ith), std::get<1>(ith), std::get<2>(ith)};
        auto satAttargetI = getInstance().getSatellitesAtTarget(i);
        int idx2 =0;
        for (auto const & k : satAttargetI){
            auto jth = satelliteCoords.at(k);
            std::vector<double> qFinal = {std::get<0>(jth), std::get<1>(jth), std::get<2>(jth)};
            DubinsPath path = getDubinsShortestPath(qInitial, qFinal, getInstance().getTurnRadius());
            double recourseCost = path.getLength();
            recourseEdges.push_back(Edge(i,idx+idx2,recourseCost));
            satEdgeMap[k] = std::make_tuple(i,idx+idx2);
            idx2+=1;
        }
        idx+=idx2;
        for( int j =0; j< getNumVertices(); ++j){
            if (j == getSource()|| j==i) continue;
            auto jth = coords.at(j);
            std::vector<double> qFinal = {std::get<0>(jth), std::get<1>(jth), std::get<2>(jth)};
            DubinsPath path = getDubinsShortestPath(qInitial, qFinal, getInstance().getTurnRadius());
            double recourseCost = path.getLength();
            recourseEdges.push_back(Edge(i,j,recourseCost));
        }
    }    //set Edges from targets to their respective satellites or other targets
    

    for (int i =0; i < satelliteCoords.size(); ++i){
        for (int j =0; j<getNumVertices(); ++j){
            if (j==getSource()|| j== satelliteMap[i]) continue;
            auto ith = satelliteCoords.at(i);
            auto jth = coords.at(j);
            std::vector<double> qInitial = {std::get<0>(ith), std::get<1>(ith), std::get<2>(ith)};
            std::vector<double> qFinal = {std::get<0>(jth), std::get<1>(jth), std::get<2>(jth)};
            DubinsPath path = getDubinsShortestPath(qInitial, qFinal, getInstance().getTurnRadius());
            double recourseCost = path.getLength();
            recourseEdges.push_back(Edge(std::get<1>(satEdgeMap.at(i)),j,recourseCost));
            
        }
           
    }     //set edges from all satellites to remaining targets

    for (int i =0; i <getNumVertices(); ++i){
         auto satAttargetI = getInstance().getSatellitesAtTarget(i);
         
         for (auto const & a : satAttargetI){
             for (auto const & b : satAttargetI){
                if(a==b) continue;
                auto ith = satelliteCoords.at(a);
                auto jth = satelliteCoords.at(b);
                std::vector<double> qInitial = {std::get<0>(ith), std::get<1>(ith), std::get<2>(ith)};
                std::vector<double> qFinal = {std::get<0>(jth), std::get<1>(jth), std::get<2>(jth)};
                DubinsPath path = getDubinsShortestPath(qInitial, qFinal, getInstance().getTurnRadius());
                double recourseCost = path.getLength();
                recourseEdges.push_back(Edge(std::get<1>(satEdgeMap.at(a)),std::get<1>(satEdgeMap.at(b)),recourseCost));

             }
         } //set edges from one satellite to another at each target location
    }
    _firstStageEdges = edges; //set first stage edges
    _satEdgeMap = satEdgeMap; //maps every satellite index to its target and satellite number (from edge)
    _recourseEdges= recourseEdges;//set recourse edges
    
return;    
};

void TwoStage::populateConstraintsPH(std::vector<int> w) {
    std::unordered_map<int, std::tuple<int,int>> mapSatEdgetoTarget;
    Model model;
    model.getModel().setName(getInstance().getName().c_str());
    IloNumVarArray x(model.getEnv());
    model.getVariables().insert({"x", x});
     for (int i=0; i<_firstStageEdges.size(); ++i) {
        auto edge = _firstStageEdges[i];
        std::string varname = "x_" + std::to_string(edge.from()) +  "_" + std::to_string(edge.to());
        IloNumVar var(model.getEnv(), 0, 1, ILOINT, varname.c_str());
        model.getVariables().at("x").add(var);
    }

    
        IloNumVarArray y_s(model.getEnv()); 
        model.getVariables().insert({"y", y_s});
        for (int i=0; i<_recourseEdges.size(); ++i) {
            auto edge = _recourseEdges[i];
            std::string varname = "y" + std::to_string(edge.from()) +  "_" + std::to_string(edge.to());
            IloNumVar var(model.getEnv(), 0, 1, ILOINT, varname.c_str());
            model.getVariables().at("y").add(var);
        }
    
    IloExpr Expr(model.getEnv()); //this constraint makes sure that atleast one arc goes out of origin
    for (int i=0; i<_firstStageEdges.size(); ++i) {
       Expr+= model.getVariables().at("x")[i];
    }
    std::string firststage = "firststage";
    IloRange firststage_constr(model.getEnv(), 1, Expr, 1, firststage.c_str());
    model.getConstraints().push_back(firststage_constr);
    //std::cout<<"onestage arcs out ="<<model.getConstraints().size()<<std::endl;

    IloExprArray ExprY(model.getEnv(), getNumVertices()-2);
    IloExprArray ExprY2(model.getEnv(), getNumVertices()-1);
    IloExprArray ExprY3(model.getEnv(), getNumVertices()-2);
    IloExprArray ExprY4(model.getEnv(), getInstance().getNumSatellites());
    IloExprArray ExprY5(model.getEnv(), getInstance().getNumSatellites());
    for (int i =0; i< getNumVertices()-2; ++i){
        ExprY[i]  = IloExpr(model.getEnv());
        ExprY3[i] = IloExpr(model.getEnv());
    }
    for (int i =0; i< getNumVertices()-1; ++i){
        ExprY2[i] = IloExpr(model.getEnv());
    }
    for(int i =0; i< getInstance().getNumSatellites(); ++i){
        ExprY4[i] = IloExpr(model.getEnv());   
        ExprY5[i] = IloExpr(model.getEnv());
    }

    for ( int p =0; p < _firstStageEdges.size(); ++p){
        int to = _firstStageEdges[p].to();
        ExprY2[to] = model.getVariables().at("x")[p];
    }    
    
    for( int m =0; m <_recourseEdges.size(); ++m){
        int from = _recourseEdges[m].from();
        int to = _recourseEdges[m].to();
        int satIndexF, satIndexT;
        if (from>=getNumVertices()){
            satIndexF= from - getNumVertices();
        }
        if (to>=getNumVertices()){
            satIndexT= to - getNumVertices();
        }
        if(from< getNumVertices())             
            ExprY[from]+= model.getVariables().at("y")[m]; // from a target {N}\{o,d} to another target or  satellite one outgoing arc
        
        if(to< getNumVertices())             
            ExprY2[to]+= model.getVariables().at("y")[m]; // to a target {N}\{o} from another target or  stellite one incoming arc
        
        if(from< getNumVertices() && to >= getNumVertices())
            ExprY3[from]+= model.getVariables().at("y")[m]; //outgoing arc from target to satellite = w{from}
        
        if(from>= getNumVertices())
            ExprY4[satIndexF]+= model.getVariables().at("y")[m]; //outgoing arc from satellite to targets/ satellites = w
        
        if(to>= getNumVertices())
            ExprY5[satIndexT] += model.getVariables().at("y")[m];//to every satellite one arc comes in if W{i}=1
            
    }

    for (int i =0; i< getNumVertices()-2; ++i){
        std::string secondstage_constr_arcout = "secondstage_arcout_" +i;
        IloRange secondstage_constr_arcouT(model.getEnv(), 1, ExprY[i], 1, secondstage_constr_arcout.c_str());
        model.getConstraints().push_back(secondstage_constr_arcouT);
        std::string secondstage_constr_arcouttoS = "secondstage_arcouttoS_" +i;
        IloRange secondstage_constr_arcouTtoS(model.getEnv(), w[i], ExprY3[i], w[i], secondstage_constr_arcouttoS.c_str());
        model.getConstraints().push_back(secondstage_constr_arcouTtoS);
    }
    //std::cout<<"twostage target arcs out ="<<model.getConstraints().size()<<std::endl;
    for (int i =0; i< getNumVertices()-1; ++i){
        std::string secondstage_constr_arcin = "secondstage_arcin_" +i;
        IloRange secondstage_constr_arcIn(model.getEnv(), 1, ExprY2[i], 1, secondstage_constr_arcin.c_str());
        model.getConstraints().push_back(secondstage_constr_arcIn);
    }
    //std::cout<<"twostage target arcs in ="<<model.getConstraints().size()<<std::endl;
    int satpertarget = getInstance().getNumSatellitesPerTarget();
    for(int i =0; i< getInstance().getNumSatellites(); ++i){
        std::string secondstage_constr_Sarcout = "secondstage_Sarcout_" +i;
        IloRange secondstage_constr_SarcouT(model.getEnv(), w[int(i/satpertarget)], ExprY4[i],  w[int(i/satpertarget)], secondstage_constr_Sarcout.c_str());
        model.getConstraints().push_back(secondstage_constr_SarcouT);
        std::string secondstage_constr_Sarcin = "secondstage_Sarcin_" +i;
        IloRange secondstage_constr_SarciN(model.getEnv(), w[int(i/satpertarget)], ExprY5[i], w[int(i/satpertarget)], secondstage_constr_Sarcin.c_str());
        model.getConstraints().push_back(secondstage_constr_SarciN);
    }
    //std::cout<<"twostage satellite arcs out and In ="<<model.getConstraints().size()<<std::endl;

    for (IloRange constr : model.getConstraints())  {
        model.getModel().add(constr);
    }

    setModel(model);
    return;
};

void TwoStage::solvePHscenario(std::vector<int> omega){
    populateConstraintsPH(omega);
    IloNumArray stage1cost(_model.getEnv());
    IloNumArray stage2cost(_model.getEnv());
    for (int i =0; i<_firstStageEdges.size(); ++i)
        stage1cost.add(_firstStageEdges[i].cost());
    for (int i =0; i<_recourseEdges.size(); ++i)
        stage2cost.add(_recourseEdges[i].cost());
    IloExpr obj(_model.getEnv());
    obj = IloScalProd(stage1cost, _model.getVariables().at("x")) + IloScalProd(stage2cost, _model.getVariables().at("y"));
    _model.getModel().add(IloMinimize(_model.getEnv(), obj )); //added objective
    IloCplex cplex(_model.getEnv());
    cplex.extract(_model.getModel());
    cplex.setOut(_model.getEnv().getNullStream());
    cplex.use(addLazyCallback1(_model.getEnv(), 
        _model,
        _firstStageEdges, 
        _recourseEdges,
        getNumVertices(), 
        getInstance().getNumSatellites(),
        getSource(), 
        getDestination()
    ));
    cplex.solve();
    double firstStageCost = 0.0;
    double recourseCost =0.0;
    std::vector<std::tuple<int, int>> solutionEdges;
    std::vector<int> firstStageX={};
    for (int i=0; i<_model.getVariables().at("x").getSize(); ++i) {
        if (cplex.getValue(_model.getVariables().at("x")[i]) > 0.9) {
            int from = _firstStageEdges[i].from();
            int to = _firstStageEdges[i].to();
            solutionEdges.push_back(std::make_pair(from, to));
            firstStageCost += _firstStageEdges[i].cost();
            firstStageX.push_back(1);
        }
        else
            firstStageX.push_back(0);
    }
    for (int i=0; i<_model.getVariables().at("y").getSize(); ++i) {
        if (cplex.getValue(_model.getVariables().at("y")[i]) > 0.9) {
            int from = _recourseEdges[i].from();
            int to = _recourseEdges[i].to();
            solutionEdges.push_back(std::make_pair(from, to));
            recourseCost += _recourseEdges[i].cost();
        }
    }
    computePath(solutionEdges);
    _pathCost = firstStageCost+ recourseCost;
    _firstStageX= firstStageX;
    _model.clearEnv();
return;
};
void TwoStage::solvePHscenario(std::vector<int> omega, std::vector<double> w, std::vector<double> Xbar, double Rho){
    populateConstraintsPH(omega);
    IloNumArray stage1cost(_model.getEnv());
    IloNumArray stage2cost(_model.getEnv());
    IloNumArray w_s(_model.getEnv());
    IloNumArray xbar(_model.getEnv());
    IloNumArray rho(_model.getEnv());
    IloNum Ro(Rho);
    for (int i =0; i<_firstStageEdges.size(); ++i){
        stage1cost.add(_firstStageEdges[i].cost());
        w_s.add(w[i]);
        xbar.add(Xbar[i]);
        rho.add(Rho/2);
    }
    
    for (int i =0; i<_recourseEdges.size(); ++i)
        stage2cost.add(_recourseEdges[i].cost());
    IloExpr obj(_model.getEnv());
        
   for ( int i =0; i<_firstStageEdges.size(); ++i)
        obj += rho[i]*_model.getVariables().at("x")[i]*_model.getVariables().at("x")[i]+ rho[i]*xbar[i]*xbar[i] -Ro*_model.getVariables().at("x")[i]*xbar[i];
    obj += IloScalProd(stage1cost, _model.getVariables().at("x")) + IloScalProd(stage2cost, _model.getVariables().at("y")) +IloScalProd(w_s, _model.getVariables().at("x"));   
   
    _model.getModel().add(IloMinimize(_model.getEnv(), obj)); //added objective
    IloCplex cplex(_model.getEnv());
    cplex.extract(_model.getModel());
    cplex.setOut(_model.getEnv().getNullStream());
    cplex.use(addLazyCallback1(_model.getEnv(), 
        _model,
        _firstStageEdges, 
        _recourseEdges,
        getNumVertices(), 
        getInstance().getNumSatellites(),
        getSource(), 
        getDestination()
    ));
    cplex.solve();
    double firstStageCost = 0.0;
    double recourseCost =0.0;
    std::vector<std::tuple<int, int>> solutionEdges;
    std::vector<int> firstStageX={};
    for (int i=0; i<_model.getVariables().at("x").getSize(); ++i) {
        if (cplex.getValue(_model.getVariables().at("x")[i]) > 0.9) {
            int from = _firstStageEdges[i].from();
            int to = _firstStageEdges[i].to();
            solutionEdges.push_back(std::make_pair(from, to));
            firstStageCost += _firstStageEdges[i].cost();
            firstStageX.push_back(1);
        }
       else
         firstStageX.push_back(0);
    }
    for (int i=0; i<_model.getVariables().at("y").getSize(); ++i) {
        if (cplex.getValue(_model.getVariables().at("y")[i]) > 0.9) {
            int from = _recourseEdges[i].from();
            int to = _recourseEdges[i].to();
            solutionEdges.push_back(std::make_pair(from, to));
            recourseCost += _recourseEdges[i].cost();
        }
    }
    computePath(solutionEdges);
    _pathCost = firstStageCost+ recourseCost;
    _firstStageX= firstStageX;
    _model.clearEnv();
return;
};

 



void TwoStage::solve(int batchId, 
    int numScenariosPerBatch) {

    auto scenarios = _scenarios.getScenarios(
        batchId, numScenariosPerBatch
    );
    assert(scenarios.size() == numScenariosPerBatch);

    populateConstraints();
    IloNumArray cost(_model.getEnv());

    for (int i=0; i<_firstStageEdges.size(); ++i) {
        auto edge = _firstStageEdges[i];
        auto recourseEdge = _recourseEdges[i];
        int from = edge.from();
        int to = edge.to();
        double directCost = edge.cost();
        double recourseCost = 0.0;
        if (_hasRecourseEdge[i]) {
            recourseCost = recourseEdge.cost() - directCost;
            double multiplier = 0;
            for (int j=0; j<scenarios.size(); ++j) {
                multiplier += scenarios[j][from];
            }
            recourseCost *= (multiplier/scenarios.size());
        }
        double effectiveCost = directCost + recourseCost;
        cost.add(effectiveCost);
    }
    
    _model.getModel().add(
        IloMinimize(_model.getEnv(), 
        IloScalProd(cost, _model.getVariables().at("x"))
    ));

    IloCplex cplex(_model.getEnv());
    cplex.extract(_model.getModel());
    cplex.setOut(_model.getEnv().getNullStream());
    cplex.use(addLazyCallback(_model.getEnv(), 
        _model,
        _firstStageEdges, 
        _edgeMap, 
        getNumVertices(), 
        getSource(), 
        getDestination()
    ));
    cplex.solve();


    double firstStageCost = 0.0;
    double secondStageCost = 0.0;
    std::vector<std::tuple<int, int>> solutionEdges;
    std::vector<int> solutionEdgeIds;
    for (int i=0; i<_model.getVariables().at("x").getSize(); ++i) {
        if (cplex.getValue(_model.getVariables().at("x")[i]) > 0.9) {
            int from = _firstStageEdges[i].from();
            int to = _firstStageEdges[i].to();
            solutionEdges.push_back(std::make_pair(from, to));
            solutionEdgeIds.push_back(i);
            firstStageCost += _firstStageEdges[i].cost();
            auto recourseEdge = _recourseEdges[i];
            double recourseCost = 0.0;
            if (_hasRecourseEdge[i]) {
                recourseCost = recourseEdge.cost() -
                    _firstStageEdges[i].cost();
                double multiplier = 0;
                for (int j=0; j<scenarios.size(); ++j) {
                    multiplier += scenarios[j][from];
                }
                recourseCost *= (multiplier/scenarios.size());
            }
            secondStageCost += recourseCost;
        }
    }

    computePath(solutionEdges);
    computeUB(solutionEdgeIds);
    _firstStageCost = firstStageCost;
    _secondStageCost = secondStageCost;
    _pathCost = firstStageCost + secondStageCost;
    writeSolution(batchId, numScenariosPerBatch);
    return;
};
void TwoStage::populateEdges() {
    auto coords = getCoords();
    auto satelliteCoords = getInstance().getSatelliteCoords();
    std::vector<Edge> edges;
    std::vector<Edge> recourseEdges;
    std::vector<bool> hasRecourseEdge;
    std::unordered_map<std::tuple<int, int>, int> edgeMap;
    std::vector<std::vector<int>> recoursePaths;

    for (int i=0; i<getNumVertices(); ++i) {
        for (int j=0; j<getNumVertices(); ++j) {
            if (i == j) continue;
            if (i == getSource() && j == getDestination()) continue;
            if (j == getSource()) continue;
            if (i == getDestination()) continue;
            auto ith = coords.at(i);
            auto jth = coords.at(j);
            std::vector<double> qInitial = {std::get<0>(ith), std::get<1>(ith), std::get<2>(ith)};
            std::vector<double> qFinal = {std::get<0>(jth), std::get<1>(jth), std::get<2>(jth)};
            DubinsPath path = getDubinsShortestPath(qInitial, qFinal, getInstance().getTurnRadius());
            double directCost = path.getLength();
            auto satellites = getInstance().getSatellitesAtTarget(i);
            double recourseCost;
            if (satellites.size() == 0) {
                hasRecourseEdge.push_back(false);
                recourseEdges.push_back(Edge());
                recoursePaths.push_back({});
            }
            else {
                hasRecourseEdge.push_back(true);
                auto kth = satelliteCoords.at(satellites[0]); 
                std::vector<double> qIntermediate = {std::get<0>(kth), std::get<1>(kth), std::get<2>(kth)};
                path = getDubinsShortestPath(qInitial, qIntermediate, getInstance().getTurnRadius());
                recourseCost = path.getLength();
                path = getDubinsShortestPath(qIntermediate, qFinal, getInstance().getTurnRadius());
                recourseCost += path.getLength();
                recourseEdges.push_back(Edge(i, j, recourseCost));
                recoursePaths.push_back({i, satellites[0], j});
            }
            edges.push_back(Edge(i, j, directCost));
            edgeMap.insert({std::make_tuple(i, j), edges.size()-1});
        }
    }

    _firstStageEdges = edges;
    _hasRecourseEdge = hasRecourseEdge;
    _recourseEdges = recourseEdges;
    _recoursePaths = recoursePaths;
    _edgeMap = edgeMap;
    return;
};
void TwoStage::populateConstraints() {

    Model model;
    model.getModel().setName(getInstance().getName().c_str());

    IloNumVarArray x(model.getEnv());
    model.getVariables().insert({"x", x});

    for (int i=0; i<_firstStageEdges.size(); ++i) {
        auto edge = _firstStageEdges[i];
        std::string varname = "x_" + std::to_string(edge.from()) +  "_" + std::to_string(edge.to());
        IloNumVar var(model.getEnv(), 0, 1, ILOINT, varname.c_str());
        model.getVariables().at("x").add(var);
    }

    for (int i=0; i<getNumVertices(); ++i) {
        IloExpr inExpr(model.getEnv());
        IloExpr outExpr(model.getEnv());
        for (int j=0; j<getNumVertices(); ++j) {
            if (i == j) continue; 
            auto inItr = _edgeMap.find(std::make_tuple(j, i));
            auto outItr = _edgeMap.find(std::make_tuple(i, j));
            if (inItr != _edgeMap.end()) {
                inExpr += model.getVariables().at("x")[inItr->second];
            }
            if (outItr != _edgeMap.end()) {
                outExpr += model.getVariables().at("x")[outItr->second];
            }
        }
        if (i != getDestination()) {
            std::string constrName = "out_" + std::to_string(i);
            IloRange constrOut(model.getEnv(), 1, outExpr, 1, constrName.c_str());
            model.getConstraints().push_back(constrOut);
        }
        if (i != getSource()) {
            std::string constrName = "in_" + std::to_string(i);
            IloRange constrIn(model.getEnv(), 1, inExpr, 1, constrName.c_str());
            model.getConstraints().push_back(constrIn);
        }
    }

    for (IloRange constr : model.getConstraints())  {
        model.getModel().add(constr);
    }

    setModel(model);
    return;
};


void TwoStage::solveDeterministic() {
    
    populateConstraints();
    IloNumArray cost(_model.getEnv());
    for (int i=0; i<_firstStageEdges.size(); ++i) {
        auto edge = _firstStageEdges[i];
        cost.add(edge.cost());
    }

    _model.getModel().add(
        IloMinimize(_model.getEnv(), 
        IloScalProd(cost, _model.getVariables().at("x"))
    ));

    IloCplex cplex(_model.getEnv());
    cplex.extract(_model.getModel());
    cplex.exportModel("test.lp");
    cplex.setOut(_model.getEnv().getNullStream());
    cplex.use(addLazyCallback(_model.getEnv(), 
        _model,
        _firstStageEdges, 
        _edgeMap, 
        getNumVertices(), 
        getSource(), 
        getDestination()
    ));
    cplex.solve();
    
    double firstStageCost = 0.0;
    std::vector<std::tuple<int, int>> solutionEdges;
    std::vector<int> solutionEdgeIds;
    for (int i=0; i<_model.getVariables().at("x").getSize(); ++i) {
        if (cplex.getValue(_model.getVariables().at("x")[i]) > 0.9) {
            int from = _firstStageEdges[i].from();
            int to = _firstStageEdges[i].to();
            
            solutionEdges.push_back(std::make_pair(from, to));
            solutionEdgeIds.push_back(i);
            firstStageCost += _firstStageEdges[i].cost();
        }
    }

    computePath(solutionEdges);
    computeUB(solutionEdgeIds);
    _firstStageCost = firstStageCost;
    _secondStageCost = 0.0;
    _pathCost = _firstStageCost + _secondStageCost;
   
    return;
};
void TwoStage::computePath(
    std::vector<std::tuple<int, int>> & edges) {

    std::vector<int> solution;
    solution.push_back(getSource());

    while (solution.back() != getDestination()) {
        auto it = std::find_if(edges.begin(), edges.end(),
                    [&solution](const std::tuple<int,int>& edge) 
                    {return std::get<0>(edge) == solution.back(); });
        solution.push_back(std::get<1>(*it));
    }

    _path = solution;
    return;
};

void TwoStage::computeUB(
    std::vector<int> & edgeIds) {
    
    std::vector<double> batchUB;
    for (int i=1; i<11; ++i) {
        auto scenarios = _scenarios.getUBScenarios(i);
        assert(scenarios.size() == 100);
        double firstStageCost = 0.0;
        double secondStageCost = 0.0;

        for (auto id : edgeIds) {
            int from = _firstStageEdges[id].from();
            firstStageCost += _firstStageEdges[id].cost();
            auto recourseEdge = _recourseEdges[id];
            double recourseCost = 0.0;
            if (_hasRecourseEdge[id]) {
                recourseCost = recourseEdge.cost() - 
                    _firstStageEdges[id].cost();
                double multiplier = 0;
                for (int j=0; j<scenarios.size(); ++j) 
                    multiplier += scenarios[j][from];
                recourseCost *= (multiplier/scenarios.size());
            }
            secondStageCost += recourseCost;
        }  
        batchUB.push_back(firstStageCost + secondStageCost);  
    }

    double sum = std::accumulate(batchUB.begin(), 
        batchUB.end(), 0.0);
    double mean = sum / batchUB.size();

    std::vector<double> diff(batchUB.size());
    std::transform(
        batchUB.begin(), 
        batchUB.end(), 
        diff.begin(), 
        [mean](double x) { return x - mean; });

    double sqSum = std::inner_product(
        diff.begin(), diff.end(), diff.begin(), 0.0);
    double stddev = std::sqrt(sqSum / (batchUB.size()-1));

    _ub = std::make_tuple(mean, stddev);
    
    std::ofstream outfile;//added
    outfile.open("data.txt");//added
    outfile<<mean<<" " <<stddev<<std::endl; //added
    outfile.close();//added*/

    double kappa = 2.2621; // alpha = 0.05, degrees of freedom = 9

    double lower = mean - (kappa*stddev)/std::sqrt(batchUB.size());
    double upper = mean + (kappa*stddev)/std::sqrt(batchUB.size());

    _ubRange = std::make_tuple(lower, upper);
   
    return;
}

void TwoStage::writeSolution(int batchId, 
    int numScenarios) {

    std::string file = _instance.getName();
    std::string path = "../output/";
    std::string filename = path + "b" 
        + std::to_string(batchId) + "-n" 
        + std::to_string(numScenarios) + "-" 
        + file;

    std::ofstream outfile;
    outfile.open(filename);

    outfile << batchId << std::endl 
        << numScenarios << std::endl 
        << _pathCost << std::endl 
        << _path.size();

    for (int i=0; i<_path.size(); ++i) {
        if (i%10 == 0) 
            outfile << std::endl;
        outfile << _path[i] << " ";
    }
    
    outfile << std::endl;

    outfile << std::get<0>(_ubRange) << " "
        << std::get<1>(_ubRange);
    outfile << std::endl;
    outfile << std::get<0>(_ub) << " "
        << std::get<1>(_ub);
    outfile.close();
    return;
}
