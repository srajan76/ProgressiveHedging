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
    _recoursePaths(), 
    _edgeMap(), 
    _satEdgeMap(),
    _path(), 
    _firstStageCost(), 
    _secondStageCost(), 
    _pathCost(), 
    _detEdges(),
    _firstStageX() {};

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

void TwoStage::calcDetCostPath(std::vector<int> &w,std::vector<int>& detPath){
    auto tinit = std::make_tuple(detPath[0], detPath[1]);
    double pathcost = detCostCompute(_firstStageEdges, tinit );
    std::cout<<"init det path cost 0-1  "<< pathcost<<std::endl;
    for( int i =1; i <detPath.size()-1; ++i){
        std::tuple<int,int> t(detPath[i], detPath[i+1]);
        if(w[std::get<0>(t)]==0)
            pathcost += detCostCompute(_recourseEdges,t);
        else{            
            populateConstraintsDet(t);
            solvePHscenarioDet(t);
            pathcost +=getPathCost();
        }
        std::cout<<"DeT PATHCOST FOR EVERY Step in scenario   "<< i<<"  "<<pathcost<<std::endl;
    }//for ends
    _pathCost = pathcost;
    std::cout<<"DeT PATHCOST FOR FULL scenario   "<<_pathCost<<std::endl;
return;
};
double TwoStage::detCostCompute(std::vector<Edge>&v, std::tuple<int,int> &z){
     double pathcost;
     for (auto const & a :v)
        if(a.from()== std::get<0>(z) && a.to()==std::get<1>(z)){
            pathcost = a.cost(); 
            break;
        }
return pathcost;
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

void TwoStage::populateConstraintsDet(std::tuple<int,int>& t){
    Model model;
    std::vector<std::tuple<int,int>> detEdges;
    auto satAttargetI = getInstance().getSatellitesAtTarget(std::get<0>(t));
    std::vector<int> satTargets={std::get<0>(t)};
    for (auto const & k : satAttargetI)
        satTargets.push_back(std::get<1>(_satEdgeMap[k]));
    auto satTargetsOut= satTargets;
    satTargets.push_back(std::get<1>(t));
    satTargets.erase(std::remove(satTargets.begin(), satTargets.end(),std::get<0>(t)),satTargets.end());
    auto satTargetsIn = satTargets;
                
    IloNumVarArray f(model.getEnv());
    model.getVariables().insert({"f", f});
    IloExprArray arcOut(model.getEnv(), satTargets.size());
    IloExprArray arcIn(model.getEnv(), satTargets.size());
    for ( int i =0; i < satTargets.size(); ++i){
        arcOut[i] = IloExpr(model.getEnv());
        arcIn[i] = IloExpr(model.getEnv());
    }
    int i =0;
    for (auto const & a : satTargetsOut){
        int j =0;
        for (auto const & b : satTargetsIn){
            if((a==std::get<0>(t) && b == std::get<1>(t)) || a==b) {j+=1;continue;}
            IloNumVar var(model.getEnv(), 0, 1, ILOINT);
            model.getVariables().at("f").add(var); 
            detEdges.push_back(std::make_tuple(a,b));
            arcOut[i]+=var;
            arcIn[j]+= var;
            j+=1;                         
        }
    i+=1;    
    }
    
    
    for (int i =0; i< satTargets.size(); ++i){
        std::string arcout = "arcout_" +i;
        IloRange arcouT(model.getEnv(), 1, arcOut[i], 1, arcout.c_str());
        model.getConstraints().push_back(arcouT);
        std::string arcin = "arcin_" +i;
        IloRange arciN(model.getEnv(), 1,arcIn[i], 1,arcin.c_str());
        model.getConstraints().push_back(arciN);
    }
    for (IloRange constr : model.getConstraints())  
        model.getModel().add(constr);
    
    setModel(model);
    _detEdges= detEdges;
    return;
};

void TwoStage::solvePHscenarioDet(std::tuple<int,int> t){
    IloExpr obj(_model.getEnv());
    IloNumArray cost(_model.getEnv());
    auto target = std::get<0>(t);
    auto nexttarget = std::get<1>(t);
    auto satAttargetI = getInstance().getSatellitesAtTarget(target);
    std::vector<int> satTargets={target};
    for (auto const & k : satAttargetI)
        satTargets.push_back(std::get<1>(_satEdgeMap[k]));
        auto satTargetsOut= satTargets;
        satTargets.push_back(nexttarget);
        satTargets.erase(std::remove(satTargets.begin(), satTargets.end(),std::get<0>(t)),satTargets.end());
        auto satTargetsIn = satTargets;
        for (auto const & a : satTargetsOut){
            for (auto const & b : satTargetsIn){
                if((a==target && b == nexttarget) || a==b) continue;
                for(auto itr = _recourseEdges.begin(); itr != _recourseEdges.end(); ++itr){
                    if ((*itr).from() ==a && (*itr).to()== b){   
                        cost.add((*itr).cost());
                        break;
                    }
                }
            }
        }
        
        obj = IloScalProd( cost, _model.getVariables().at("f"));
        _model.getModel().add(IloMinimize(_model.getEnv(), obj ));
        IloCplex cplex(_model.getEnv());
        cplex.extract(_model.getModel());
        cplex.setOut(_model.getEnv().getNullStream());
        cplex.use(addLazyCallback(_model.getEnv(), 
        _model,
        _detEdges,
        satTargetsIn,
        satTargetsOut, 
        getNumVertices(),
        getInstance().getNumSatellitesPerTarget()
        ));
        cplex.solve();
        _pathCost =cplex.getObjValue();
        //computePath(solutionEdges);
        std::cout<<"DETPATHCOST   "<<_pathCost<<std::endl;
        _model.clearEnv();
    return;
};
