#include <ilcplex/ilocplex.h>
#include <lemon/list_graph.h>
#include <lemon/hao_orlin.h>
#include <lemon/connectivity.h>
#include <optionParser.hpp>
#include "util.hpp"
#include "instance.hpp"
#include "model.hpp"
#include "edge.hpp"
#include "scenarios.hpp"
#include "twostage.hpp"

std::vector<std::vector<double>> calcW (std::vector<std::vector<int>> X_s, std::vector<double> meanX, std::vector<std::vector<double>> W, double rho){
    std::vector<std::vector<double>> Wnew;
    for (int p =0; p < X_s.size(); ++p){ 
            
            std::vector<double> temp;
            std::transform(X_s[p].begin(), X_s[p].end(),meanX.begin(),std::back_inserter(temp), std::minus<double>());
            std::vector<double> temp2;
            std::transform(temp.begin(), temp.end(), std::back_inserter(temp2), [rho](double i){return i*rho;}); 

            for ( int z =0; z< W[p].size(); ++z)
                W[p][z] += temp2[z];
            Wnew.push_back(W[p]);
                
            
    }
        return Wnew;
}

double calcNorm(std::vector<double> g){
    double total =0.0;
    for ( int y =0; y < g.size(); ++y)
        total+= g[y]*g[y];
    return std::sqrt(total);
}

int main(int argc, char* argv[]){

    op::OptionParser opt;
    opt.add_option("h", "help", "shows option help" ); 
    opt.add_option("p", "instance_path", "instance_path", "../data/" );
    opt.add_option("f", "file", "name of the instance file", "20-4-5-4.txt" ); 
    opt.add_option("s", "num_scenarios", "number of scenarios", "100");

    // parse the options and verify that all went well
    bool correct_parsing = opt.parse_options(argc, argv);
    
    if(!correct_parsing) return EXIT_FAILURE;
    if(op::str2bool(opt["h"])) { 
        opt.show_help();
        return 0;
    }

    Instance instance;
    instance.setName(opt["f"]);
    instance.setPath(opt["p"]);
    instance.readData();
   
    Scenarios scenarios;
    scenarios.setSeed(instance.getSeed());
    scenarios.setNumTargets(instance.getNumTargets());
    scenarios.setNumScenarios(std::stoi(opt["s"]));
    scenarios.generateScenarios();
    auto scen = scenarios.getScenariosPH();

    TwoStage formulation(instance,scenarios);
    formulation.initialize();
    formulation.populateEdgesPH();

    //defining the PH opt variables to be used
    std::vector<double> costs;
    std::vector<std::vector<int>> soltn; //This gives the path nodes
    std::vector<std::vector<int>> X_s; //this is a vector of vector ∈ {0,1} of the first stage solution
    std::vector<double> meanX;
    std::vector<std::vector<double>> W;
    const double rho = 0.5;
    const double epsilon =0.00001;
    int iteration=0;


    //solving the optimizaton x_s(0) = argminx,ys (c · x + fs · ys ) : (x, ys ) ∈ Qs
    std::string savepath = "../output/";
    std::ofstream outfile;
    std::string file = savepath +"O"+ instance.getName() ;
    std::cout <<file<<std::endl;
    outfile.open(file);
    outfile <<"PH ITERATION  "<<iteration <<std::endl;
    //std::cout <<"mainLoop 0 start"<<std::endl;
    for ( int i =0; i< scen.size(); ++i){
        outfile <<"Scenario  "<<i+1 <<std::endl;
        formulation.solvePHscenario(scen[i]);
        X_s.push_back(formulation.getfirstStageScenarioSolution());
        soltn.push_back(formulation.getPath());
        costs.push_back(formulation.getPathCost());
        for (int j=0; j< scen[i].size(); ++j)
            outfile << scen[i][j] << " " ;
        outfile << std::endl;
        for ( int j =0; j< soltn[i].size();++j)
            outfile << soltn[i][j] <<" ";
        outfile << costs[i] <<std::endl;
    
    }
    std::cout<<"SIZE OF X_S   "<<X_s.size()<<std::endl;
    std::cout<<"SIZE OF X_S[i]  "<<X_s[X_s.size()-1].size()<<std::endl;
    std::cout<<"SIZE OF W   "<<W.size()<<std::endl;
    
    std::cout<<"SIZE OF meanX   "<<meanX.size()<<std::endl;
    
    iteration+=1;
    soltn.clear();
    costs.clear();
    //Calculating mean first stage solution X_0 for all s
    std::transform(X_s[0].begin(), X_s[0].end(),X_s[1].begin(),std::back_inserter(meanX), std::plus<double>());
    for (int p =1; p < X_s.size()-1; ++p)  
        std::transform(meanX.begin(), meanX.end(),X_s[p+1].begin(),meanX.begin(), std::plus<double>());
    outfile << "********************************MEAN SOLUTION OF FIRST STAGE***************" <<std::endl;
    for (auto t= meanX.begin(); t!= meanX.end(); ++t){
        *t/=X_s.size();
        outfile << *t << " ";
    }
    outfile <<std::endl;
    //Initialize the value of W using 0th stage solution
    for (int p =0; p < X_s.size(); ++p){ 
        std::vector<double> temp;
        std::transform(X_s[p].begin(), X_s[p].end(),meanX.begin(),std::back_inserter(temp), std::minus<double>());
        std::vector<double> temp2;
        std::transform(temp.begin(), temp.end(), std::back_inserter(temp2), [rho](double i){return i*rho;});       
        W.push_back(temp2);
    }
    
    
    std::cout<<"SIZE OF X_S   "<<X_s.size()<<std::endl;
    std::cout<<"SIZE OF W   "<<W.size()<<std::endl;
    std::cout<<"SIZE OF W[i]  "<<W[W.size()-1].size()<<std::endl;
    std::cout<<"SIZE OF meanX   "<<meanX.size()<<std::endl;
    for ( auto z =0; z<X_s.size(); ++z )
        X_s[z].clear();
    X_s.clear();
    std:: cout <<"PH LOOP  "<<iteration<<std::endl;
    std::cout<<"SIZE OF X_S   "<<X_s.size()<<std::endl;
    std::cout<<"SIZE OF X_S[i]   "<<X_s[9].size()<<std::endl;
    //PH LOOP
    auto g = 100.0+epsilon;
    while (g> epsilon){
        // solving the optimizaton x_s(k)=  argminx,ys (c · x + w_s(k-1). x + rho/2 ||x- xbar(k-1)||^2+  fs · ys ) : (x, ys ) ∈ Qs
        outfile <<"PH ITERATION  "<<iteration <<std::endl;
        //std::vector<std::vector<int>> X_sp;
        for ( int i =0; i< scen.size(); ++i){
            outfile <<"Scenario  "<<i+1 <<std::endl;
            formulation.solvePHscenario(scen[i], W[i], meanX, rho);
            X_s.push_back(formulation.getfirstStageScenarioSolution());
            soltn.push_back(formulation.getPath());
            costs.push_back(formulation.getPathCost());
            
            for (int j=0; j< scen[i].size(); ++j)
                outfile << scen[i][j] << " " ;
            outfile << std::endl;
            
            for ( int j =0; j< soltn[i].size();++j)
                outfile << soltn[i][j] <<" ";
            outfile << costs[i] <<std::endl;
        }
        soltn.clear();
        costs.clear();
        //std::fill(meanX.begin(), meanX.end(),0.0);
        meanX.clear();
        std::cout<<"SIZE OF X_S  "<<X_s.size()<<std::endl;
        std::cout<<"SIZE OF X_S[i]   "<<X_s[X_s.size()-1].size()<<std::endl;
        std::cout<<"SIZE OF W   "<<W.size()<<std::endl;
        std::cout<<"SIZE OF W[i]  "<<W[W.size()-1].size()<<std::endl;
        std::cout<<"SIZE OF meanX   "<<meanX.size()<<std::endl;

        //calc new meanX
        std::transform(X_s[0].begin(), X_s[0].end(),X_s[1].begin(),std::back_inserter(meanX), std::plus<double>());
        for (int p =1; p < X_s.size()-1; ++p)  
            std::transform(meanX.begin(), meanX.end(),X_s[p+1].begin(),meanX.begin(), std::plus<double>());
        outfile << "********************************MEAN SOLUTION OF FIRST STAGE***************" <<std::endl;       
        for (auto t= meanX.begin(); t!= meanX.end(); ++t){
            *t/=X_s.size(); 
            outfile << *t << " ";
        }
        outfile <<std::endl;

        //calculate new W
        W= calcW(X_s, meanX, W, rho); 
        
        std::cout <<"W done"<<std::endl;
        std::cout<<"SIZE OF W   "<<W.size()<<std::endl;
        std::cout<<"SIZE OF W[i]  "<<W[W.size()-1].size()<<std::endl;
        //calculate g
        
        double gmat=0.0;
        for (int f =0; f< X_s.size();++f){
            std::vector<double> temp2;
            std::transform(X_s[f].begin(), X_s[f].end(), meanX.begin(), std::back_inserter(temp2), std::minus<double>());
            gmat+= calcNorm(temp2);
        }
        g = gmat/ X_s.size();
    
        //update values
        std:: cout <<iteration <<"   g(k)  "<<g<<std::endl;
        iteration+=1;
        for ( auto z =0; z<X_s.size(); ++z )
            X_s[z].clear();
        X_s.clear();
    } 
   outfile.close();
    return 0;
}



