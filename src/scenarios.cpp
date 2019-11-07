#include "scenarios.hpp"

#include <random> 
#include <iostream>

Scenarios::Scenarios() :
    _seed(2019),
    _numTargets(), 
    _numScenarios(2000),
    _maxScenarios(1000),
    _prob(),
    _probofvisit(0.50),
    _omega() {};

void Scenarios::generateScenarios() {
    _omega.resize(_numScenarios);
    _prob.resize(_numTargets, _probofvisit);
    std::mt19937 generator(_seed);

    for (int i=0; i<_numScenarios; ++i) {
        for (int j=0; j<_numTargets; ++j) {
            std::bernoulli_distribution pr(_prob[j]);
            _omega[i].push_back(pr(generator));
        }
    }
    return;
};

