#include "scenarios.hpp"

#include <random> 
#include <iostream>

Scenarios::Scenarios() :
    _seed(2019),
    _numTargets(), 
    _numScenarios(2000),
    _maxScenarios(1000),
    _prob(),
    _omega() {};

void Scenarios::generateScenarios() {
    _omega.resize(_numScenarios);
    _prob.resize(_numTargets, 0.333333333);
    std::mt19937 generator(_seed);

    for (int i=0; i<_numScenarios; ++i) {
        for (int j=0; j<_numTargets; ++j) {
            std::bernoulli_distribution pr(_prob[j]);
            _omega[i].push_back(pr(generator));
        }
    }
    return;
};

std::vector<std::vector<int>> Scenarios::getScenarios(
    int batchId, int scenariosPerBatch) const {
    
    if (batchId*scenariosPerBatch > _maxScenarios) {
        std::cerr << "batch id * scenario per batch should be less than or equal to" 
            << _maxScenarios << std::endl;
        exit(1);
    }

    int from = scenariosPerBatch * (batchId - 1);
    int to = scenariosPerBatch * batchId;
    std::vector<std::vector<int>> reduced(&_omega[from], &_omega[to]);
    
    return reduced;
};

std::vector<std::vector<int>> Scenarios::getUBScenarios(
    int batchId) const { 
    
    int from = 1000 + 100 * (batchId - 1);
    int to = 1000 + 100 * batchId;
    std::vector<std::vector<int>> reduced(&_omega[from], &_omega[to]);
    
    return reduced;
};