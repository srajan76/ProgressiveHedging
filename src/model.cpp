#include "model.hpp"

Model::Model() : 
    _env(), _model(IloModel(_env)), 
    _variables(), _constraints() {};