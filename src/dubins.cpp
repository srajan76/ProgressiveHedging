#include "dubins.hpp"

#include <iostream>

/* constructors */
DubinsPath::DubinsPath() : _qInitial(3, 0.0), _lengths(3, 0.0), _turnRadius(1.0), _pathType(LSL), _isComputed(false), _errCode(edubok) {};

DubinsPath::DubinsPath(const DubinsPath& d) {
    _qInitial = d._qInitial;
    _lengths = d._lengths;
    _turnRadius = d._turnRadius;
    _pathType = d._pathType;
    _isComputed = d._isComputed;
    _errCode = d._errCode;
};

DubinsPath::DubinsPath(std::vector<double> qInitial, std::vector<double> lengths, double turnRadius, DubinsPathType pathType) {
    _qInitial = qInitial;
    _lengths = lengths;
    _turnRadius = turnRadius;
    _pathType = pathType;
    _isComputed = false;
    _errCode = (turnRadius < 0) ? edubbadrho : edubok;
};

DubinsPath::DubinsPath(ErrCode errCode) {
    _qInitial = std::vector<double>(3,0.0);
    _lengths = std::vector<double>(3,0.0);
    _turnRadius = 0.0;
    _pathType = LSL;
    _isComputed = false;
    _errCode = errCode;
};

/* functions */
void DubinsPath::setLengths(std::vector<double>& lengths) {
    _lengths[0] = lengths[0];
    _lengths[1] = lengths[1];
    _lengths[2] = lengths[2];
};

void DubinsPath::setComputed() {
    _isComputed = true;
};

void DubinsPath::setInitialPoint(std::vector<double>& qInitial) {
    _qInitial[0] = qInitial[0];
    _qInitial[1] = qInitial[1];
    _qInitial[2] = qInitial[2];
};

void DubinsPath::setTurnRadius(double turnRadius) {
    _turnRadius = turnRadius;
};

void DubinsPath::setPathType(DubinsPathType pathType) {
    _pathType = pathType;
};

void DubinsPath::setErrCode(ErrCode errCode) {
    _errCode = errCode;
};

double DubinsPath::getTurnRadius() const {
    return _turnRadius;
};

DubinsPathType DubinsPath::getPathType() const {
    return _pathType;
};

double DubinsPath::getLength() const {
    double sum = 0.0;
    for (auto& n : _lengths)
        sum += n;
    return sum * _turnRadius;
};

bool DubinsPath::isPathComputed() const {
    return _isComputed;
};

ErrCode DubinsPath::getErrCode() const {
    return _errCode;
};

double DubinsPath::getSegmentLength(int i) const {
    if (i < 0 || i > 2)
        return std::numeric_limits<double>::infinity();
    else
        return _lengths[i] * _turnRadius;

};

double DubinsPath::getSegmentLengthNormalized(int i) const {
    if (i < 0 || i > 2)
        return std::numeric_limits<double>::infinity();
    else
        return _lengths[i];
};


std::vector<double> DubinsPath::getEndPoint() const {
    return this->getSample(this->getLength() - 1e-5);
};

std::vector<double> DubinsPath::getSample(double t) const {
    /* tprime is the normalised variant of the parameter t */
    double tprime = t / _turnRadius;
    std::vector<double> qi(3,0.0); /* The translated initial configuration */
    std::vector<double> q1; /* end-of segment 1 */
    std::vector<double> q2; /* end-of segment 2 */
    std::vector<double> q; /* point to return */
    const SegmentType* types = DirData[_pathType];
    double p1, p2;

    if (t < 0 || t > this->getLength()) {
        throw std::out_of_range("sampling parameter out-of-range");
    }

    /* initial configuration */
    qi[0] = 0.0;
    qi[1] = 0.0;
    qi[2] = _qInitial[2];

    /* generate the target configuration */
    p1 = _lengths[0];
    p2 = _lengths[1];
    q1 = transformPoint(p1, qi, types[0]);
    q2 = transformPoint(p2, q1, types[1]);
    if (tprime < p1) {
        q = transformPoint(tprime, qi, types[0]);
    }
    else {
        if (tprime < (p1+p2)) {
            q = transformPoint(tprime-p1, q1, types[1]);
        }
        else {
            q = transformPoint(tprime-p1-p2, q2, types[2]);
        }
    }

    /* scale the target configuration, translate back to the original starting point */
    q[0] = q[0] * _turnRadius + _qInitial[0];
    q[1] = q[1] * _turnRadius + _qInitial[1];
    q[2] = mod2pi(q[2]);

    return q;
};

std::vector<std::vector<double>> DubinsPath::samplePath(double stepSize) const {
    std::vector<double> q;
    std::vector<std::vector<double>> configurations;
    double x = 0.0;
    double length = this->getLength();

    if (stepSize < 0 || stepSize > length) {
        throw std::out_of_range("sampling parameter out-of-range");
    }

    while (x < length) {
        std::vector<double> sample = this->getSample(x);
        configurations.push_back(sample);
        x += stepSize;
    }

    return configurations;
}


/* constructor */
DubinsIntermediateResults::DubinsIntermediateResults() : _alpha(0.0), _beta(0.0), _d(0.0), _sa(0.0), _sb(0.0), _ca(1.0), _cb(1.0), _cab(1.0), _dsq(0.0) {};

DubinsIntermediateResults::DubinsIntermediateResults(const DubinsIntermediateResults& d) {
    _alpha = d._alpha;
    _beta = d._beta;
    _d = d._d;
    _sa = d._sa;
    _sb = d._sb;
    _ca = d._ca;
    _cb = d._cb;
    _cab = d._cab;
    _dsq = d._dsq;
};

DubinsIntermediateResults::DubinsIntermediateResults(const std::vector<double> q0, const std::vector<double> q1, double rho) {
    double dx = q1[0] - q0[0];
    double dy = q1[1] - q0[1];
    double D = sqrt(dx*dx + dy*dy);
    double d = D / rho;
    double theta = 0;

    if (d > 0) {theta = mod2pi(std::atan2(dy, dx));}
    _alpha = mod2pi(q0[2] - theta);
    _beta = mod2pi(q1[2] - theta);
    _d = d;
    _sa = std::sin(_alpha);
    _sb = std::sin(_beta);
    _ca = std::cos(_alpha);
    _cb = std::cos(_beta);
    _cab = std::cos(_alpha - _beta);
    _dsq = d*d;
};

/* other dubins functions */
DubinsPath getDubinsShortestPath(std::vector<double> qInitial, std::vector<double> qFinal, double rho) {
    DubinsPath path(edubok);
    path.setInitialPoint(qInitial);
    ErrCode errCode;
    std::vector<double> params(3,0.0);
    double cost;
    double bestCost = std::numeric_limits<double>::infinity();
    int bestWord = -1;

    if (rho < 0.0) {
        path.setErrCode(edubbadrho);
        path.setComputed();
        return path;
    }
    DubinsIntermediateResults in(qInitial, qFinal, rho);

    path.setTurnRadius(rho);

    for(auto i = 0; i < 6; i++) {
        DubinsPathType pathType = (DubinsPathType)i;
        errCode = (ErrCode)findDubinsPath(in, pathType, params);
        if (errCode == edubok) {
            cost = params[0] + params[1] + params[2];
            if (cost < bestCost) {
                bestWord = i;
                bestCost = cost;
                path.setLengths(params);
                path.setPathType(DubinsPathType(bestWord));
                path.setErrCode(edubok);
                path.setComputed();
            }
        }
    }

    if (bestWord == -1) {
        path.setErrCode(edubnopath);
        path.setComputed();
        return path;
    }

    return path;
};

DubinsPath getDubinsPath(std::vector<double> qInitial, std::vector<double> qFinal, double rho, DubinsPathType pathType) {
    DubinsPath path(edubok);
    path.setInitialPoint(qInitial);
    ErrCode errCode;
    std::vector<double> params(3,0.0);

    if (rho < 0.0) {
        path.setErrCode(edubbadrho);
        path.setComputed();
        return path;
    }

    path.setTurnRadius(rho);
    DubinsIntermediateResults in(qInitial, qFinal, rho);
    errCode = (ErrCode)findDubinsPath(in, pathType, params);
    if(errCode == edubok) {
        path.setLengths(params);
        path.setPathType(pathType);
        path.setErrCode(edubok);
        path.setComputed();
    }
    else {
        path.setComputed();
        path.setErrCode(errCode);
    }

    return path;
};

std::vector<double> transformPoint(double t, const std::vector<double> qInitial, SegmentType type) {
    double st = std::sin(qInitial[2]);
    double ct = std::cos(qInitial[2]);
    std::vector<double> qFinal(3,0.0);
    if (type == L_SEG) {
        qFinal[0] = +std::sin(qInitial[2]+t) - st;
        qFinal[1] = -std::cos(qInitial[2]+t) + ct;
        qFinal[2] = t;
    }
    else if (type == R_SEG) {
        qFinal[0] = -std::sin(qInitial[2]-t) + st;
        qFinal[1] = +std::cos(qInitial[2]-t) - ct;
        qFinal[2] = -t;
    }
    else if (type == S_SEG) {
        qFinal[0] = ct * t;
        qFinal[1] = st * t;
        qFinal[2] = 0.0;
    }
    qFinal[0] += qInitial[0];
    qFinal[1] += qInitial[1];
    qFinal[2] += qInitial[2];

    return qFinal;
}

ErrCode getDubinsLSL(DubinsIntermediateResults& in, std::vector<double>& out) {
    double tmp0, tmp1, p_sq;

    tmp0 = in._d + in._sa - in._sb;
    p_sq = 2 + in._dsq - (2*in._cab) + (2 * in._d * (in._sa - in._sb));

    if (p_sq >= 0) {
        tmp1 = std::atan2( (in._cb - in._ca), tmp0 );
        out[0] = mod2pi(tmp1 - in._alpha);
        out[1] = sqrt(p_sq);
        out[2] = mod2pi(in._beta - tmp1);
        return edubok;
    }
    return edubnopath;
};


ErrCode getDubinsRSR(DubinsIntermediateResults& in, std::vector<double>& out) {
    double tmp0 = in._d - in._sa + in._sb;
    double p_sq = 2 + in._dsq - (2 * in._cab) + (2 * in._d * (in._sb - in._sa));
    if (p_sq >= 0) {
        double tmp1 = std::atan2((in._ca - in._cb), tmp0);
        out[0] = mod2pi(in._alpha - tmp1);
        out[1] = std::sqrt(p_sq);
        out[2] = mod2pi(tmp1-in._beta);
        return edubok;
    }
    return edubnopath;
};

ErrCode getDubinsLSR(DubinsIntermediateResults& in, std::vector<double>& out) {
    double p_sq = -2 + (in._dsq) + (2 * in._cab) + (2 * in._d * (in._sa + in._sb));
    if( p_sq >= 0 ) {
        double p    = sqrt(p_sq);
        double tmp0 = atan2( (-in._ca - in._cb), (in._d + in._sa + in._sb) ) - atan2(-2.0, p);
        out[0] = mod2pi(tmp0 - in._alpha);
        out[1] = p;
        out[2] = mod2pi(tmp0 - mod2pi(in._beta));
        return edubok;
    }
    return edubnopath;
};

ErrCode getDubinsRSL(DubinsIntermediateResults& in, std::vector<double>& out) {
    double p_sq = -2 + in._dsq + (2 * in._cab) - (2 * in._d * (in._sa + in._sb));
    if( p_sq >= 0 ) {
        double p    = sqrt(p_sq);
        double tmp0 = atan2( (in._ca + in._cb), (in._d - in._sa - in._sb) ) - atan2(2.0, p);
        out[0] = mod2pi(in._alpha - tmp0);
        out[1] = p;
        out[2] = mod2pi(in._beta - tmp0);
        return edubok;
    }
    return edubnopath;
};

ErrCode getDubinsRLR(DubinsIntermediateResults& in, std::vector<double>& out) {
    double tmp0 = (6. - in._dsq + 2*in._cab + 2*in._d*(in._sa - in._sb)) / 8.;
    double phi  = atan2( in._ca - in._cb, in._d - in._sa + in._sb );
    if( fabs(tmp0) <= 1) {
        double p = mod2pi((2*M_PI) - acos(tmp0) );
        double t = mod2pi(in._alpha - phi + mod2pi(p/2.));
        out[0] = t;
        out[1] = p;
        out[2] = mod2pi(in._alpha - in._beta - t + mod2pi(p));
        return edubok;
    }
    return edubnopath;
};

ErrCode getDubinsLRL(DubinsIntermediateResults& in, std::vector<double>& out) {
    double tmp0 = (6. - in._dsq + 2*in._cab + 2*in._d*(in._sb - in._sa)) / 8.;
    double phi = atan2( in._ca - in._cb, in._d + in._sa - in._sb );
    if (fabs(tmp0) <= 1) {
        double p = mod2pi( 2*M_PI - acos( tmp0) );
        double t = mod2pi(-in._alpha - phi + p/2.);
        out[0] = t;
        out[1] = p;
        out[2] = mod2pi(mod2pi(in._beta) - in._alpha -t + mod2pi(p));
        return edubok;
    }
    return edubnopath;
};

ErrCode findDubinsPath(DubinsIntermediateResults& in, DubinsPathType pathType, std::vector<double>& out) {
    ErrCode result;
    switch(pathType) {
    case LSL:
        result = getDubinsLSL(in, out);
        break;
    case RSL:
        result = getDubinsRSL(in, out);
        break;
    case LSR:
        result = getDubinsLSR(in, out);
        break;
    case RSR:
        result = getDubinsRSR(in, out);
        break;
    case LRL:
        result = getDubinsLRL(in, out);
        break;
    case RLR:
        result = getDubinsRLR(in, out);
        break;
    default:
        result = edubnopath;
    }
    return result;
};