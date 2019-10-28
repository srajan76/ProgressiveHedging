#ifndef DUBINS_HPP
#define DUBINS_HPP

#pragma once

#include <vector>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <tuple>

#define _USE_MATH_DEFINES

enum DubinsPathType {LSL = 0, LSR = 1, RSL = 2,  RSR = 3, RLR = 4, LRL = 5};
enum ErrCode {edubok = 0, edubcoconfigs = 1, edubparam = 2, edubbadrho = 3, edubnopath = 4};
enum SegmentType {L_SEG = 0, S_SEG = 1, R_SEG = 2};
const SegmentType DirData[][3] = {
    { L_SEG, S_SEG, L_SEG },
    { L_SEG, S_SEG, R_SEG },
    { R_SEG, S_SEG, L_SEG },
    { R_SEG, S_SEG, R_SEG },
    { R_SEG, L_SEG, R_SEG },
    { L_SEG, R_SEG, L_SEG }
};

class DubinsPath {

    protected:
        std::vector<double>         _qInitial = {0.0, 0.0, 0.0};   // initial point
        std::vector<double>         _lengths = {0.0, 0.0, 0.0};    // segment lengths
        double                      _turnRadius = 1.0;             // turn radius
        DubinsPathType              _pathType = LSL;               // path type
        bool                        _isComputed = false;           // path computed?
        ErrCode                     _errCode = edubok;             // error code

    public:
        DubinsPath();
        DubinsPath(const DubinsPath& d);
        DubinsPath(std::vector<double>, std::vector<double>, double, DubinsPathType);
        DubinsPath(ErrCode);

        void setLengths(std::vector<double>&);
        void setComputed();
        void setInitialPoint(std::vector<double>&);
        void setTurnRadius(double);
        void setPathType(DubinsPathType);
        void setErrCode(ErrCode);

        double getTurnRadius() const;
        DubinsPathType getPathType() const;
        double getLength() const;
        bool isPathComputed() const;
        ErrCode getErrCode() const;

        double getSegmentLength(int) const;
        double getSegmentLengthNormalized(int) const;
        std::vector<double> getSample(double) const;
        std::vector<double> getEndPoint() const;
        std::vector<std::vector<double>> samplePath(double) const;
};

class DubinsIntermediateResults {

    public:
        double                      _alpha = 0.0;                   // transformed alpha
        double                      _beta = 0.0;                    // transformed beta
        double                      _d = 0.0;                       // transformed d
        double                      _sa = std::sin(0.0);            // sin(alpha)
        double                      _sb = std::sin(0.0);            // sin(alpha)
        double                      _ca = std::cos(0.0);            // cos(alpha)
        double                      _cb = std::cos(0.0);            // cos(beta)
        double                      _cab = std::cos(0.0);          // cos(alpha-beta)
        double                      _dsq = 0.0;                    // d*d

        DubinsIntermediateResults();
        DubinsIntermediateResults(const DubinsIntermediateResults& d);
        DubinsIntermediateResults(const std::vector<double>, const std::vector<double>, double);
};

DubinsPath getDubinsShortestPath(std::vector<double>, std::vector<double>, double);
DubinsPath getDubinsPath(std::vector<double>, std::vector<double>, double, DubinsPathType);
std::vector<double> transformPoint(double, const std::vector<double>, SegmentType);

auto fmodr = [](double x, double y) {return x - y*std::floor(x/y);};
auto mod2pi = [](double theta) {return fmodr(theta, 2*M_PI);};

ErrCode getDubinsLSL(DubinsIntermediateResults&, std::vector<double>&);
ErrCode getDubinsLSR(DubinsIntermediateResults&, std::vector<double>&);
ErrCode getDubinsRSL(DubinsIntermediateResults&, std::vector<double>&);
ErrCode getDubinsLSR(DubinsIntermediateResults&, std::vector<double>&);
ErrCode getDubinsRSR(DubinsIntermediateResults&, std::vector<double>&);
ErrCode getDubinsLRL(DubinsIntermediateResults&, std::vector<double>&);
ErrCode findDubinsPath(DubinsIntermediateResults&, DubinsPathType, std::vector<double>&);

#endif