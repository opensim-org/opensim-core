#ifndef GEODESIC_CURVE_STATE_H_
#define GEODESIC_CURVE_STATE_H_

#include <functional>
#include <OpenSim/OpenSim.h>

namespace OpenSim
{

struct GeodesicVariation
{
    // ds in thesis.
    double dTangential = NAN;
    double dTheta      = NAN;
    double dBeta       = NAN;
    double dLength     = NAN;
};

struct DarbouxFrameVariation
{
    DarbouxFrameVariation() = default;

    SimTK::Vec3 t{NAN, NAN, NAN};
    SimTK::Vec3 n{NAN, NAN, NAN};
    SimTK::Vec3 b{NAN, NAN, NAN};
};

struct DarbouxFrame
{
    DarbouxFrame() = default;

    DarbouxFrame(SimTK::Vec3 velocity, SimTK::Vec3 surfaceNormal);

    // TODO use rotation vector?
    SimTK::Vec3 t{NAN, NAN, NAN};
    SimTK::Vec3 n{NAN, NAN, NAN};
    SimTK::Vec3 b{NAN, NAN, NAN};
};

struct JacobiFieldScalar
{
    using Dot = SimTK::Vec2;
    JacobiFieldScalar(double a, double aDot)
        : value(a), derivative(aDot) {}

    Dot calcDerivative(double gaussianCurvature) const;

    JacobiFieldScalar &operator+=(const Dot &derivative);

    double value = NAN;
    double derivative = NAN;
};

/// State of the geodesic at a point along the curve.
struct GeodesicCurveState
{
    DarbouxFrameVariation calcFrameToDBeta() const;

    DarbouxFrame frame;
    SimTK::Vec3 position {NAN, NAN, NAN};
    double normalCurvature = NAN;
    double geodesicTorsion = NAN;
    JacobiFieldScalar a;
    JacobiFieldScalar r;
};

class GeodesicCurve
{
    GeodesicCurveState start;
    GeodesicCurveState end;
    double length = NAN;
    std::vector<SimTK::Vec3>& pointsLog;
};

}

#endif
