#ifndef IMPLICIT_SURFACE_STATE_H_
#define IMPLICIT_SURFACE_STATE_H_

#include <OpenSim/OpenSim.h>
#include "GeodesicCurveState.h"
#include <iostream>

namespace OpenSim
{

struct ImplicitSurfaceState
{
    using Dot = SimTK::Vec6;

    Dot calcDerivative(const SimTK::Vec3& acceleration) const;

    ImplicitSurfaceState &operator+=(const Dot &i);

    SimTK::Vec3 postion {NAN, NAN, NAN};
    SimTK::Vec3 velocity {NAN, NAN, NAN};
};

struct ImplicitGeodesicState: ImplicitSurfaceState
{
    using Dot = std::tuple<ImplicitSurfaceState::Dot, JacobiFieldScalar::Dot,JacobiFieldScalar::Dot>;

    explicit ImplicitGeodesicState(ImplicitSurfaceState q)
        :
            ImplicitSurfaceState(std::move(q)),
            a({0., 1.}),
            r({1., 0.}){}

    Dot calcDerivative(
            const SimTK::Vec3& acceleration,
            double gaussianCurvature) const;

    ImplicitGeodesicState &operator+=(const Dot &derivative);

    JacobiFieldScalar a;
    JacobiFieldScalar r;
};

std::ostream& operator<<(std::ostream& os, const ImplicitSurfaceState& q);
std::ostream& operator<<(std::ostream& os, const JacobiFieldScalar& x);
std::ostream& operator<<(std::ostream& os, const ImplicitGeodesicState& x);

}

#endif
