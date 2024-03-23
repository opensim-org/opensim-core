#ifndef IMPLICIT_SURFACE_PARAMS_H_
#define IMPLICIT_SURFACE_PARAMS_H_

#include "GeodesicCurveState.h"
#include "ImplicitSurfaceState.h"
#include <memory>
#include <vector>
#include <SimTKcommon/SmallMatrix.h>

namespace OpenSim 
{

//=============================================================================
// IMPLICIT SURFACE REQUIREMENTS
//=============================================================================
// The minimal requirements for a specific implicit wrap surface
// characterization: i.e. Cylinder, Sphere, etc.
class ImplicitSurfaceParamsImpl
{
public:
    using Hessian = SimTK::SymMat33;

    ImplicitSurfaceParamsImpl() = default;
    ~ImplicitSurfaceParamsImpl() = default;

    // Constraint that defines the implicit surface.
    virtual double calcSurfaceConstraint(const SimTK::Vec3& p) const = 0;

    // Gradient of implicit surface constraint to position.
    virtual SimTK::Vec3 calcSurfaceConstraintGradient(const SimTK::Vec3& p) const = 0;

    // Hessian of implicit surface constraint to position.
    virtual const Hessian& calcSurfaceConstraintHessian(
        const SimTK::Vec3& p) const = 0;

    size_t integrationSteps; // TODO integration steps here?

protected:
    // Many surfaces have a constant hessian.
    Hessian _hessian;
};

//=============================================================================
// CONCRETE IMPLICIT SURFACE PARAMETERS
//=============================================================================
/// Implicit Surface parameters.
/// Computes the geodesic given the start state, but does not manage the state
/// data.
class ImplicitSurfaceParams
{
    using State = ImplicitSurfaceState;
    using Impl  = ImplicitSurfaceParamsImpl;

public:
    explicit ImplicitSurfaceParams(std::unique_ptr<ImplicitSurfaceParamsImpl>&& s) :
        _impl(std::move(s))
    {}

    //==========================================================================
    //                  SURFACE CONSTRAINTS
    //==========================================================================

    double calcSurfaceConstraint(const State& q) const;

    SimTK::Vec3 calcSurfaceConstraintGradient(const State& q) const;

    const ImplicitSurfaceParamsImpl::Hessian& calcSurfaceConstraintHessian(
        const State& q) const;

    //==========================================================================
    //                  LOCAL IMPLICIT GEODESIC STATE CALCULATIONS
    //==========================================================================

    std::pair<ImplicitGeodesicState, ImplicitGeodesicState> calcLocalGeodesic(
        State qStart,
        double length,
        std::vector<SimTK::Vec3>& log) const;

    // Initial guess for the implicit state.
    State calcInitState(const SimTK::Vec3& pStart, const SimTK::Vec3& pEnd) const;

    // Initial guess for the length.
    double calcInitLength(const SimTK::Vec3& pStart, const SimTK::Vec3& pEnd) const;

    void applyVariation(const GeodesicVariation& var, State& q) const;

private:
    std::unique_ptr<Impl> _impl;
};

}

#endif
