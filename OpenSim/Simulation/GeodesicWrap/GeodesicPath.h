#ifndef OPENSIM_GEODESIC_PATH_H_
#define OPENSIM_GEODESIC_PATH_H_

// INCLUDE

#include "OpenSim/Simulation/GeodesicWrap/GeodesicCurveState.h"
#include "GeodesicWrapObject.h"
#include "GeodesicPathSolver.h"

namespace OpenSim {

// Notes:
// - rename Object to Surface?
// - GeodesicWrapObject holds pointer to frame, how?
// - ImplicitSurface holds the local state, how can it cache it?
// - what is the workflow?
//     - create surface component, add to model, connect to frame.
//     - create wrapping path, add to model, connect each surface.
//     - what are those connections? Sockets? Parameters? shared-pointers?

class AbstractGeometryPath;

// Concrete implementation of the AbstractGeometryPath.
//
// Contains the solver for computing the entire wrapping path.
//
// Comparable to GeometryPath?
//
// Does the following:
// - Caches the result from the solver.
// - Holds set of WrapOstacles
// - Implements AbstractGeometryPath
class OSIMSIMULATION_API GeodesicPath : public AbstractGeometryPath {
OpenSim_DECLARE_CONCRETE_OBJECT(GeodesicPath, ModelComponent);
public:
    //==============================================================================
    // PROPERTIES
    //==============================================================================
    OpenSim_DECLARE_PROPERTY(some_property, std::string,
        "some property");

    //--------------------------------------------------------------------------
    // CONSTRUCTION
    //--------------------------------------------------------------------------
public:
    GeodesicPath() = default;
    ~GeodesicPath() = default;

//=============================================================================
// METHODS
//=============================================================================

    //--------------------------------------------------------------------------
    // ABSTRACTGEOMETRYPATH IMPLEMENTATION
    //--------------------------------------------------------------------------
    double getLength(const SimTK::State& s) const override;
    double getLengtheningSpeed(const SimTK::State& s) const override;
    void addInEquivalentForces(const SimTK::State& state,
    const double& tension,
    SimTK::Vector_<SimTK::SpatialVec>& bodyForces,
    SimTK::Vector& mobilityForces) const override;
    double computeMomentArm(const SimTK::State& s,
    const Coordinate& aCoord) const override;
    bool isVisualPath() const override;

private:

    // Path start and end points.
    SimTK::Vec3 _pStart;
    SimTK::Vec3 _pEnd;

    // List of wrap objects.
    std::vector<GeodesicWrapObject> _wrapObstacles;

    // Buffer for holding the shooter results.
    std::vector<GeodesicCurve> _surfaceShooterResults;

    GeodesicPathSolver _solver;
};

} // end of namespace OpenSim

#endif // OPENSIM_PATH_WRAP_H_


