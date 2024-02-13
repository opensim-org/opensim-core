#ifndef GEODESIC_WRAP_OBJECT_H_
#define GEODESIC_WRAP_OBJECT_H_

#include "GeodesicCurveState.h"
#include "ImplicitSurfaceParams.h"
#include <memory>
#include <vector>

namespace OpenSim
{

    // TODO rename WrapObject to Surface?

//==============================================================================
// GEODESIC WRAP OBJECT INTERFACE
//==============================================================================
/// This is where implicit and parametric surfaces come together.
// Also provides the separation between local and global coordinates.
class GeodesicWrapObjectImpl
{
public:
    virtual void calcInitState(
        const SimTK::Vec3& pStart,
        const SimTK::Vec3& pEnd) = 0;

    // Shoots the geodesic over the surface.
    virtual void calcLocalGeodesic(
        GeodesicCurveState& start,
        GeodesicCurveState& end,
        double& length,
        std::vector<SimTK::Vec3>& pointsLog) = 0;

    virtual void applyVariation(
        const GeodesicVariation& var) = 0;
};

//==============================================================================
// CONCRETE GEODESIC WRAP OBJECT
//==============================================================================
// Handles both implicit, parametric and analytic.
// Applies local-surface-coords to global-coords
// Does the shooting: calcGeodesic.
class GeodesicWrapObject
{
    public:
    // Construct Implicit GeodesicWrapObject
    GeodesicWrapObject(
            ImplicitSurfaceParams&& surface,
            std::shared_ptr<SimTK::Transform> transformHandle
            );

    // Construct Parametric GeodesicWrapObject
    /* GeodesicWrapObject( */
    /*         ParametricSurfaceParams&& surface, */ 
    /*         std::shared_ptr<SimTK::Transform> transformHandle */
    /*         ); */


    private:
    GeodesicWrapObject(
            std::unique_ptr<GeodesicWrapObjectImpl>&& impl,
            std::shared_ptr<SimTK::Transform> transform
            );

    public:

    // Shoots the geodesic.
    // Applies conversion from local to global coordinates.
    void calcGeodesic(GeodesicCurve& shooterResult);

    void applyVariation(
        const GeodesicVariation& var);

    // Triggers fetching the global coordinates from the underlying surface body.
    const SimTK::Transform& updTransformToGround();

private:
    std::unique_ptr<GeodesicWrapObjectImpl> _impl;
    std::shared_ptr<SimTK::Transform> _transform; // TODO a socket or something?
};

}

#endif
