#ifndef IMPLICIT_SURFACE_OBJECT_H_
#define IMPLICIT_SURFACE_OBJECT_H_

#include "ImplicitSurfaceState.h"
#include "ImplicitSurfaceParams.h"
#include "GeodesicWrapObject.h"
#include <memory>

namespace OpenSim
{

//==============================================================================
// CONCRETE IMPLICIT WRAP SURFACE
//==============================================================================
// Lives in local surface coords.
// Provides abstraction over all implicit surfaces.
// Maintains the implicit-curve-state.
// Aplies natural variations.
class ImplicitSurfaceObject: public GeodesicWrapObjectImpl
{
    public:
    explicit ImplicitSurfaceObject(
        ImplicitSurfaceParams&& params);

    explicit ImplicitSurfaceObject(
        std::unique_ptr<ImplicitSurfaceParamsImpl>&& params);

    //--------------------------------------------------------------------------
    // GEODESIC WRAP OBJECT IMPLEMENTATION
    //--------------------------------------------------------------------------

    void calcInitState(
        const SimTK::Vec3& pStart,
        const SimTK::Vec3& pEnd) override;

    void calcLocalGeodesic(
        GeodesicCurveState& start,
        GeodesicCurveState& end,
        double& length,
        std::vector<SimTK::Vec3>& pointsLog) override;

    void applyVariation(
        const GeodesicVariation& var) override;

    //--------------------------------------------------------------------------
    // DATA
    //--------------------------------------------------------------------------

private:
    ImplicitSurfaceParams _surface;
    ImplicitSurfaceState _startState;
    // TODO compute initial length.
    double _length = NAN;
};

}

#endif
