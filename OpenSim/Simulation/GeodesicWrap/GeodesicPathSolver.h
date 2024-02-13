#ifndef OPENSIM_GEODESIC_PATH_SOLVER_H_
#define OPENSIM_GEODESIC_PATH_SOLVER_H_

// INCLUDE

#include "OpenSim/Simulation/GeodesicWrap/GeodesicCurveState.h"
#include "GeodesicWrapObject.h"

namespace OpenSim {

class GeodesicPathSolver
{

    public:

    GeodesicPathSolver() = default;

    struct GeodesicWrappingPath {
        // Useful fields...
        double length = NAN;
        double lengtheningSpeed = NAN;
        // Log of points
    };

    // 1. Call shooter on all surface objects
    // 2. Compute wrapping-error (stop here if done)
    // 2. Compute GeodesicVariation from the surface geodesics
    // 3. Apply variation to local surface state.
    // 4. Repeat until error converges.
    void calcWrappingPath(
            const SimTK::Vec3& pStart,
            const SimTK::Vec3& pEnd,
            std::vector<GeodesicWrapObject>& objects, GeodesicWrappingPath& result, size_t maxIter, double eps = 1e-13) const;

    GeodesicVariation calcGeodesicCorrection();

    private:
    // Data structs, matrices etc for solving newton algorithm.
};

} // end of namespace OpenSim

#endif // OPENSIM_PATH_WRAP_H_


