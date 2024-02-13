#ifndef OPENSIM_GEODESIC_PATH_H_
#define OPENSIM_GEODESIC_PATH_H_

// INCLUDE

namespace OpenSim {

class Model;
class AbstractGeometryPath;
class GeodesicPathSolver;

// Concrete implementation of the AbstractGeometryPath.
//
// Contains the solver for computing the entire wrapping path.
//
// Comparable to GeometryPath?
//
// Does the following:
// - Caches the result from the solver
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

//=============================================================================
// METHODS
//=============================================================================
    //--------------------------------------------------------------------------
    // CONSTRUCTION
    //--------------------------------------------------------------------------
public:
    GeodesicPath() = default;
    ~GeodesicPath() = default;

private:

    GeodesicPathSolver _solver;
};

class GeodesicPathSolver
{

};

} // end of namespace OpenSim

#endif // OPENSIM_PATH_WRAP_H_


