#ifndef OPENSIM_GEODESIC_PATH_H_
#define OPENSIM_GEODESIC_PATH_H_

// INCLUDE

namespace OpenSim {

class Model;
class WrapObject;
class GeometryPath;

struct GeodesicCurveResult
{

};


// Similar role as the WrapObject ?
class OSIMSIMULATION_API GeodesicWrapObstacle {
public:

//=============================================================================
// METHODS
//=============================================================================
    //--------------------------------------------------------------------------
    // CONSTRUCTION
    //--------------------------------------------------------------------------
public:

#ifndef SWIG
    void setStartPoint( const SimTK::State& s, int aIndex);
    void setEndPoint( const SimTK::State& s, int aIndex);
#endif
    int getStartPoint() const { return get_range(0); }
    int getEndPoint() const { return get_range(1); }
    const std::string& getWrapObjectName() const { return get_wrap_object(); }
    const WrapObject* getWrapObject() const { return _wrapObject; }
    void setWrapObject(WrapObject& aWrapObject);

    const PathWrapPoint& getWrapPoint1() const {
        return getMemberSubcomponent<PathWrapPoint>(_wrapPoint1Ix);
    }
    const PathWrapPoint& getWrapPoint2() const {
        return getMemberSubcomponent<PathWrapPoint>(_wrapPoint2Ix);
    }
    PathWrapPoint& updWrapPoint1() { 
        return updMemberSubcomponent<PathWrapPoint>(_wrapPoint1Ix);
    }
    PathWrapPoint& updWrapPoint2() {
        return updMemberSubcomponent<PathWrapPoint>(_wrapPoint2Ix);
    }

    WrapMethod getMethod() const { return _method; }
    void setMethod(WrapMethod aMethod);
    const std::string& getMethodName() const { return get_method(); }

    const WrapResult& getPreviousWrap() const { return _previousWrap; }
    void setPreviousWrap(const WrapResult& aWrapResult);
    void resetPreviousWrap();

private:
    void constructProperties();
    void extendConnectToModel(Model& model) override;
    void setNull();

private:
    WrapMethod _method;

    const WrapObject* _wrapObject;
    const GeometryPath* _path;

    WrapResult _previousWrap;  // results from previous wrapping

    MemberSubcomponentIndex _wrapPoint1Ix{
        constructSubcomponent<PathWrapPoint>("pwpt1") };
    MemberSubcomponentIndex _wrapPoint2Ix{
        constructSubcomponent<PathWrapPoint>("pwpt2") };
//=============================================================================
};  // END of class PathWrap
//=============================================================================
//=============================================================================

/** @endcond **/

} // end of namespace OpenSim

#endif // OPENSIM_PATH_WRAP_H_


