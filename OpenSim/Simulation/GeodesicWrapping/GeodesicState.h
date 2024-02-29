#ifndef OPENSIM_GEODESIC_STATE_H
#define OPENSIM_GEODESIC_STATE_H

#include <OpenSim/Simulation/osimSimulationDLL.h>
#include <OpenSim/Simulation/Model/Model.h>

namespace OpenSim {

using SimTK::Vec3;

struct GeodesicVariation {
    // ds in thesis.
    double dTangential = NAN;
    double dTheta      = NAN;
    double dBeta       = NAN;
    double dLength     = NAN;
};

struct GeodesicBoundaryFrameVariation {
    GeodesicBoundaryFrameVariation() = default;

    SimTK::Vec3 tangent;
    SimTK::Vec3 normal;
    SimTK::Vec3 binormal;
};

struct GeodesicBoundaryFrame {
    GeodesicBoundaryFrame() = default;
    //GeodesicBoundaryFrame(SimTK::Vec3 velocity, SimTK::Vec3 surfaceNormal);

    // TODO use rotation vector?
    SimTK::Vec3 tangent;
    SimTK::Vec3 normal;
    SimTK::Vec3 binormal;
};

struct JacobiFieldScalar
{
    JacobiFieldScalar() = default;
    using Dot = SimTK::Vec2;
    JacobiFieldScalar(double a, double aDot)
            : value(a), derivative(aDot) {}

    //Dot calcDerivative(double gaussianCurvature) const;

    //JacobiFieldScalar &operator+=(const Dot &derivative);

    double value;
    double derivative;
};

/// State of the geodesic at a point along the curve.
struct GeodesicState
{
    GeodesicState() = default;
    //GeodesicBoundaryFrameVariation calcFrameToDBeta() const;

    GeodesicBoundaryFrame frame;
    SimTK::Vec3 position;
    double normalCurvature;
    double geodesicTorsion;
    JacobiFieldScalar a;
    JacobiFieldScalar r;
};

class Geodesic : public Component {
    OpenSim_DECLARE_CONCRETE_OBJECT(Geodesic, Component);
public:
    Geodesic() = default;
    std::string getValueAsString() const {
        return "TODO";
    }

private:
    GeodesicState startState;
    GeodesicState endState;
    SimTK::Real length;
    SimTK::Vector_<SimTK::Vec3> pointsLog;
};

struct GeodesicWrapResult {
    SimTK::Real length;
    SimTK::Real lengtheningSpeed;
    SimTK::Vector_<SimTK::Vec3> points;
    GeodesicBoundaryFrame startFrame;
    GeodesicBoundaryFrame endFrame;
};




class GeodesicInitialConditions : public Component {
    OpenSim_DECLARE_CONCRETE_OBJECT(GeodesicInitialConditions, Component);
public:
    GeodesicInitialConditions() = default;
    GeodesicInitialConditions(const SimTK::Vec3& position,
                              const SimTK::Vec3& velocity,
                              SimTK::Real length) {
        constructProperties();
        set_position(position);
        set_velocity(velocity);
        set_length(length);
    }

    const SimTK::Vec3& getPosition() const {
        return get_position();
    }
    const SimTK::Vec3& getVelocity() const {
        return get_velocity();
    }
    SimTK::Real getLength() const {
        return get_length();
    }

private:
    OpenSim_DECLARE_PROPERTY(position, SimTK::Vec3,
            "The initial position of the geodesic on the surface.");
    OpenSim_DECLARE_PROPERTY(velocity, SimTK::Vec3,
            "The initial velocity of the geodesic on the surface.");
    OpenSim_DECLARE_PROPERTY(length, SimTK::Real,
            "The initial length of the geodesic on the surface.");

    void constructProperties() {
        constructProperty_position(SimTK::Vec3(0));
        constructProperty_velocity(SimTK::Vec3(0));
        constructProperty_length(0);
    }
};



// An abstract component class that you can calculate geodesics over.
class GeodesicWrapObject
{
public:
    virtual ~GeodesicWrapObject() = default;

protected:
    GeodesicWrapObject()                                         = default;
    GeodesicWrapObject(GeodesicWrapObject&&) noexcept            = default;
    GeodesicWrapObject& operator=(GeodesicWrapObject&&) noexcept = default;
    GeodesicWrapObject(const GeodesicWrapObject&)                = default;
    GeodesicWrapObject& operator=(const GeodesicWrapObject&)     = default;

public:

    void setInitialConditions(
            const Vec3& initPosition, const Vec3& initVelocity, double length) {
        _geodesicInitialConditions = {initPosition, initVelocity, length};
    }

    void setInitialConditions(
            const GeodesicInitialConditions& initialConditions) {
        _geodesicInitialConditions = initialConditions;
    }

//    Geodesic calcGeodesic() const;
//    Geodesic calcWrappingPath(Vec3 pointBefore, Vec3 pointAfter) const;

    void setTransform(const SimTK::Transform& transform) {
        _transform = transform;
    }

private:
    virtual Geodesic calcLocalGeodesicImpl(
            Vec3 initPosition,
            Vec3 initVelocity,
            double length) const = 0;

    GeodesicInitialConditions _geodesicInitialConditions;
    SimTK::Transform _transform;
};

class ImplicitWrapObject : public virtual GeodesicWrapObject
{
public:
    // TODO Use symmetric matrix class.
    using Hessian = SimTK::SymMat33;

    virtual ~ImplicitWrapObject() = default;

protected:
    ImplicitWrapObject()                                         = default;
    ImplicitWrapObject(const ImplicitWrapObject&)                = default;
    ImplicitWrapObject(ImplicitWrapObject&&) noexcept            = default;
    ImplicitWrapObject& operator=(const ImplicitWrapObject&)     = default;
    ImplicitWrapObject& operator=(ImplicitWrapObject&&) noexcept = default;

public:
    // TODO put local in front of everything?

//    double calcSurfaceConstraint(Vec3 position) const;
//    Vec3 calcSurfaceConstraintGradient(Vec3 position) const;
//    Hessian calcSurfaceConstraintHessian(Vec3 position) const;

private:
    // Implicit surface constraint.
//    virtual double calcSurfaceConstraintImpl(Vec3 position) const         = 0;
//    virtual Vec3 calcSurfaceConstraintGradientImpl(Vec3 position) const   = 0;
//    virtual Hessian calcSurfaceConstraintHessianImpl(Vec3 position) const = 0;

    Geodesic calcLocalGeodesicImpl(
            Vec3 initPosition,
            Vec3 initVelocity,
            double length) const override {
        // TODO implement.
        return {};
    }

    // TODO would become obsolete with variable step integration.
    size_t _integratorSteps = 100;
};


// Concrete component.
class ImplicitCylinderWrapObject : public ImplicitWrapObject
{
public:
    explicit ImplicitCylinderWrapObject(double radius) : _radius(radius) {}

private:
    // Implicit surface constraint.
//    double calcSurfaceConstraintImpl(Vec3 position) const override;
//    Vec3 calcSurfaceConstraintGradientImpl(Vec3 position) const override;
//    Hessian calcSurfaceConstraintHessianImpl(Vec3 position) const override;

    double _radius;
};





}

#endif // OPENSIM_GEODESIC_STATE_H
