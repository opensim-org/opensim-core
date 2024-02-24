#ifndef OPENSIM_GEODESIC_STATE_H
#define OPENSIM_GEODESIC_STATE_H

#include "ImplicitSurfaceParameters.h"

#include <OpenSim/Simulation/osimSimulationDLL.h>

namespace OpenSim {

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
    GeodesicBoundaryFrame(SimTK::Vec3 velocity, SimTK::Vec3 surfaceNormal);

    // TODO use rotation vector?
    SimTK::Vec3 tangent;
    SimTK::Vec3 normal;
    SimTK::Vec3 binormal;
};

struct JacobiFieldScalar
{
    using Dot = SimTK::Vec2;
    JacobiFieldScalar(double a, double aDot)
            : value(a), derivative(aDot) {}

    Dot calcDerivative(double gaussianCurvature) const;

    JacobiFieldScalar &operator+=(const Dot &derivative);

    double value;
    double derivative;
};

/// State of the geodesic at a point along the curve.
struct GeodesicState
{
    GeodesicBoundaryFrameVariation calcFrameToDBeta() const;

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

class GeodesicWrapObject {
public:
    GeodesicWrapObject(ImplicitSurfaceParameters&& parameters,
                       SimTK::MobilizedBodyIndex mobodIndex) :
        _implicitSurfaceParameters(std::move(parameters)),
        _mobodIndex(mobodIndex) {}
    ~GeodesicWrapObject() = default;

    GeodesicWrapObject(const GeodesicWrapObject& other) = default;
    GeodesicWrapObject& operator=(const GeodesicWrapObject& other) {
        if (this != &other) {
            _implicitSurfaceParameters = other._implicitSurfaceParameters;
            _mobodIndex = other._mobodIndex;
        }
        return *this;
    }

    GeodesicWrapObject(GeodesicWrapObject&&) = default;
    GeodesicWrapObject& operator=(GeodesicWrapObject&&) = default;

private:
    ImplicitSurfaceParameters _implicitSurfaceParameters;
    SimTK::MobilizedBodyIndex _mobodIndex;
};


class GeodesicInitialConditions : public Component {
    OpenSim_DECLARE_CONCRETE_OBJECT(GeodesicInitialConditions, Component);
public:
    GeodesicInitialConditions() = default;
    GeodesicInitialConditions(const SimTK::Vec3& position,
                              const SimTK::Vec3& velocity,
                              SimTK::Real length) {
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
};

/**
 * A helper class to allow storing a collection of geodesics belonging to a
 * GeodesicPathSegment in a discrete variable.
 */
class Geodesics : public SimTK::AbstractValue {

public:
    Geodesics() = default;

    void set(const std::vector<Geodesic>& geodesics) {
        _geodesics = geodesics;
    }

    const std::vector<Geodesic>& get() const {
        return _geodesics;
    }

    std::vector<Geodesic>& upd() {
        return _geodesics;
    }

    Geodesics* clone() const override {
        return new Geodesics(*this);
    }

    SimTK::String getTypeName() const override {
        return "Geodesics";
    }

    SimTK::String getValueAsString() const override {
        std::stringstream ss;
        for (const auto& geodesic : _geodesics) {
            ss << geodesic.getValueAsString() << std::endl;
        }
        return ss.str();
    }

    static bool isA(const AbstractValue& value) {
        return dynamic_cast<const Geodesics*>(&value) != nullptr;
    }

    bool isCompatible(const AbstractValue& value) const override {
        return isA(value);
    }

    void compatibleAssign(const AbstractValue& value) override {
        if (!isA(value)) {
            OPENSIM_THROW(Exception,
                    "Geodesics::compatibleAssign(): incompatible value.");
        }

        *this = dynamic_cast<const Geodesics&>(value);
    }

private:
    std::vector<Geodesic> _geodesics;

    // Disable public usage of the getValue() and setValue() methods, which are
    // not relevant to this class (they depend on Value<T>).
    using SimTK::AbstractValue::getValue;
    using SimTK::AbstractValue::updValue;
};

}

#endif // OPENSIM_GEODESIC_STATE_H
