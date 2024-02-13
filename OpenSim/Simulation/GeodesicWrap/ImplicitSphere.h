#ifndef IMPLICIT_SPHERE_H_
#define IMPLICIT_SPHERE_H_

#include "ImplicitSurfaceParams.h"

namespace OpenSim
{

//=============================================================================
// CONCRETE IMPLICIT SPHERICAL SURFACE PARAMETERS
//=============================================================================
// The minimal requirements for a specific implicit wrap surface
// characterization: i.e. Cylinder, Sphere, etc.
class ImplicitSphere final: public ImplicitSurfaceParamsImpl
{
public:
    explicit ImplicitSphere(double radius);

    ~ImplicitSphere() = default;

    double calcSurfaceConstraint(const SimTK::Vec3& position) const override;

    SimTK::Vec3 calcSurfaceConstraintGradient(const SimTK::Vec3& position) const override;

    const Hessian& calcSurfaceConstraintHessian(const SimTK::Vec3& position) const override;

private:
    double _radius = NAN;
};

}

#endif
