#ifndef OPENSIM_IMPLICIT_CYLINDER_H
#define OPENSIM_IMPLICIT_CYLINDER_H

#include "ImplicitSurfaceParameters.h"

namespace OpenSim {

class ImplicitCylinder final : public ImplicitSurfaceParametersImpl {
public:
    explicit ImplicitCylinder(SimTK::Real radius) {
        m_radius = radius;
    }

private:
    SimTK::Real m_radius;
};

}

#endif //OPENSIM_IMPLICIT_CYLINDER_H
