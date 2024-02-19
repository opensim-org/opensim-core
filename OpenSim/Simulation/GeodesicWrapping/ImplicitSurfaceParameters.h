#ifndef OPENSIM_IMPLICIT_SURFACE_PARAMETERS_H
#define OPENSIM_IMPLICIT_SURFACE_PARAMETERS_H

namespace OpenSim {

class ImplicitSurfaceParametersImpl {
public:

};

class ImplicitSurfaceParameters {
    using Impl = ImplicitSurfaceParametersImpl;

public:
    explicit ImplicitSurfaceParameters(
            std::unique_ptr<ImplicitSurfaceParametersImpl>&& s) :
            _impl(std::move(s)) {}

private:
    std::unique_ptr<Impl> _impl;
};

} // namespace OpenSim


#endif // OPENSIM_IMPLICIT_SURFACE_PARAMETERS_H
