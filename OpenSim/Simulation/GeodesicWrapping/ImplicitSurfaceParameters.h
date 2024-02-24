#include <memory>

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

    // Provide copy constructor
    ImplicitSurfaceParameters(const ImplicitSurfaceParameters& other) :
                              _impl(new Impl(*other._impl)) {}

    // Provide copy assignment operator
    ImplicitSurfaceParameters& operator=(const ImplicitSurfaceParameters& other) {
        if (this != &other) {
            _impl = std::make_unique<Impl>(*other._impl);
        }
        return *this;
    }

private:
    std::unique_ptr<Impl> _impl;
};

} // namespace OpenSim


#endif // OPENSIM_IMPLICIT_SURFACE_PARAMETERS_H
