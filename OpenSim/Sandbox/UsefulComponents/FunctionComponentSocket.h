#ifndef FUNCTION_COMPONENT_SOCKET_H
#define FUNCTION_COMPONENT_SOCKET_H

#include <OpenSim/Simulation/Model/ModelComponent.h>

namespace OpenSim
{
    /**
    * Implements a socket between a non component function and a Component.
    *
    * @author Dimitar Stanev
    */
    template<class T>
    class FunctionComponentSocket : public ModelComponent
    {
        OpenSim_DECLARE_CONCRETE_OBJECT(FunctionComponentSocket, ModelComponent);

    public:

        OpenSim_DECLARE_OUTPUT(output, T, getValue, SimTK::Stage::Time);

        FunctionComponentSocket(std::function<T(const SimTK::State& s)> f)
            : ModelComponent(), function(f)
        {
        }

    private:

        T getValue(const SimTK::State& s) const
        {
            return function(s);
        }

        std::function<T(const SimTK::State& s)> function;
    };
}; // end of namespace OpenSim

#endif
