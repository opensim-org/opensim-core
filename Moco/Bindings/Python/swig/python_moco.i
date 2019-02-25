%module(package="opensim", directors="1") moco
// %module(package="opensim", directors="1", threads="1") moco
#pragma SWIG nowarn=822,451,503,516,325
#pragma SWIG nowarn=401

%{
#define SWIG_FILE_WITH_INIT
#include <Bindings/OpenSimHeaders_common.h>
#include <Bindings/OpenSimHeaders_simulation.h>
#include <Bindings/OpenSimHeaders_actuators.h>
#include <Bindings/OpenSimHeaders_analyses.h>
#include <Bindings/OpenSimHeaders_tools.h>
#include <Bindings/OpenSimHeaders_moco.h>
%}

%{
using namespace OpenSim;
using namespace SimTK;
%}

%include "python_preliminaries.i"

// Tell SWIG about the modules we depend on.
// TODO import "python_examplecomponents.i"
%import "python_tools.i"

// Relay exceptions to the target language.
// This causes substantial code bloat and possibly hurts performance.
// Without these try-catch block, a SimTK or OpenSim exception causes the
// program to crash.
%include "exception.i"
// Delete any previous exception handlers.
%exception; 
%exception {
    try {
        $action
    } catch (const std::exception& e) {
        std::string str("std::exception in '$fulldecl': ");
        std::string what(e.what());
        SWIG_exception_fail(SWIG_RuntimeError, (str + what).c_str());
    }
}

// Typemaps
// ========
// None.

%extend OpenSim::MocoPhase {
%pythoncode %{
    def _convert(self, cls, v):
        if hasattr(v, '__len__'):
            if len(v) > 2:
                raise Exception("Bounds cannot have more than 2 elements.")
            elif len(v) == 1:
                return cls(v[0])
            elif len(v) == 2:
                return cls(v[0], v[1])
            else:
                return cls()
        else:
            return cls(v)
%}
}
%pythonprepend OpenSim::MocoPhase::setTimeBounds %{
    if not type(arg2) is MocoInitialBounds:
        arg2 = self._convert(MocoInitialBounds, arg2)
    if not type(arg3) is MocoFinalBounds:
        arg3 = self._convert(MocoFinalBounds, arg3)
%}
%pythonprepend OpenSim::MocoPhase::setStateInfo %{
    args = list(args)
    if len(args) >= 2 and not type(args[1]) is MocoBounds:
        args[1] = self._convert(MocoBounds, args[1])
    if len(args) >= 3 and not type(args[2]) is MocoInitialBounds:
        args[2] = self._convert(MocoInitialBounds, args[2])
    if len(args) >= 4 and not type(args[3]) is MocoFinalBounds:
        args[3] = self._convert(MocoFinalBounds, args[3])
%}
%pythonprepend OpenSim::MocoPhase::setControlInfo %{
    args = list(args)
    if len(args) >= 2 and not type(args[1]) is MocoBounds:
        args[1] = self._convert(MocoBounds, args[1])
    if len(args) >= 3 and not type(args[2]) is MocoInitialBounds:
        args[2] = self._convert(MocoInitialBounds, args[2])
    if len(args) >= 4 and not type(args[3]) is MocoFinalBounds:
        args[3] = self._convert(MocoFinalBounds, args[3])
%}

%extend OpenSim::MocoProblem {
%pythoncode %{
    def _convert(self, cls, v):
        if hasattr(v, '__len__'):
            if len(v) > 2:
                raise Exception("Bounds cannot have more than 2 elements.")
            elif len(v) == 1:
                return cls(v[0])
            elif len(v) == 2:
                return cls(v[0], v[1])
            else:
                return cls()
        else:
            return cls(v)
%}
}
%pythonprepend OpenSim::MocoProblem::setTimeBounds %{
    if not type(arg2) is MocoInitialBounds:
        arg2 = self._convert(MocoInitialBounds, arg2)
    if not type(arg3) is MocoFinalBounds:
        arg3 = self._convert(MocoFinalBounds, arg3)
%}
%pythonprepend OpenSim::MocoProblem::setStateInfo %{
    args = list(args)
    if len(args) >= 2 and not type(args[1]) is MocoBounds:
        args[1] = self._convert(MocoBounds, args[1])
    if len(args) >= 3 and not type(args[2]) is MocoInitialBounds:
        args[2] = self._convert(MocoInitialBounds, args[2])
    if len(args) >= 4 and not type(args[3]) is MocoFinalBounds:
        args[3] = self._convert(MocoFinalBounds, args[3])
%}
%pythonprepend OpenSim::MocoProblem::setControlInfo %{
    args = list(args)
    if len(args) >= 2 and not type(args[1]) is MocoBounds:
        args[1] = self._convert(MocoBounds, args[1])
    if len(args) >= 3 and not type(args[2]) is MocoInitialBounds:
        args[2] = self._convert(MocoInitialBounds, args[2])
    if len(args) >= 4 and not type(args[3]) is MocoFinalBounds:
        args[3] = self._convert(MocoFinalBounds, args[3])
%}


%extend OpenSim::MocoIterate {
    void setTime(const std::vector<double>& time) {
        $self->setTime(SimTK::Vector((int)time.size(), time.data()));
    }
    void setState(const std::string& name, const std::vector<double>& traj) {
        $self->setState(name, SimTK::Vector((int)traj.size(), traj.data()));
    }
    void setControl(const std::string& name, const std::vector<double>& traj) {
        $self->setControl(name, SimTK::Vector((int)traj.size(), traj.data()));
    }
    void setMultiplier(const std::string& name, const std::vector<double>& traj)
    {
        $self->setMultiplier(name,
                SimTK::Vector((int)traj.size(), traj.data()));
    }
}

// Memory management
// =================

%pythonappend OpenSim::MocoPhase::setModel %{
    model._markAdopted()
%}
%pythonappend OpenSim::MocoPhase::addParameter %{
    ptr._markAdopted()
%}
%pythonappend OpenSim::MocoPhase::addCost %{
    ptr._markAdopted()
%}
%pythonappend OpenSim::MocoPhase::addPathConstraint %{
    ptr._markAdopted()
%}
%pythonappend OpenSim::MocoProblem::setModel %{
    model._markAdopted()
%}
%pythonappend OpenSim::MocoProblem::addParameter %{
    ptr._markAdopted()
%}
%pythonappend OpenSim::MocoProblem::addCost %{
    ptr._markAdopted()
%}
%pythonappend OpenSim::MocoProblem::addPathConstraint %{
    ptr._markAdopted()
%}

%pythonprepend OpenSim::MocoTool::solve %{
    if MocoCasADiSolver.safeDownCast(self.updSolver()):
        solver = MocoCasADiSolver.safeDownCast(self.updSolver())
        solver.setRunningInPython(True)
%}


// Include all the OpenSim code.
// =============================
%include <Bindings/preliminaries.i>
%include <Bindings/moco.i>

// %thread OpenSim::MocoTool::solve;
