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

// Add support for converting between NumPy and C arrays (for MocoTrajectory).
%include "numpy.i"
%init %{
    import_array();
%}


%include <Bindings/preliminaries.i>
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
            elif len(v) == 0:
                return cls()
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
%pythonprepend OpenSim::MocoPhase::setStateInfoPattern %{
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
%pythonprepend OpenSim::MocoPhase::setControlInfoPattern %{
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
            elif len(v) == 0:
                return cls()
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
%pythonprepend OpenSim::MocoProblem::setStateInfoPattern %{
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
%pythonprepend OpenSim::MocoProblem::setControlInfoPattern %{
    args = list(args)
    if len(args) >= 2 and not type(args[1]) is MocoBounds:
        args[1] = self._convert(MocoBounds, args[1])
    if len(args) >= 3 and not type(args[2]) is MocoInitialBounds:
        args[2] = self._convert(MocoInitialBounds, args[2])
    if len(args) >= 4 and not type(args[3]) is MocoFinalBounds:
        args[3] = self._convert(MocoFinalBounds, args[3])
%}

%extend OpenSim::MocoControlTrackingGoal {
%pythoncode %{
    def _convert(self, cls, v):
        if hasattr(v, '__len__'):
            if len(v) > 2:
                raise Exception("Bounds cannot have more than 2 elements.")
            elif len(v) == 0:
                return cls()
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
%pythonprepend OpenSim::MocoControlTrackingGoal::addScaleFactor %{
    if not type(bounds) is MocoBounds:
        bounds = self._convert(MocoBounds, bounds)
%}

%extend OpenSim::MocoStateTrackingGoal {
%pythoncode %{
    def _convert(self, cls, v):
        if hasattr(v, '__len__'):
            if len(v) > 2:
                raise Exception("Bounds cannot have more than 2 elements.")
            elif len(v) == 0:
                return cls()
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
%pythonprepend OpenSim::MocoStateTrackingGoal::addScaleFactor %{
    if not type(bounds) is MocoBounds:
        bounds = self._convert(MocoBounds, bounds)
%}

%extend OpenSim::MocoMarkerTrackingGoal {
%pythoncode %{
    def _convert(self, cls, v):
        if hasattr(v, '__len__'):
            if len(v) > 2:
                raise Exception("Bounds cannot have more than 2 elements.")
            elif len(v) == 0:
                return cls()
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
%pythonprepend OpenSim::MocoMarkerTrackingGoal::addScaleFactor %{
    if not type(bounds) is MocoBounds:
        bounds = self._convert(MocoBounds, bounds)
%}

%extend OpenSim::MocoContactTrackingGoal {
%pythoncode %{
    def _convert(self, cls, v):
        if hasattr(v, '__len__'):
            if len(v) > 2:
                raise Exception("Bounds cannot have more than 2 elements.")
            elif len(v) == 0:
                return cls()
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
%pythonprepend OpenSim::MocoContactTrackingGoal::addScaleFactor %{
    if not type(bounds) is MocoBounds:
        bounds = self._convert(MocoBounds, bounds)
%}

// MocoTrajectory's functions contain lots of matrices, and Python users
// feel more comfortable providing/getting these matrices as NumPy types rather
// than as SimTK numeric types. We can use SWIG NumPy typemaps to accomplish
// this.
// https://docs.scipy.org/doc/numpy/reference/swig.interface-file.html
// https://scipy-cookbook.readthedocs.io/items/SWIG_NumPy_examples.html
// For constructor.
// The Python version of any Moco function taking the pair of arguments
// (int ntime, double* time) or (int nparams, double* params) will instead
// take a single NumPy array.
%apply (int DIM1, double* IN_ARRAY1) {
    (int ntime, double* time),
    (int nparams, double* params)
};
// For constructor.
// The Python version of any Moco function taking sets of arguments
// (int nrowstates, int ncolstates, double* states) will instead take a single
// NumPy 2D array.
%apply (int DIM1, int DIM2, double* IN_ARRAY2) {
    (int nrowstates, int ncolstates, double* states),
    (int nrowcontrols, int ncolcontrols, double* controls),
    (int nrowmults, int ncolmults, double* mults),
    (int nrowderivs, int ncolderivs, double* derivs)
};
// For getState(), etc.
// The typemaps for functions that return matrices is more complicated: we must
// pass in a NumPy array that already has the correct size. We create a (hidden)
// C++ function that takes, for example, (int n, double* timeOut)
// (e.g., _getTimeMat()).
// Then we create a Python function that invokes the hidden C++ function with
// a correctly-sized NumPy array (e.g., getTimeMat()).
%apply (int DIM1, double* ARGOUT_ARRAY1) {
    (int n, double* timeOut),
    (int n, double* stateOut),
    (int n, double* controlOut),
    (int n, double* multOut),
    (int n, double* derivOut),
    (int n, double* paramsOut)
};
// For getStatesTrajectory(), etc.
// Similar to above but for 2D arrays.
%apply (int DIM1, int DIM2, double* INPLACE_FARRAY2) {
    (int nrow, int ncol, double* statesOut),
    (int nrow, int ncol, double* controlsOut),
    (int nrow, int ncol, double* multsOut),
    (int nrow, int ncol, double* derivsOut)
};
%extend OpenSim::MocoTrajectory {
    MocoTrajectory(
            int ntime,
            double* time,
            std::vector<std::string> state_names,
            std::vector<std::string> control_names,
            std::vector<std::string> multiplier_names,
            std::vector<std::string> parameter_names,
            int nrowstates, int ncolstates, double* states,
            int nrowcontrols, int ncolcontrols, double* controls,
            int nrowmults, int ncolmults, double* mults,
            int nparams, double* params) {
        return new MocoTrajectory(SimTK::Vector(ntime, time, true),
                        std::move(state_names),
                        std::move(control_names),
                        std::move(multiplier_names),
                        std::move(parameter_names),
                        SimTK::Matrix(nrowstates, ncolstates, states),
                        SimTK::Matrix(nrowcontrols, ncolcontrols, controls),
                        SimTK::Matrix(nrowmults, ncolmults, mults),
                        SimTK::RowVector(nparams, params, true));
    }
    MocoTrajectory(
            int ntime,
            double* time,
            std::vector<std::string> state_names,
            std::vector<std::string> control_names,
            std::vector<std::string> multiplier_names,
            std::vector<std::string> derivative_names,
            std::vector<std::string> parameter_names,
            int nrowstates, int ncolstates, double* states,
            int nrowcontrols, int ncolcontrols, double* controls,
            int nrowmults, int ncolmults, double* mults,
            int nrowderivs, int ncolderivs, double* derivs,
            int nparams, double* params) {
        return new MocoTrajectory(SimTK::Vector(ntime, time, true),
                std::move(state_names),
                std::move(control_names),
                std::move(multiplier_names),
                std::move(derivative_names),
                std::move(parameter_names),
                SimTK::Matrix(nrowstates, ncolstates, states),
                SimTK::Matrix(nrowcontrols, ncolcontrols, controls),
                SimTK::Matrix(nrowmults, ncolmults, mults),
                SimTK::Matrix(nrowderivs, ncolderivs, derivs),
                SimTK::RowVector(nparams, params, true));
    }
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
    void setDerivative(const std::string& name, const std::vector<double>& traj)
    {
        $self->setDerivative(name,
                SimTK::Vector((int)traj.size(), traj.data()));
    }
    void _getTimeMat(int n, double* timeOut) const {
        OPENSIM_THROW_IF(n != $self->getNumTimes(), OpenSim::Exception,
                "n != getNumTimes()");
        const auto& time = $self->getTime();
        std::copy_n(time.getContiguousScalarData(), n, timeOut);
    }
    void _getStateMat(std::string name, int n, double* stateOut) const {
        OPENSIM_THROW_IF(n != $self->getNumTimes(), OpenSim::Exception,
                "n != getNumTimes()");
        Vector state = $self->getState(name);
        std::copy_n(state.getContiguousScalarData(), n, stateOut);
    }
    void _getControlMat(std::string name, int n, double* controlOut) const {
        OPENSIM_THROW_IF(n != $self->getNumTimes(), OpenSim::Exception,
                "n != getNumTimes()");
        Vector control = $self->getControl(name);
        std::copy_n(control.getContiguousScalarData(), n, controlOut);
    }
    void _getMultiplierMat(std::string name, int n, double* multOut) const {
        OPENSIM_THROW_IF(n != $self->getNumTimes(), OpenSim::Exception,
                "n != getNumTimes()");
        Vector mult = $self->getMultiplier(name);
        std::copy_n(mult.getContiguousScalarData(), n, multOut);
    }
    void _getDerivativeMat(std::string name, int n, double* derivOut) const {
        OPENSIM_THROW_IF(n != $self->getNumTimes(), OpenSim::Exception,
                "n != getNumTimes()");
        Vector deriv = $self->getDerivative(name);
        std::copy_n(deriv.getContiguousScalarData(), n, derivOut);
    }
    void _getParametersMat(int n, double* paramsOut) const {
        const RowVector& params = $self->getParameters();
        OPENSIM_THROW_IF(n != params.size(), OpenSim::Exception,
                "n != number of parameters");
        std::copy_n(params.getContiguousScalarData(), n, paramsOut);
    }
    void _getStatesTrajectoryMat(int nrow, int ncol, double* statesOut) const {
        OPENSIM_THROW_IF(nrow != $self->getNumTimes(), OpenSim::Exception,
                "nrow != getNumTimes()");
        const auto& states = $self->getStatesTrajectory();
        OPENSIM_THROW_IF(ncol != states.ncol(), OpenSim::Exception,
                "ncol != number of states");
        std::copy_n(states.getContiguousScalarData(), nrow * ncol, statesOut);
    }
    void _getControlsTrajectoryMat(int nrow, int ncol, double* controlsOut) const {
        OPENSIM_THROW_IF(nrow != $self->getNumTimes(), OpenSim::Exception,
                "nrow != getNumTimes()");
        const auto& controls = $self->getControlsTrajectory();
        OPENSIM_THROW_IF(ncol != controls.ncol(), OpenSim::Exception,
                "ncol != number of controls");
        std::copy_n(controls.getContiguousScalarData(), nrow * ncol, controlsOut);
    }
    void _getMultipliersTrajectoryMat(int nrow, int ncol, double* multsOut) const {
        OPENSIM_THROW_IF(nrow != $self->getNumTimes(), OpenSim::Exception,
                "nrow != getNumTimes()");
        const auto& mults = $self->getMultipliersTrajectory();
        OPENSIM_THROW_IF(ncol != mults.ncol(), OpenSim::Exception,
                "ncol != number of mults");
        std::copy_n(mults.getContiguousScalarData(), nrow * ncol, multsOut);
    }
    void _getDerivativesTrajectoryMat(int nrow, int ncol, double* derivsOut) const {
        OPENSIM_THROW_IF(nrow != $self->getNumTimes(), OpenSim::Exception,
                "nrow != getNumTimes()");
        const auto& derivs = $self->getDerivativesTrajectory();
        OPENSIM_THROW_IF(ncol != derivs.ncol(), OpenSim::Exception,
                "ncol != number of derivs");
        std::copy_n(derivs.getContiguousScalarData(), nrow * ncol, derivsOut);
    }
%pythoncode %{
    def getTimeMat(self):
        return self._getTimeMat(self.getNumTimes())
    def getStateMat(self, name):
        return self._getStateMat(name, self.getNumTimes())
    def getControlMat(self, name):
        return self._getControlMat(name, self.getNumTimes())
    def getMultiplierMat(self, name):
        return self._getMultiplierMat(name, self.getNumTimes())
    def getDerivativeMat(self, name):
        return self._getDerivativeMat(name, self.getNumTimes())
    def getParametersMat(self):
        return self._getParametersMat(len(self.getParameterNames()))

    def getStatesTrajectoryMat(self):
        import numpy as np
        mat = np.empty([self.getNumTimes(), len(self.getStateNames())])
        self._getStatesTrajectoryMat(mat)
        return mat
    def getControlsTrajectoryMat(self):
        import numpy as np
        mat = np.empty([self.getNumTimes(), len(self.getControlNames())])
        self._getControlsTrajectoryMat(mat)
        return mat
    def getMultipliersTrajectoryMat(self):
        import numpy as np
        mat = np.empty([self.getNumTimes(), len(self.getMultiplierNames())])
        self._getMultipliersTrajectoryMat(mat)
        return mat
    def getDerivativesTrajectoryMat(self):
        import numpy as np
        mat = np.empty([self.getNumTimes(), len(self.getDerivativeNames())])
        self._getDerivativesTrajectoryMat(mat)
        return mat

%};
}

// Memory management
// =================

%pythonappend OpenSim::MocoPhase::setModel %{
    model._markAdopted()
%}
%pythonappend OpenSim::MocoPhase::addParameter %{
    ptr._markAdopted()
%}
%pythonappend OpenSim::MocoPhase::addGoal %{
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
%pythonappend OpenSim::MocoProblem::addGoal %{
    ptr._markAdopted()
%}
%pythonappend OpenSim::MocoProblem::addPathConstraint %{
    ptr._markAdopted()
%}


// Include all the OpenSim code.
// =============================

opensim_unique_ptr(OpenSim::MocoProblemRep);

%include <Bindings/moco.i>
