%module(package="opensim", directors="1") simulation
#pragma SWIG nowarn=822,451,503,516,325
// 401 is "Nothing known about base class *some-class*.
//         Maybe you forgot to instantiate *some-template* using %template."
// When wrapping new classes it's good to uncomment the line below to make sure
// you've wrapped your new class properly. However, SWIG generates lots of 401
// errors that are very difficult to resolve.
#pragma SWIG nowarn=401

%{
#define SWIG_FILE_WITH_INIT
#include <Bindings/OpenSimHeaders_common.h>
#include <Bindings/OpenSimHeaders_simulation.h>
%}

%{
using namespace OpenSim;
using namespace SimTK;
%}

// Ignore method that is not callable from Python (uses double[] arg)
%ignore OpenSim::Coordinate::setRange;


%include "python_preliminaries.i"

// Tell SWIG about the modules we depend on.
%import "python_common.i"

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

// Rename
// ======

%rename(appendNative) OpenSim::ForceSet::append(Force* aForce);

// Memory management
// =================
/*
The added component is not responsible for its own memory management anymore
once added to the Model.  These lines must go at this point in this .i file. I
originally had them at the bottom, and then they didn't work!

note: ## is a "glue" operator: `a ## b` --> `ab`.
*/
%define MODEL_ADOPT_HELPER(NAME)
%pythonappend OpenSim::Model::add ## NAME %{
    adoptee._markAdopted()
%}
%enddef

MODEL_ADOPT_HELPER(ModelComponent);
MODEL_ADOPT_HELPER(Body);
MODEL_ADOPT_HELPER(Marker)
MODEL_ADOPT_HELPER(Probe);
MODEL_ADOPT_HELPER(Joint);
MODEL_ADOPT_HELPER(Constraint);
MODEL_ADOPT_HELPER(ContactGeometry);
MODEL_ADOPT_HELPER(Analysis);
MODEL_ADOPT_HELPER(Force);
MODEL_ADOPT_HELPER(Controller);


%pythonappend OpenSim::Frame::attachGeometry %{
    geom._markAdopted()
%}

%pythonappend OpenSim::PhysicalFrame::addWrapObject %{
    wrapObject._markAdopted()
%}

// PrescribedController::prescribeControlForActuator takes ownership of
// the passed-in function.
// There are two overloads of this function; we append to both of them.
// args[1] is `Function* prescribedFunction`.
%pythonappend OpenSim::PrescribedController::prescribeControlForActuator %{
    args[1]._markAdopted()
%}

// PathPointSet takes ownership of passed in object
%pythonappend OpenSim::PathPointSet::insert %{
    aObject._markAdopted()
%}
// Typemaps
// ========
// None.

// Pythonic operators
// ==================
// Allow iterating through a StatesTrajectory.
// This extend block must appear before the %template call in simulation.i.

// TODO remove.
%rename(_getBetween) OpenSim::StatesTrajectory::getBetween;

%extend OpenSim::StatesTrajectory {
%pythoncode %{

    def __iter__(self):
        """Get an iterator for this Set, to be used as such (where `states` is
        the StatesTrajectory object)::

            for state in states:
                model.calcMassCenterPosition(state)
        """
        it = self.begin()
        while it != self.end():
            yield it.next()

    def getBetween(self, *args, **kwargs):
        iter_range = self._getBetween(*args, **kwargs)
        it = iter_range.begin()
        while it != iter_range.end():
            yield it.next()
%}
};

// TODO we already made a StdVectorState in simbody.i, but this is required
// to create type traits for the simulation module. Ideally, we would not need
// the following line:
%template(_StdVectorState) std::vector<SimTK::State>;

opensim_unique_ptr(OpenSim::PositionMotion);

// Include all the OpenSim code.
// =============================
%include <Bindings/preliminaries.i>
%include <Bindings/simulation.i>


// Memory management
// =================
SET_ADOPT_HELPER(BodyScale);
SET_ADOPT_HELPER(PathPoint);
SET_ADOPT_HELPER(Marker);
SET_ADOPT_HELPER(Control);
SET_ADOPT_HELPER(Force);
SET_ADOPT_HELPER(Analysis);

// This didn't work with the macro for some reason. I got complaints about
// multiple definitions of, e.g., Probe in the target language.
%extend OpenSim::ProbeSet {
%pythoncode %{
    def adoptAndAppend(self, aProbe):
        aProbe._markAdopted()
        return super(ProbeSet, self).adoptAndAppend(aProbe)
%}
};

// Attempt to solve segfault when calling ForceSet::append()
// from scripting.
%extend OpenSim::ForceSet {
%pythoncode %{
    def append(self, aForce):
        aForce._markAdopted()
        return self.appendNative(aForce)
%}
};

// Pythonic operators
// ==================
// Allow indexing operator in python (e.g., states[i]).
%extend OpenSim::StatesTrajectory {
    const SimTK::State&  __getitem__(int i) const {
        return $self->get(i);
    }
};
