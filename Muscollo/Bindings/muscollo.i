%newobject *::clone;

%include <Muscollo/osimMuscolloDLL.h>
%include <Muscollo/MuscolloUtilities.h>
%include <Muscollo/MucoCost.h>
%include <Muscollo/MucoWeightSet.h>
%include <Muscollo/MucoStateTrackingCost.h>
%include <Muscollo/MucoMarkerTrackingCost.h>
%include <Muscollo/MucoControlCost.h>

%include <Muscollo/MucoBounds.h>
%include <Muscollo/MucoProblem.h>
%include <Muscollo/MucoParameter.h>

// Workaround for SWIG not supporting inherited constructors.
%define EXPOSE_BOUNDS_CONSTRUCTORS_HELPER(NAME)
%extend OpenSim::NAME {
    NAME() { return new NAME(); }
    NAME(double value) { return new NAME(value); }
    NAME(double lower, double upper) { return new NAME(lower, upper); }
};
%enddef
EXPOSE_BOUNDS_CONSTRUCTORS_HELPER(MucoInitialBounds);
EXPOSE_BOUNDS_CONSTRUCTORS_HELPER(MucoFinalBounds);


/* SWIG does not support initializer_list, but we can use Java arrays to
 * achieve similar syntax in MATLAB. */
%ignore OpenSim::MucoIterate::setTime(std::initializer_list<double>);
%ignore OpenSim::MucoIterate::setState(const std::string&,
        std::initializer_list<double>);
%ignore OpenSim::MucoIterate::setControl(const std::string&,
        std::initializer_list<double>);

%include <Muscollo/MucoIterate.h>

%include <Muscollo/MucoSolver.h>


namespace OpenSim {
    %ignore MucoTropterSolver::MucoTropterSolver(const MucoProblem&);
}
%include <Muscollo/MucoTropterSolver.h>
%include <Muscollo/MucoTool.h>

