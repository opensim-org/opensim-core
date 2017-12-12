%include <Muscollo/osimMuscolloDLL.h>
%include <Muscollo/MucoCost.h>
%include <Muscollo/MucoWeightSet.h>
%include <Muscollo/MucoStateTrackingCost.h>
%include <Muscollo/MucoMarkerTrackingCost.h>
%include <Muscollo/MucoControlCost.h>

%typemap(javacode) OpenSim::MucoPhase %{
    public static MucoBounds convertArrayToMB(double[] arr) throws Exception {
        MucoBounds bounds = new MucoBounds();
        if (arr.length > 2) {
            throw new RuntimeException(
                    "Bounds cannot have more than 2 elements.");
        } else if (arr.length == 1) {
            bounds = new MucoBounds(arr[0]);
        } else if (arr.length == 2) {
            bounds = new MucoBounds(arr[0], arr[1]);
        }
        return bounds;
    }
    public static MucoInitialBounds convertArrayToMIB(double[] arr)
            throws Exception {
        MucoInitialBounds bounds = new MucoInitialBounds();
        if (arr.length > 2) {
            throw new RuntimeException(
                    "Bounds cannot have more than 2 elements.");
        } else if (arr.length == 1) {
            bounds = new MucoInitialBounds(arr[0]);
        } else if (arr.length == 2) {
            bounds = new MucoInitialBounds(arr[0], arr[1]);
        }
        return bounds;
    }
    public static MucoFinalBounds convertArrayToMFB(double[] arr)
            throws Exception {
        MucoFinalBounds bounds = new MucoFinalBounds();
        if (arr.length > 2) {
            throw new RuntimeException(
                    "Bounds cannot have more than 2 elements.");
        } else if (arr.length == 1) {
            bounds = new MucoFinalBounds(arr[0]);
        } else if (arr.length == 2) {
            bounds = new MucoFinalBounds(arr[0], arr[1]);
        }
        return bounds;
    }

    public void setStateInfo(String name, double[] b)
        throws Exception {
        setStateInfo(name, this.convertArrayToMB(b));
    }
    public void setStateInfo(String name, double[] b, double[] ib)
        throws Exception {
        setStateInfo(name, this.convertArrayToMB(b),
                this.convertArrayToMIB(ib));
    }
    public void setStateInfo(String name, double[] b, double[] ib, double[] fb)
        throws Exception {
        setStateInfo(name, this.convertArrayToMB(b),
                this.convertArrayToMIB(ib), this.convertArrayToMFB(fb));
    }

    public void setControlInfo(String name, double[] b)
        throws Exception {
        setControlInfo(name, this.convertArrayToMB(b));
    }
    public void setControlInfo(String name, double[] b, double[] ib)
        throws Exception {
        setControlInfo(name, this.convertArrayToMB(b),
                this.convertArrayToMIB(ib));
    }
    public void setControlInfo(String name, double[] b, double[] ib, double[] fb)
        throws Exception {
        setControlInfo(name, this.convertArrayToMB(b),
                this.convertArrayToMIB(ib), this.convertArrayToMFB(fb));
    }
%}

%typemap(javacode) OpenSim::MucoProblem %{
    public void setStateInfo(String name, double[] b)
        throws Exception {
        setStateInfo(name, MucoPhase.convertArrayToMB(b));
    }
    public void setStateInfo(String name, double[] b, double[] ib)
        throws Exception {
        setStateInfo(name, MucoPhase.convertArrayToMB(b),
                MucoPhase.convertArrayToMIB(ib));
    }
    public void setStateInfo(String name, double[] b, double[] ib, double[] fb)
        throws Exception {
        setStateInfo(name, MucoPhase.convertArrayToMB(b),
                MucoPhase.convertArrayToMIB(ib), 
                MucoPhase.convertArrayToMFB(fb));
    }

    public void setControlInfo(String name, double[] b)
        throws Exception {
        setControlInfo(name, MucoPhase.convertArrayToMB(b));
    }
    public void setControlInfo(String name, double[] b, double[] ib)
        throws Exception {
        setControlInfo(name, MucoPhase.convertArrayToMB(b),
                MucoPhase.convertArrayToMIB(ib));
    }
    public void setControlInfo(String name, double[] b, double[] ib,
            double[] fb)
        throws Exception {
        setControlInfo(name, MucoPhase.convertArrayToMB(b),
                MucoPhase.convertArrayToMIB(ib), 
                MucoPhase.convertArrayToMFB(fb));
    }
%}

%include <Muscollo/MucoProblem.h>

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

/* TODO MucoInitialBounds constructor that takes double[] */


/* TODO want to be able to use initializer list. */
%ignore OpenSim::MucoIterate::setTime(std::initializer_list<double>);
%ignore OpenSim::MucoIterate::setState(const std::string&,
        std::initializer_list<double>);
%ignore OpenSim::MucoIterate::setControl(const std::string&,
        std::initializer_list<double>);
/* TODO %feature("valuewrapper") VectorView */
%include <Muscollo/MucoIterate.h>

%include <Muscollo/MucoSolver.h>


namespace OpenSim {
    %ignore MucoTropterSolver::MucoTropterSolver(const MucoProblem&);
}
%include <Muscollo/MucoTropterSolver.h>
%include <Muscollo/MucoTool.h>
