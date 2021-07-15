%module(directors="1") opensimMoco

%{
#include <Bindings/OpenSimHeaders_common.h>
#include <Bindings/OpenSimHeaders_simulation.h>
#include <Bindings/OpenSimHeaders_actuators.h>
#include <Bindings/OpenSimHeaders_analyses.h>
#include <Bindings/OpenSimHeaders_tools.h>
#include <Bindings/OpenSimHeaders_moco.h>
using namespace OpenSim;
using namespace SimTK;
%}

%include <Bindings/preliminaries.i>
%include "java_preliminaries.i";

// In this file, we add functions suffixed with "...Mat"
// to the interface that take double[] arguments.
// This allows Matlab users to directly pass Matlab arrays to Moco's "...Mat"
// functions.

// An alternative to introducing the additional "...Mat" functions is to permit
// Moco's existing functions to take double[] arguments instead of their
// original SimTK::Vector arguments, using SWIG typemaps. These commented
// typemaps are an attempt at this alternative. However, we are not using this
// alternative because it would get rid of the possibility of passing
// SimTK::Vector, etc. to Moco's existing functions.
// %typemap(jtype) (const SimTK::Vector&) "double[]"
// %typemap(jstype) (const SimTK::Vector&) "double[]"
// %typemap(jni) (const SimTK::Vector&) "jdoubleArray"
// %typemap(javain) (const SimTK::Vector&) "$javainput"
// %typemap(in) (const SimTK::Vector&) {
//     const int size = (int) JCALL1(GetArrayLength, jenv, $input);
//     const double* data = JCALL2(GetDoubleArrayElements, jenv, $input, 0);
//     $1 = new SimTK::Vector(size, data);
// }
// %typemap(freearg) (const SimTK::Vector&) {
//     if ($1) {
//         delete $1;
//     }
// }
// %typemap(argout) (const SimTK::Vector&) {
//     delete $1;
//     $1 = NULL;
// }
// // VectorView
// %typemap(jtype) (const SimTK::Vector&) "double[]"
// %typemap(jstype) (const SimTK::Vector&) "double[]"
// %typemap(jni) (const SimTK::Vector&) "jdoubleArray"
// %typemap(javain) (const SimTK::Vector&) "$javainput"
// %typemap(in) (const SimTK::Vector&) {
//     const int size = (int) JCALL1(GetArrayLength, jenv, $input);
//     const double* data = JCALL2(GetDoubleArrayElements, jenv, $input, 0);
//     $1 = new SimTK::Vector(size, data);
// }
// %typemap(freearg) (const SimTK::Vector&) {
//     if ($1) {
//         delete $1;
//     }
// }
// %typemap(argout) (const SimTK::Vector&) {
//     delete $1;
//     $1 = NULL;
// }
//
// // When const SimTK::Vector& is the return type.
// // Change the implementation of, e.g., MocoTrajectory.setTime() in MocoTrajectory.java
// // from "return new Vector($jnicall)" to "return $jnicall" (since we no longer
// // return a Vector).
// %typemap(javaout) (const SimTK::Vector&) {
//     return $jnicall;
// }
// // In java_moco.cxx, convert the SimTK::Vector to a Java double array.
// // "&(*$1)[0] is the pointer to the start of array underlying the SimTK::Vector.
// %typemap(out) (const SimTK::Vector&) {
//     $result = JCALL1(NewDoubleArray, jenv, $1->size());
//     JCALL4(SetDoubleArrayRegion, jenv, $result, 0, $1->size(), &(*$1)[0]);
// }

// Memory management
// =================
%javamethodmodifiers OpenSim::MocoPhase::setModel "private";
%rename OpenSim::MocoPhase::setModel private_setModel;
%javamethodmodifiers OpenSim::MocoPhase::addParameter "private";
%rename OpenSim::MocoPhase::addParameter private_addParameter;
%javamethodmodifiers OpenSim::MocoPhase::addGoal "private";
%rename OpenSim::MocoPhase::addGoal private_addGoal;
%javamethodmodifiers OpenSim::MocoPhase::addPathConstraint "private";
%rename OpenSim::MocoPhase::addPathConstraint private_addPathConstraint;

%javamethodmodifiers OpenSim::MocoProblem::setModel "private";
%rename OpenSim::MocoProblem::setModel private_setModel;
%javamethodmodifiers OpenSim::MocoProblem::addParameter "private";
%rename OpenSim::MocoProblem::addParameter private_addParameter;
%javamethodmodifiers OpenSim::MocoProblem::addGoal "private";
%rename OpenSim::MocoProblem::addGoal private_addGoal;
%javamethodmodifiers OpenSim::MocoProblem::addPathConstraint "private";
%rename OpenSim::MocoProblem::addPathConstraint private_addPathConstraint;

// Interface
// =========

%typemap(javacode) OpenSim::MocoPhase %{
    public static MocoBounds convertArrayToMB(double[] arr) throws Exception {
        MocoBounds bounds = new MocoBounds();
        if (arr == null) {
            return bounds;
        } else if (arr.length > 2) {
            throw new RuntimeException(
                    "Bounds cannot have more than 2 elements.");
        } else if (arr.length == 1) {
            bounds = new MocoBounds(arr[0]);
        } else if (arr.length == 2) {
            bounds = new MocoBounds(arr[0], arr[1]);
        }
        return bounds;
    }
    public static MocoInitialBounds convertArrayToMIB(double[] arr)
            throws Exception {
        MocoInitialBounds bounds = new MocoInitialBounds();
        if (arr == null) {
            return bounds;
        } else if (arr.length > 2) {
            throw new RuntimeException(
                    "Bounds cannot have more than 2 elements.");
        } else if (arr.length == 1) {
            bounds = new MocoInitialBounds(arr[0]);
        } else if (arr.length == 2) {
            bounds = new MocoInitialBounds(arr[0], arr[1]);
        }
        return bounds;
    }
    public static MocoFinalBounds convertArrayToMFB(double[] arr)
            throws Exception {
        MocoFinalBounds bounds = new MocoFinalBounds();
        if (arr == null) {
            return bounds;
        } else if (arr.length > 2) {
            throw new RuntimeException(
                    "Bounds cannot have more than 2 elements.");
        } else if (arr.length == 1) {
            bounds = new MocoFinalBounds(arr[0]);
        } else if (arr.length == 2) {
            bounds = new MocoFinalBounds(arr[0], arr[1]);
        }
        return bounds;
    }

    public void setModel(Model model) {
        private_setModel(model);
        model.markAdopted();
    }
    public void addParameter(MocoParameter obj) {
        private_addParameter(obj);
        obj.markAdopted();
    }
    public void addGoal(MocoGoal obj) {
        private_addGoal(obj);
        obj.markAdopted();
    }
    public void addPathConstraint(MocoPathConstraint obj) {
        private_addPathConstraint(obj);
        obj.markAdopted();
    }
    public void setTimeBounds(double[] ib, double[] fb) throws Exception {
        setTimeBounds(this.convertArrayToMIB(ib), this.convertArrayToMFB(fb));
    }
    public void setStateInfo(String name, double[] b) throws Exception {
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
    public void setStateInfoPattern(String pattern, double[] b) 
        throws Exception {
        setStateInfoPattern(pattern, this.convertArrayToMB(b));
    }
    public void setStateInfoPattern(String pattern, double[] b, double[] ib)
        throws Exception {
        setStateInfoPattern(pattern, this.convertArrayToMB(b),
                this.convertArrayToMIB(ib));
    }
    public void 
    setStateInfoPattern(String pattern, double[] b, double[] ib, double[] fb) 
        throws Exception {
        setStateInfoPattern(pattern, this.convertArrayToMB(b),
                this.convertArrayToMIB(ib), this.convertArrayToMFB(fb));
    }

    public void setControlInfo(String name, double[] b) throws Exception {
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
    public void setControlInfoPattern(String pattern, double[] b) 
        throws Exception {
        setControlInfoPattern(pattern, this.convertArrayToMB(b));
    }
    public void setControlInfoPattern(String pattern, double[] b, double[] ib)
        throws Exception {
        setControlInfoPattern(pattern, this.convertArrayToMB(b),
                this.convertArrayToMIB(ib));
    }
    public void 
    setControlInfoPattern(String pattern, double[] b, double[] ib, double[] fb) 
        throws Exception {
        setControlInfoPattern(pattern, this.convertArrayToMB(b),
                this.convertArrayToMIB(ib), this.convertArrayToMFB(fb));
    }
%}

%typemap(javacode) OpenSim::MocoProblem %{
    public void setModel(Model model) {
        private_setModel(model);
        model.markAdopted();
    }
    public void addParameter(MocoParameter obj) {
        private_addParameter(obj);
        obj.markAdopted();
    }
    public void addGoal(MocoGoal obj) {
        private_addGoal(obj);
        obj.markAdopted();
    }
    public void addPathConstraint(MocoPathConstraint obj) {
        private_addPathConstraint(obj);
        obj.markAdopted();
    }
    public void setTimeBounds(double[] ib, double[] fb) throws Exception {
        setTimeBounds(MocoPhase.convertArrayToMIB(ib),
            MocoPhase.convertArrayToMFB(fb));
    }
    public void setStateInfo(String name, double[] b)
        throws Exception {
        setStateInfo(name, MocoPhase.convertArrayToMB(b));
    }
    public void setStateInfo(String name, double[] b, double[] ib)
        throws Exception {
        setStateInfo(name, MocoPhase.convertArrayToMB(b),
                MocoPhase.convertArrayToMIB(ib));
    }
    public void setStateInfo(String name, double[] b, double[] ib, double[] fb)
        throws Exception {
        setStateInfo(name, MocoPhase.convertArrayToMB(b),
                MocoPhase.convertArrayToMIB(ib), 
                MocoPhase.convertArrayToMFB(fb));
    }
    public void setStateInfoPattern(String pattern, double[] b)
        throws Exception {
        setStateInfoPattern(pattern, MocoPhase.convertArrayToMB(b));
    }
    public void setStateInfoPattern(String pattern, double[] b, double[] ib)
        throws Exception {
        setStateInfoPattern(pattern, MocoPhase.convertArrayToMB(b),
                MocoPhase.convertArrayToMIB(ib));
    }
    public void 
    setStateInfoPattern(String pattern, double[] b, double[] ib, double[] fb) 
        throws Exception {
        setStateInfoPattern(pattern, MocoPhase.convertArrayToMB(b),
                MocoPhase.convertArrayToMIB(ib), 
                MocoPhase.convertArrayToMFB(fb));
    }

    public void setControlInfo(String name, double[] b)
        throws Exception {
        setControlInfo(name, MocoPhase.convertArrayToMB(b));
    }
    public void setControlInfo(String name, double[] b, double[] ib)
        throws Exception {
        setControlInfo(name, MocoPhase.convertArrayToMB(b),
                MocoPhase.convertArrayToMIB(ib));
    }
    public void setControlInfo(String name, double[] b, double[] ib,
            double[] fb)
        throws Exception {
        setControlInfo(name, MocoPhase.convertArrayToMB(b),
                MocoPhase.convertArrayToMIB(ib), 
                MocoPhase.convertArrayToMFB(fb));
    }
    public void setControlInfoPattern(String pattern, double[] b)
        throws Exception {
        setControlInfoPattern(pattern, MocoPhase.convertArrayToMB(b));
    }
    public void setControlInfoPattern(String pattern, double[] b, double[] ib)
        throws Exception {
        setControlInfoPattern(pattern, MocoPhase.convertArrayToMB(b),
                MocoPhase.convertArrayToMIB(ib));
    }
    public void 
    setControlInfoPattern(String pattern, double[] b, double[] ib, double[] fb)
        throws Exception {
        setControlInfoPattern(pattern, MocoPhase.convertArrayToMB(b),
                MocoPhase.convertArrayToMIB(ib), 
                MocoPhase.convertArrayToMFB(fb));
    }
%}

%typemap(javacode) OpenSim::MocoControlTrackingGoal %{
    public void addScaleFactor(String name, String control, double[] b)
            throws Exception {
            addScaleFactor(name, control, MocoPhase.convertArrayToMB(b));
    }
%}

%typemap(javacode) OpenSim::MocoStateTrackingGoal %{
    public void addScaleFactor(String name, String state, double[] b)
            throws Exception {
            addScaleFactor(name, state, MocoPhase.convertArrayToMB(b));
    }
%}

%typemap(javacode) OpenSim::MocoMarkerTrackingGoal %{
    public void addScaleFactor(String name, String marker, int index, double[] b)
            throws Exception {
            addScaleFactor(name, marker, index, MocoPhase.convertArrayToMB(b));
    }
%}

%typemap(javacode) OpenSim::MocoContactTrackingGoal %{
    public void addScaleFactor(String name, String externalForceName, int index,
        double[] b) throws Exception {
        addScaleFactor(name, externalForceName, index,
                       MocoPhase.convertArrayToMB(b));
    }
%}

/* SWIG does not support initializer_list, but we can use Java arrays to
 * achieve similar syntax in MATLAB.
 * TODO create Vector(double[]) constructor. */
%typemap(javacode) OpenSim::MocoTrajectory %{
    public void setTime(double[] time) {
        Vector v = new Vector();
        v.resize(time.length);
        for (int i = 0; i < time.length; ++i) { v.set(i, time[i]); }
        setTime(v);
    }
    public void setState(String name, double[] traj) {
        Vector v = new Vector();
        v.resize(traj.length);
        for (int i = 0; i < traj.length; ++i) { v.set(i, traj[i]); }
        setState(name, v);
    }
    public void setControl(String name, double[] traj) {
        Vector v = new Vector();
        v.resize(traj.length);
        for (int i = 0; i < traj.length; ++i) { v.set(i, traj[i]); }
        setControl(name, v);
    }
    public void setMultiplier(String name, double[] traj) {
        Vector v = new Vector();
        v.resize(traj.length);
        for (int i = 0; i < traj.length; ++i) { v.set(i, traj[i]); }
        setMultiplier(name, v);
    }
    public void setDerivative(String name, double[] traj) {
        Vector v = new Vector();
        v.resize(traj.length);
        for (int i = 0; i < traj.length; ++i) { v.set(i, traj[i]); }
        setDerivative(name, v);
    }
    public double[] getTimeMat() {
        Vector time = getTime();
        double[] ret = new double[time.size()];
        for (int i = 0; i < time.size(); ++i) { ret[i] = time.get(i); };
        return ret;
    }
    public double[] getStateMat(String name) {
        VectorView state = getState(name);
        double[] ret = new double[state.size()];
        for (int i = 0; i < state.size(); ++i) { ret[i] = state.get(i); };
        return ret;
    }
    public double[] getControlMat(String name) {
        VectorView control = getControl(name);
        double[] ret = new double[control.size()];
        for (int i = 0; i < control.size(); ++i) { ret[i] = control.get(i); };
        return ret;
    }
    public double[] getMultiplierMat(String name) {
        VectorView mult = getMultiplier(name);
        double[] ret = new double[mult.size()];
        for (int i = 0; i < mult.size(); ++i) { ret[i] = mult.get(i); };
        return ret;
    }
    public double[] getDerivativeMat(String name) {
        VectorView deriv = getDerivative(name);
        double[] ret = new double[deriv.size()];
        for (int i = 0; i < deriv.size(); ++i) { ret[i] = deriv.get(i); };
        return ret;
    }
    public double[] getParametersMat() {
        RowVector params = getParameters();
        double[] ret = new double[params.size()];
        for (int i = 0; i < params.size(); ++i) { ret[i] = params.get(i); };
        return ret;
    }
    public double[][] getStatesTrajectoryMat() {
        Matrix matrix = getStatesTrajectory();
        double[][] ret = new double[matrix.nrow()][matrix.ncol()];
        for (int i = 0; i < matrix.nrow(); ++i) {
            for (int j = 0; j < matrix.ncol(); ++j) {
                ret[i][j] = matrix.getElt(i, j);
            }
        }
        return ret;
    }
    public double[][] getControlsTrajectoryMat() {
        Matrix matrix = getControlsTrajectory();
        double[][] ret = new double[matrix.nrow()][matrix.ncol()];
        for (int i = 0; i < matrix.nrow(); ++i) {
            for (int j = 0; j < matrix.ncol(); ++j) {
                ret[i][j] = matrix.getElt(i, j);
            }
        }
        return ret;
    }
    public double[][] getMultipliersTrajectoryMat() {
        Matrix matrix = getMultipliersTrajectory();
        double[][] ret = new double[matrix.nrow()][matrix.ncol()];
        for (int i = 0; i < matrix.nrow(); ++i) {
            for (int j = 0; j < matrix.ncol(); ++j) {
                ret[i][j] = matrix.getElt(i, j);
            }
        }
        return ret;
    }
    public double[][] getDerivativesTrajectoryMat() {
        Matrix matrix = getDerivativesTrajectory();
        double[][] ret = new double[matrix.nrow()][matrix.ncol()];
        for (int i = 0; i < matrix.nrow(); ++i) {
            for (int j = 0; j < matrix.ncol(); ++j) {
                ret[i][j] = matrix.getElt(i, j);
            }
        }
        return ret;
    }
%}

opensim_unique_ptr(OpenSim::MocoProblemRep);

%import "java_actuators.i"

%include <Bindings/moco.i>


