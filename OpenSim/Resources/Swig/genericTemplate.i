/*Directors %module(directors="1") opensimModel */
%module opensimModel
%{
#include <OpenSim/Tools/rdToolsDLL.h>
#include <OpenSim/Simulation/rdSimulationDLL.h>
#include <OpenSim/Tools/Exception.h>
#include <OpenSim/Tools/Array.h>
#include <OpenSim/Tools/ArrayPtrs.h>
#include <OpenSim/Tools/Property.h>
#include <OpenSim/Tools/PropertySet.h>
#include <OpenSim/Tools/Object.h>
#include <OpenSim/Tools/Material.h>
#include <OpenSim/Tools/VisibleProperties.h>
#include <OpenSim/Tools/Transform.h>
#include <OpenSim/Tools/Geometry.h>
#include <OpenSim/Tools/VisibleObject.h>

#include <OpenSim/Tools/Set.h>

#include <OpenSim/Tools/MaterialSet.h>

#include <OpenSim/Simulation/SIMM/AbstractActuator.h>
#include <OpenSim/Simulation/SIMM/ActuatorSet.h>

#include <OpenSim/Simulation/Model/ContactForceSet.h>

#include <OpenSim/Tools/StateVector.h>
#include <OpenSim/Tools/Storage.h>

#include <OpenSim/Simulation/SIMM/AbstractModel.h>
#include <OpenSim/Simulation/Control/Control.h>
#include <OpenSim/Simulation/Control/ControlSet.h>
#include <OpenSim/Simulation/Control/ControlConstant.h>
#include <OpenSim/Simulation/Control/ControlLinear.h>
#include <OpenSim/Simulation/Control/Controller.h>
#include <OpenSim/Simulation/Integrator/Integrand.h>
#include <OpenSim/Simulation/Integrator/RKF.h>
#include <OpenSim/Simulation/Integrator/IntegRKF.h>
#include <OpenSim/Simulation/Model/ModelIntegrand.h>
#include <OpenSim/Simulation/Manager/Manager.h>
#include <OpenSim/Simulation/Model/Callback.h>
#include <OpenSim/Simulation/Model/CallbackSet.h>
#include <OpenSim/Simulation/Model/IntegCallback.h>
#include <OpenSim/Simulation/Model/Analysis.h>
#include <OpenSim/Simulation/Model/AnalysisSet.h>
#include <OpenSim/Simulation/Model/AnalysisFactory.h>
#include <OpenSim/Simulation/Model/Investigation.h>
#include <OpenSim/Analyses/suAnalysesDLL.h>
#include <OpenSim/Analyses/InvestigationForward.h>
#include <OpenSim/Analyses/InvestigationPerturbation.h>

#include <OpenSim/Simulation/Simtk/SimtkAnimationCallback.h>

#include <OpenSim/Analyses/Actuation.h>
#include <OpenSim/Analyses/IndAcc.h>
#include <OpenSim/Analyses/Kinematics.h>
#include <OpenSim/Analyses/GeneralizedForces.h>

#include <OpenSim/Simulation/SIMM/AbstractMarker.h>

#include <OpenSim/Simulation/SIMM/MarkerSet.h>
#include <OpenSim/Tools/Range.h>
#include <OpenSim/Tools/Scale.h>
#include <OpenSim/Tools/ScaleSet.h>

	/* This group of headers added by KMS 3/22/06 */
#include <OpenSim/Simulation/SIMM/AbstractDynamicsEngine.h>

#include <OpenSim/Simulation/SIMM/AbstractBody.h>
#include <OpenSim/Simulation/SIMM/SimmBody.h>
#include <OpenSim/Simulation/SIMM/BodySet.h>

#include <OpenSim/Tools/Function.h>
#include <OpenSim/Tools/Constant.h>
#include <OpenSim/Simulation/SIMM/AbstractCoordinate.h>
#include <OpenSim/Simulation/SIMM/SimmCoordinate.h>
#include <OpenSim/Simulation/SIMM/CoordinateSet.h>

#include <OpenSim/Simulation/SIMM/AbstractDof.h>

#include <OpenSim/Simulation/SIMM/AbstractJoint.h>
#include <OpenSim/Simulation/SIMM/SimmJoint.h>
#include <OpenSim/Simulation/SIMM/JointSet.h>

#include <OpenSim/Simulation/SIMM/SimmKinematicsEngine.h>
#include <OpenSim/Simulation/SIMM/AbstractMarker.h>
#include <OpenSim/Simulation/SIMM/SimmMarker.h>

#include <OpenSim/Simulation/SIMM/SimmMuscleGroup.h>
#include <OpenSim/Simulation/SIMM/SimmMusclePoint.h>
#include <OpenSim/Simulation/SIMM/SimmMusclePointSet.h>
#include <OpenSim/Simulation/SIMM/SimmMuscleViaPoint.h>
#include <OpenSim/Simulation/SIMM/SimmPoint.h>
#include <OpenSim/Simulation/SIMM/SimmRotationDof.h>
#include <OpenSim/Simulation/SIMM/SimmTranslationDof.h>
#include <OpenSim/Simulation/SIMM/SimmUnits.h>

#include <OpenSim/Applications/Workflow/workflowDLL.h>
#include <OpenSim/Subject/SimmGenericModelMaker.h>
#include <OpenSim/Subject/SimmModelScaler.h>
#include <OpenSim/Subject/SimmMarkerPlacer.h>
#include <OpenSim/Subject/SimmIKTrial.h>
#include <OpenSim/Subject/SimmIKTrialSet.h>

#include <OpenSim/Subject/simmSubject.h>
#include <OpenSim/Subject/SimmFileWriter.h>
#include <OpenSim/Simulation/SIMM/SimmMotionData.h>
#include <OpenSim/Simulation/SIMM/SimmMarkerData.h>

#include <OpenSim/Simulation/SIMM/IKSolverInterface.h>
#include <OpenSim/Simulation/SIMM/SimmMeasurement.h>
#include <OpenSim/Simulation/SIMM/SimmMeasurementSet.h>

#include <OpenSim/Applications/IK/InvestigationIK.h>
#include <OpenSim/Applications/IK/SimmIKSolverImpl.h>
#include <OpenSim/SQP/rdSQPDLL.h>
#include <OpenSim/SQP/rdOptimizationTarget.h>

#include <OpenSim/Cmc/rdCMCDLL.h>
#include <OpenSim/Cmc/InvestigationCMCGait.h>

using namespace OpenSim;
%}

/*Directors %feature("director") OpenSim::SimtkAnimationCallback; */


%rename(OpenSimObject) OpenSim::Object;
%rename(OpenSimException) OpenSim::Exception;

/* This file is for creation/handling of arrays */
%include "arrays_java.i";

/* This interface file is for better handling of pointers and references */
%include "typemaps.i"
%include "std_string.i"


%typemap(javacode) OpenSim::Object %{
  public boolean equals(Object obj) {
    boolean equal = false;
    if (obj instanceof $javaclassname)
      equal = ((($javaclassname)obj).swigCPtr == this.swigCPtr);
    return equal;
  }
  public int hashCode() {
    return( getName().hashCode()+10000 * getType().hashCode());
  }
%}

%pragma(java) jniclassclassmodifiers="public class"

%pragma(java) jniclassimports="import org.opensim.utils.TheApp;"

%pragma(java) jniclasscode=%{
  static {
      try{
        System.loadLibrary("rdModelDll");
      }
      catch(UnsatisfiedLinkError e){
           TheApp.exitApp("Required library failed to load. Check that the dynamic library rdModelDll is in your PATH\n"+e);
      }
  }
%}

/* make getCPtr public not package private */
%typemap(javabody) SWIGTYPE, SWIGTYPE *, SWIGTYPE &, SWIGTYPE [], SWIGTYPE (CLASS::*)%{
  private long swigCPtr;
  protected boolean swigCMemOwn;

  public $javaclassname(long cPtr, boolean cMemoryOwn) {
    swigCMemOwn = cMemoryOwn;
    swigCPtr = cPtr;
  }

  public static long getCPtr($javaclassname obj) {
    return (obj == null) ? 0 : obj.swigCPtr;
  }
%}

%typemap(javabody_derived) SWIGTYPE %{
  private long swigCPtr;

  public $javaclassname(long cPtr, boolean cMemoryOwn) {
    super(opensimModelJNI.SWIGStorageUpcast(cPtr), cMemoryOwn);
    swigCPtr = cPtr;
  }

  public static long getCPtr($javaclassname obj) {
    return (obj == null) ? 0 : obj.swigCPtr;
  }
%}
// Generic Exception handling
%typemap(throws) SWIGTYPE, SWIGTYPE &, SWIGTYPE *, SWIGTYPE [ANY] %{
  SWIG_JavaThrowException(jenv, SWIG_JavaIOException,
                          "C++ $1_type exception thrown");
  return $null;
%}

%typemap(throws, throws="java.io.IOException") OpenSim::Exception {
  jclass excep = jenv->FindClass("java/io/IOException");
  if (excep)
    jenv->ThrowNew(excep, ($1).getMessage());
  return $null;
}

%exception OpenSim::AnalyticGeometry::dynamic_cast(Geometry *geometry) {
    $action
    if (!result) {
        jclass excep = jenv->FindClass("java/lang/ClassCastException");
        if (excep) {
            jenv->ThrowNew(excep, "dynamic_cast exception");
        }
    }
}
%extend OpenSim::AnalyticGeometry {
    static AnalyticGeometry *dynamic_cast(Geometry *geometry) {
        return dynamic_cast<AnalyticGeometry *>(geometry);
    }
};

%extend OpenSim::LineGeometry {
    static LineGeometry *dynamic_cast(Geometry *geometry) {
        return dynamic_cast<LineGeometry *>(geometry);
    }
};

/* rest of header files to be wrapped */
%include <OpenSim/Tools/rdToolsDLL.h>
%include <OpenSim/Simulation/rdSimulationDLL.h>
%include <OpenSim/Tools/Exception.h>
%include <OpenSim/Tools/Array.h>
%include <OpenSim/Tools/ArrayPtrs.h>
%include <OpenSim/Tools/Property.h>
%include <OpenSim/Tools/PropertySet.h>
%include <OpenSim/Tools/Object.h>
%include <OpenSim/Tools/Material.h>
%include <OpenSim/Tools/VisibleProperties.h>
%include <OpenSim/Tools/Transform.h>
%include <OpenSim/Tools/Geometry.h>
%include <OpenSim/Tools/VisibleObject.h>

%include <OpenSim/Tools/Set.h>

%template(SetMaterials) OpenSim::Set<OpenSim::Material>;
%include <OpenSim/Tools/MaterialSet.h>

%include <OpenSim/Simulation/SIMM/AbstractActuator.h>
%template(SetActuators) OpenSim::Set<OpenSim::AbstractActuator>;
%include <OpenSim/Simulation/SIMM/ActuatorSet.h>

%include <OpenSim/Simulation/Model/ContactForceSet.h>
%include <OpenSim/Tools/StateVector.h>
%include <OpenSim/Tools/Storage.h>

%include <OpenSim/Simulation/SIMM/AbstractModel.h>

%include <OpenSim/Simulation/Control/Control.h>
%template(SetControls) OpenSim::Set<OpenSim::Control>;
%include <OpenSim/Simulation/Control/ControlSet.h>
%include <OpenSim/Simulation/Control/ControlConstant.h>
%include <OpenSim/Simulation/Control/ControlLinear.h>
%include <OpenSim/Simulation/Control/Controller.h>

%include <OpenSim/Simulation/Integrator/Integrand.h>
%include <OpenSim/Simulation/Model/ModelIntegrand.h>
%include <OpenSim/Simulation/Integrator/RKF.h>
%include <OpenSim/Simulation/Integrator/IntegRKF.h>
%include <OpenSim/Simulation/Manager/Manager.h>
%include <OpenSim/Simulation/Model/Callback.h>
%template(SetCallback) OpenSim::Set<OpenSim::Callback>;
%include <OpenSim/Simulation/Model/CallbackSet.h>
%include <OpenSim/Simulation/Model/IntegCallback.h>
%template(ArrayStorage) OpenSim::ArrayPtrs<OpenSim::Storage>;
%include <OpenSim/Simulation/Model/Analysis.h>
%template(SetAnalysis) OpenSim::Set<OpenSim::Analysis>;
%include <OpenSim/Simulation/Model/AnalysisSet.h>
%include <OpenSim/Simulation/Model/AnalysisFactory.h>
%include <OpenSim/Simulation/Model/Investigation.h>
%include <OpenSim/Analyses/suAnalysesDLL.h>
%include <OpenSim/Analyses/InvestigationForward.h>
%include <OpenSim/Analyses/InvestigationPerturbation.h>

%include <OpenSim/Simulation/Simtk/SimtkAnimationCallback.h>

%include <OpenSim/Analyses/suAnalysesDLL.h>
%include <OpenSim/Analyses/Kinematics.h>
%include <OpenSim/Analyses/Actuation.h>
%include <OpenSim/Analyses/IndAcc.h>
%include <OpenSim/Analyses/GeneralizedForces.h>

%template(ArrayBool) OpenSim::Array<bool>;
%template(ArrayDouble) OpenSim::Array<double>;
%template(ArrayInt) OpenSim::Array<int>;
%template(ArrayStr) OpenSim::Array<std::string>;
%template(ArrayPtrsObj) OpenSim::ArrayPtrs<OpenSim::Object>;
%include <OpenSim/Simulation/SIMM/AbstractMarker.h>
%template(SetMarkers) OpenSim::Set<OpenSim::AbstractMarker>;
%include <OpenSim/Simulation/SIMM/MarkerSet.h>
%include <OpenSim/Tools/Range.h>
%include <OpenSim/Tools/Scale.h>
%template(SetScales) OpenSim::Set<OpenSim::Scale>;
%include <OpenSim/Tools/ScaleSet.h>

	/* This group of headers added by KMS 3/22/06 */
%include <OpenSim/Simulation/SIMM/AbstractBody.h>
%include <OpenSim/Simulation/SIMM/SimmBody.h>
%template(SetBodies) OpenSim::Set<OpenSim::AbstractBody>;
%include <OpenSim/Simulation/SIMM/BodySet.h>

%include <OpenSim/Tools/Function.h>
%include <OpenSim/Tools/Constant.h>

%include <OpenSim/Simulation/SIMM/AbstractDof.h>
%include <OpenSim/Simulation/SIMM/AbstractCoordinate.h>
%include <OpenSim/Simulation/SIMM/SimmCoordinate.h>
%template(SetCoordinates) OpenSim::Set<OpenSim::AbstractCoordinate>;
%include <OpenSim/Simulation/SIMM/CoordinateSet.h>

%include <OpenSim/Simulation/SIMM/AbstractJoint.h>
%include <OpenSim/Simulation/SIMM/SimmJoint.h>
%template(SetJoints) OpenSim::Set<OpenSim::AbstractJoint>;
%include <OpenSim/Simulation/SIMM/JointSet.h>

%include <OpenSim/Simulation/SIMM/AbstractDynamicsEngine.h>
%include <OpenSim/Simulation/SIMM/SimmKinematicsEngine.h>


%include <OpenSim/Simulation/SIMM/AbstractMarker.h>
%include <OpenSim/Simulation/SIMM/SimmMarker.h>

%include <OpenSim/Simulation/SIMM/SimmMuscleGroup.h>

%include <OpenSim/Simulation/SIMM/SimmMusclePoint.h>
%include <OpenSim/Simulation/SIMM/SimmMuscleViaPoint.h>
%template(SetSimmMusclePoint) OpenSim::Set<OpenSim::SimmMusclePoint>;
%include <OpenSim/Simulation/SIMM/SimmMusclePointSet.h>

%include <OpenSim/Simulation/SIMM/SimmPoint.h>
%include <OpenSim/Simulation/SIMM/SimmRotationDof.h>
%include <OpenSim/Simulation/SIMM/SimmTranslationDof.h>
%include <OpenSim/Simulation/SIMM/SimmUnits.h>

%include <OpenSim/Applications/Workflow/workflowDLL.h>
%include <OpenSim/Subject/SimmGenericModelMaker.h>
%include <OpenSim/Subject/SimmModelScaler.h>
%include <OpenSim/Subject/SimmMarkerPlacer.h>
%include <OpenSim/Subject/SimmIKTrial.h>
%template(SetSimmIKTrial) OpenSim::Set<OpenSim::SimmIKTrial>;
%include <OpenSim/Subject/SimmIKTrialSet.h>

%include <OpenSim/Subject/simmSubject.h>
%include <OpenSim/Subject/SimmFileWriter.h>
%include <OpenSim/Simulation/SIMM/SimmMotionData.h>
%include <OpenSim/Simulation/SIMM/SimmMarkerData.h>

%include <OpenSim/Simulation/SIMM/IKSolverInterface.h>
%include <OpenSim/Applications/IK/InvestigationIK.h>
%include <OpenSim/Simulation/SIMM/SimmMeasurement.h>
%template(SetSimmMeasurements) OpenSim::Set<OpenSim::SimmMeasurement>;
%include <OpenSim/Simulation/SIMM/SimmMeasurementSet.h>
%include <OpenSim/Applications/IK/SimmIKSolverImpl.h>
%include <OpenSim/SQP/rdSQPDLL.h>
%include <OpenSim/SQP/rdOptimizationTarget.h>
%include <OpenSim/Cmc/rdCMCDLL.h>
%include <OpenSim/Cmc/InvestigationCMCGait.h>
