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
#include <OpenSim/Tools/VisibleObject.h>

#include <OpenSim/Tools/Set.h>
#include <OpenSim/Simulation/Model/Body.h>
#include <OpenSim/Simulation/Model/BodySet.h>

#include <OpenSim/Tools/MaterialSet.h>

#include <OpenSim/Simulation/Model/Actuator.h>
#include <OpenSim/Simulation/Model/ActuatorSet.h>

#include <OpenSim/Simulation/Model/ContactForceSet.h>

#include <OpenSim/Tools/StateVector.h>
#include <OpenSim/Tools/Storage.h>

#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/Control/Control.h>
#include <OpenSim/Simulation/Control/ControlSet.h>
#include <OpenSim/Simulation/Control/ControlConstant.h>
#include <OpenSim/Simulation/Control/ControlLinear.h>
#include <OpenSim/Simulation/Integrator/Integrand.h>
#include <OpenSim/Simulation/Integrator/RKF.h>
#include <OpenSim/Simulation/Integrator/IntegRKF.h>
#include <OpenSim/Simulation/Model/ModelIntegrand.h>
#include <OpenSim/Simulation/Manager/Manager.h>
#include <OpenSim/Simulation/Model/Callback.h>
#include <OpenSim/Simulation/Model/CallbackSet.h>
#include <OpenSim/Simulation/Model/IntegCallback.h>
#include <OpenSim/Simulation/Simtk/SimtkAnimationCallback.h>
#include <OpenSim/Simulation/Model/Analysis.h>
#include <OpenSim/Simulation/Model/AnalysisSet.h>
#include <OpenSim/Simulation/Model/AnalysisFactory.h>

#include <OpenSim/Analyses/Actuation.h>
#include <OpenSim/Analyses/IndAcc.h>
#include <OpenSim/Analyses/Kinematics.h>
#include <OpenSim/Analyses/GeneralizedForces.h>

#include <OpenSim/Simulation/Model/Marker.h>
#include <OpenSim/Simulation/Model/MarkerSet.h>
#include <OpenSim/Tools/Range.h>
#include <OpenSim/Tools/Scale.h>
#include <OpenSim/Tools/ScaleSet.h>

	/* This group of headers added by KMS 3/22/06 */
#include <OpenSim/Simulation/Model/AbstractDynamicsEngine.h>
#include <OpenSim/Simulation/SIMM/SimmBody.h>
#include <OpenSim/Simulation/SIMM/SimmBone.h>
#include <OpenSim/Tools/Function.h>
#include <OpenSim/Simulation/SIMM/Constant.h>
#include <OpenSim/Simulation/SIMM/Coordinate.h>
#include <OpenSim/Simulation/SIMM/SimmCoordinate.h>
#include <OpenSim/Simulation/SIMM/SimmDof.h>
#include <OpenSim/Simulation/SIMM/SimmJoint.h>
#include <OpenSim/Simulation/SIMM/SimmKinematicsEngine.h>
#include <OpenSim/Simulation/SIMM/SimmMarker.h>
#include <OpenSim/Simulation/SIMM/SimmModel.h>
#include <OpenSim/Simulation/SIMM/SimmMuscle.h>
#include <OpenSim/Simulation/SIMM/SimmMuscleGroup.h>
#include <OpenSim/Simulation/SIMM/SimmMusclePoint.h>
#include <OpenSim/Simulation/SIMM/SimmMuscleViaPoint.h>
#include <OpenSim/Simulation/SIMM/SimmPath.h>
#include <OpenSim/Simulation/SIMM/SimmPathMatrix.h>
#include <OpenSim/Simulation/SIMM/SimmPoint.h>
#include <OpenSim/Simulation/SIMM/SimmRotationDof.h>
#include <OpenSim/Simulation/SIMM/SimmStep.h>
#include <OpenSim/Simulation/SIMM/SimmTranslationDof.h>
#include <OpenSim/Simulation/SIMM/SimmUnits.h>

#include <OpenSim/Simulation/SIMM/SimmGenericModelParams.h>
#include <OpenSim/Simulation/SIMM/SimmScalingParams.h>
#include <OpenSim/Simulation/SIMM/SimmMarkerPlacementParams.h>
#include <OpenSim/Simulation/SIMM/SimmIKParams.h>
#include <OpenSim/Simulation/SIMM/SimmSubject.h>

using namespace OpenSim;
%}

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

%pragma(java) jniclasscode=%{
  static {
    try {
        System.loadLibrary("rdModelDll");
    } catch (UnsatisfiedLinkError e) {
      System.err.println("Native code library failed to load. Check that the dynamic library rdModelDll is in the PATH\n" + e);
      System.exit(1);
    }
  }
%}

/* make getCPtr public not package private */
%typemap(javabody) SWIGTYPE *, SWIGTYPE &, SWIGTYPE [], SWIGTYPE (CLASS::*)  %{
  private long swigCPtr;

  public $javaclassname(long cPtr, boolean bFutureUse) {
    swigCPtr = cPtr;
  }

  protected $javaclassname() {
    swigCPtr = 0;
  }

  public static long getCPtr($javaclassname obj) {
    return (obj == null) ? 0 : obj.swigCPtr;
  }
%}


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
%include <OpenSim/Tools/VisibleObject.h>

%include <OpenSim/Tools/Set.h>

%include <OpenSim/Simulation/Model/Body.h>
%template(SetBodies) OpenSim::Set<OpenSim::Body>;
%include <OpenSim/Simulation/Model/BodySet.h>

%template(SetMaterials) OpenSim::Set<OpenSim::Material>;
%include <OpenSim/Tools/MaterialSet.h>

%include <OpenSim/Simulation/Model/Actuator.h>
%template(SetActuators) OpenSim::Set<OpenSim::Actuator>;
%include <OpenSim/Simulation/Model/ActuatorSet.h>

%include <OpenSim/Simulation/Model/ContactForceSet.h>
%include <OpenSim/Tools/StateVector.h>
%include <OpenSim/Tools/Storage.h>

%include <OpenSim/Simulation/Model/Model.h>

%include <OpenSim/Simulation/Control/Control.h>
%template(SetControls) OpenSim::Set<OpenSim::Control>;
%include <OpenSim/Simulation/Control/ControlSet.h>
%include <OpenSim/Simulation/Control/ControlConstant.h>
%include <OpenSim/Simulation/Control/ControlLinear.h>

%include <OpenSim/Simulation/Integrator/Integrand.h>
%include <OpenSim/Simulation/Model/ModelIntegrand.h>
%include <OpenSim/Simulation/Integrator/RKF.h>
%include <OpenSim/Simulation/Integrator/IntegRKF.h>
%include <OpenSim/Simulation/Manager/Manager.h>
%include <OpenSim/Simulation/Model/Callback.h>
%template(SetCallback) OpenSim::Set<OpenSim::Callback>;
%include <OpenSim/Simulation/Model/CallbackSet.h>
%include <OpenSim/Simulation/Model/IntegCallback.h>
%include <OpenSim/Simulation/Simtk/SimtkAnimationCallback.h>
%template(ArrayStorage) OpenSim::ArrayPtrs<OpenSim::Storage>;
%include <OpenSim/Simulation/Model/Analysis.h>
%template(ArrayAnalysis) OpenSim::ArrayPtrs<OpenSim::Analysis>;
%template(SetAnalysis) OpenSim::Set<OpenSim::Analysis>;
%include <OpenSim/Simulation/Model/AnalysisFactory.h>

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
%include <OpenSim/Simulation/Model/Marker.h>
%template(SetMarkers) OpenSim::Set<OpenSim::Marker>;
%include <OpenSim/Simulation/Model/MarkerSet.h>
%include <OpenSim/Tools/Range.h>
%include <OpenSim/Tools/Scale.h>
%template(SetScales) OpenSim::Set<OpenSim::Scale>;
%include <OpenSim/Tools/ScaleSet.h>

	/* This group of headers added by KMS 3/22/06 */
%include <OpenSim/Simulation/Model/AbstractDynamicsEngine.h>
%include <OpenSim/Simulation/SIMM/SimmBody.h>
%include <OpenSim/Simulation/SIMM/SimmBone.h>
%include <OpenSim/Tools/Function.h>
%include <OpenSim/Simulation/SIMM/Constant.h>
%include <OpenSim/Simulation/SIMM/Coordinate.h>
%include <OpenSim/Simulation/SIMM/SimmCoordinate.h>
%include <OpenSim/Simulation/SIMM/SimmDof.h>
%include <OpenSim/Simulation/SIMM/SimmJoint.h>
%include <OpenSim/Simulation/SIMM/SimmKinematicsEngine.h>
%include <OpenSim/Simulation/SIMM/SimmMarker.h>
%include <OpenSim/Simulation/SIMM/SimmModel.h>
%include <OpenSim/Simulation/SIMM/SimmMuscle.h>
%include <OpenSim/Simulation/SIMM/SimmMuscleGroup.h>
%include <OpenSim/Simulation/SIMM/SimmMusclePoint.h>
%include <OpenSim/Simulation/SIMM/SimmMuscleViaPoint.h>
%include <OpenSim/Simulation/SIMM/SimmPath.h>
%include <OpenSim/Simulation/SIMM/SimmPathMatrix.h>
%include <OpenSim/Simulation/SIMM/SimmPoint.h>
%include <OpenSim/Simulation/SIMM/SimmRotationDof.h>
%include <OpenSim/Simulation/SIMM/SimmStep.h>
%include <OpenSim/Simulation/SIMM/SimmTranslationDof.h>
%include <OpenSim/Simulation/SIMM/SimmUnits.h>

%include <OpenSim/Simulation/SIMM/SimmGenericModelParams.h>
%include <OpenSim/Simulation/SIMM/SimmScalingParams.h>
%include <OpenSim/Simulation/SIMM/SimmMarkerPlacementParams.h>
%include <OpenSim/Simulation/SIMM/SimmIKParams.h>
%include <OpenSim/Simulation/SIMM/SimmSubject.h>
