%module Model
%{
#include <NMBLTK/Tools/rdToolsDLL.h>
#include <NMBLTK/Simulation/rdSimulationDLL.h>
#include <NMBLTK/Tools/rdException.h>
#include <NMBLTK/Tools/rdArray.h>
#include <NMBLTK/Tools/rdArrayPtrs.h>
#include <NMBLTK/Tools/rdProperty.h>
#include <NMBLTK/Tools/rdPropertySet.h>
#include <NMBLTK/Tools/rdObject.h>
#include <NMBLTK/Tools/rdMaterial.h>
#include <NMBLTK/Tools/rdVisibleProperties.h>
#include <NMBLTK/Tools/rdTransform.h>
#include <NMBLTK/Tools/rdVisibleObject.h>

#include <NMBLTK/Tools/rdSet.h>
#include <NMBLTK/Simulation/Model/rdBody.h>
#include <NMBLTK/Simulation/Model/rdBodySet.h>

#include <NMBLTK/Tools/rdMaterialSet.h>

#include <NMBLTK/Simulation/Model/rdActuator.h>
#include <NMBLTK/Simulation/Model/rdActuatorSet.h>

#include <NMBLTK/Simulation/Model/rdContactForceSet.h>

#include <NMBLTK/Tools/rdStateVector.h>
#include <NMBLTK/Tools/rdStorage.h>

#include <NMBLTK/Simulation/Model/rdModel.h>
#include <NMBLTK/Simulation/Control/rdControl.h>
#include <NMBLTK/Simulation/Control/rdControlSet.h>
#include <NMBLTK/Simulation/Control/rdControlConstant.h>
#include <NMBLTK/Simulation/Control/rdControlLinear.h>
#include <NMBLTK/Simulation/Integrator/Integrand.h>
#include <NMBLTK/Simulation/Integrator/rdRKF.h>
#include <NMBLTK/Simulation/Integrator/rdIntegRKF.h>
#include <NMBLTK/Simulation/Model/rdModelIntegrand.h>
#include <NMBLTK/Simulation/Manager/rdManager.h>
#include <NMBLTK/Simulation/Model/rdCallback.h>
#include <NMBLTK/Simulation/Model/rdCallbackSet.h>
#include <NMBLTK/Simulation/Model/rdIntegCallback.h>
#include <NMBLTK/Simulation/Simtk/rdSimtkAnimationCallback.h>
#include <NMBLTK/Simulation/Model/rdAnalysis.h>
#include <NMBLTK/Simulation/Model/rdAnalysisSet.h>
#include <NMBLTK/Simulation/Model/rdAnalysisFactory.h>

#include <NMBLTK/Analyses/suAnalysisFactory.h>
#include <NMBLTK/Analyses/suActuation.h>
#include <NMBLTK/Analyses/suIndAcc.h>
#include <NMBLTK/Analyses/suKinematics.h>
#include <NMBLTK/Analyses/suGeneralizedForces.h>

#include <NMBLTK/Simulation/Model/suMarker.h>
#include <NMBLTK/Simulation/Model/suMarkerSet.h>
#include <NMBLTK/Tools/suRange.h>
#include <NMBLTK/Tools/suScale.h>
#include <NMBLTK/Tools/suScaleSet.h>

	/* This group of headers added by KMS 3/22/06 */
#include <NMBLTK/Simulation/Model/nmblKinematicsEngine.h>
#include <NMBLTK/Simulation/SIMM/simmBody.h>
#include <NMBLTK/Simulation/SIMM/simmBone.h>
#include <NMBLTK/Simulation/SIMM/simmConstant.h>
#include <NMBLTK/Simulation/SIMM/simmCoordinate.h>
#include <NMBLTK/Simulation/SIMM/simmDof.h>
#include <NMBLTK/Simulation/SIMM/simmJoint.h>
#include <NMBLTK/Simulation/SIMM/simmKinematicsEngine.h>
#include <NMBLTK/Simulation/SIMM/simmMarker.h>
#include <NMBLTK/Simulation/SIMM/simmModel.h>
#include <NMBLTK/Simulation/SIMM/simmMuscle.h>
#include <NMBLTK/Simulation/SIMM/simmMuscleGroup.h>
#include <NMBLTK/Simulation/SIMM/simmMusclePoint.h>
#include <NMBLTK/Simulation/SIMM/simmMuscleViaPoint.h>
#include <NMBLTK/Simulation/SIMM/simmPath.h>
#include <NMBLTK/Simulation/SIMM/simmPathMatrix.h>
#include <NMBLTK/Simulation/SIMM/simmPoint.h>
#include <NMBLTK/Simulation/SIMM/simmRotationDof.h>
#include <NMBLTK/Simulation/SIMM/simmStep.h>
#include <NMBLTK/Simulation/SIMM/simmTranslationDof.h>
#include <NMBLTK/Simulation/SIMM/simmUnits.h>

%}

/* This file is for creation/handling of arrays */
%include "sarrays.i"

/* This interface file is for better handling of pointers and references */
%include "typemaps.i"
%include "std_string.i"

/* inline code for rdObject.java */
%typemap(javacode) rdObject %{
  public boolean equals(Object aObject) {
    if (! (aObject instanceof rdObject))
      return false;
    rdObject rObj = (rdObject) aObject;
    return (this.getName().equals(rObj.getName()) &&
            this.getType().equals(rObj.getType()));
  }
  public int hashCode() {
    return( this.getName().hashCode()+10000 * getType().hashCode());
  }
%}
/* make getCPtr public not package private */
%typemap(javagetcptr) SWIGTYPE, SWIGTYPE *, SWIGTYPE &, SWIGTYPE [], SWIGTYPE (CLASS::*)  %{
  public static long getCPtr($javaclassname obj) {
    return (obj == null) ? 0 : obj.swigCPtr;
  }
%}
/* make constructor(log, <type>) public as well */
%typemap(javaptrconstructormodifiers) SWIGTYPE, SWIGTYPE *, SWIGTYPE &, SWIGTYPE [], SWIGTYPE (CLASS::*)  %{
  public %}

%include "exception.i"
// Exception handler (intended for JNI calls that have a non void return)
%exception {
  try {
    $function
  }
  catch (rdException) {
    jclass clazz = jenv->FindClass("simtkModel/rdException");
    jenv->ThrowNew(clazz, "Native Exception");
    return NULL;
  }
}

// This overrides anything in the %except(java) typemap
// Used for JNI calls that return void
%typemap(except) void {
  try {
    $function
  }
  catch (rdException) {
    jclass clazz = jenv->FindClass("simtkModel/rdException");
    jenv->ThrowNew(clazz, "Native Exception");
    return;
  }
}


/* rest of header files to be wrapped */
%include <NMBLTK/Tools/rdToolsDLL.h>
%include <NMBLTK/Simulation/rdSimulationDLL.h>
%include <NMBLTK/Tools/rdException.h>
%include <NMBLTK/Tools/rdArray.h>
%include <NMBLTK/Tools/rdArrayPtrs.h>
%include <NMBLTK/Tools/rdProperty.h>
%include <NMBLTK/Tools/rdPropertySet.h>
%include <NMBLTK/Tools/rdObject.h>
%include <NMBLTK/Tools/rdMaterial.h>
%include <NMBLTK/Tools/rdVisibleProperties.h>
%include <NMBLTK/Tools/rdTransform.h>
%include <NMBLTK/Tools/rdVisibleObject.h>

%include <NMBLTK/Tools/rdSet.h>

%include <NMBLTK/Simulation/Model/rdBody.h>
%template(rdSetBodies) rdSet<rdBody>;
%include <NMBLTK/Simulation/Model/rdBodySet.h>

%template(rdSetMaterials) rdSet<rdMaterial>;
%include <NMBLTK/Tools/rdMaterialSet.h>

%include <NMBLTK/Simulation/Model/rdActuator.h>
%template(rdSetActuators) rdSet<rdActuator>;
%include <NMBLTK/Simulation/Model/rdActuatorSet.h>

%include <NMBLTK/Simulation/Model/rdContactForceSet.h>
%include <NMBLTK/Tools/rdStateVector.h>
%include <NMBLTK/Tools/rdStorage.h>

%include <NMBLTK/Simulation/Model/rdModel.h>

%include <NMBLTK/Simulation/Control/rdControl.h>
%template(rdSetControls) rdSet<rdControl>;
%include <NMBLTK/Simulation/Control/rdControlSet.h>
%include <NMBLTK/Simulation/Control/rdControlConstant.h>
%include <NMBLTK/Simulation/Control/rdControlLinear.h>

%include <NMBLTK/Simulation/Integrator/Integrand.h>
%include <NMBLTK/Simulation/Model/rdModelIntegrand.h>
%include <NMBLTK/Simulation/Integrator/rdRKF.h>
%include <NMBLTK/Simulation/Integrator/rdIntegRKF.h>
%include <NMBLTK/Simulation/Manager/rdManager.h>
%include <NMBLTK/Simulation/Model/rdCallback.h>
%template(rdSetCallback) rdSet<rdCallback>;
%include <NMBLTK/Simulation/Model/rdCallbackSet.h>
%include <NMBLTK/Simulation/Model/rdIntegCallback.h>
%include <NMBLTK/Simulation/Simtk/rdSimtkAnimationCallback.h>
%template(rdArrayStorage) rdArrayPtrs<rdStorage>;
%include <NMBLTK/Simulation/Model/rdAnalysis.h>
%template(rdArrayAnalysis) rdArrayPtrs<rdAnalysis>;
%template(rdSetAnalysis) rdSet<rdAnalysis>;
%include <NMBLTK/Simulation/Model/rdAnalysisFactory.h>

%include <NMBLTK/Analyses/suAnalysesDLL.h>
%include <NMBLTK/Analyses/suAnalysisFactory.h>
%include <NMBLTK/Analyses/suKinematics.h>
%include <NMBLTK/Analyses/suActuation.h>
%include <NMBLTK/Analyses/suIndAcc.h>
%include <NMBLTK/Analyses/suGeneralizedForces.h>

%template(rdArrayBool) rdArray<bool>;
%template(rdArrayDouble) rdArray<double>;
%template(rdArrayInt) rdArray<int>;
%template(rdArrayStr) rdArray<std::string>;
%template(rdArrayPtrsObj) rdArrayPtrs<rdObject>;
%include <NMBLTK/Simulation/Model/suMarker.h>
%template(suSetMarkers) rdSet<suMarker>;
%include <NMBLTK/Simulation/Model/suMarkerSet.h>
%include <NMBLTK/Tools/suRange.h>
%include <NMBLTK/Tools/suScale.h>
%template(suSetScales) rdSet<suScale>;
%include <NMBLTK/Tools/suScaleSet.h>

	/* This group of headers added by KMS 3/22/06 */
%include <NMBLTK/Simulation/Model/nmblKinematicsEngine.h>
%include <NMBLTK/Simulation/SIMM/simmBody.h>
%include <NMBLTK/Simulation/SIMM/simmBone.h>
%include <NMBLTK/Simulation/SIMM/simmConstant.h>
%include <NMBLTK/Simulation/SIMM/simmCoordinate.h>
%include <NMBLTK/Simulation/SIMM/simmDof.h>
%include <NMBLTK/Simulation/SIMM/simmJoint.h>
%include <NMBLTK/Simulation/SIMM/simmKinematicsEngine.h>
%include <NMBLTK/Simulation/SIMM/simmMarker.h>
%include <NMBLTK/Simulation/SIMM/simmModel.h>
%include <NMBLTK/Simulation/SIMM/simmMuscle.h>
%include <NMBLTK/Simulation/SIMM/simmMuscleGroup.h>
%include <NMBLTK/Simulation/SIMM/simmMusclePoint.h>
%include <NMBLTK/Simulation/SIMM/simmMuscleViaPoint.h>
%include <NMBLTK/Simulation/SIMM/simmPath.h>
%include <NMBLTK/Simulation/SIMM/simmPathMatrix.h>
%include <NMBLTK/Simulation/SIMM/simmPoint.h>
%include <NMBLTK/Simulation/SIMM/simmRotationDof.h>
%include <NMBLTK/Simulation/SIMM/simmStep.h>
%include <NMBLTK/Simulation/SIMM/simmTranslationDof.h>
%include <NMBLTK/Simulation/SIMM/simmUnits.h>
