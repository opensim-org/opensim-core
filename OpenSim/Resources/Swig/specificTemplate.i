%module _MODEL_NAME_HERE_Module
%{
#include <NMBLTK/Tools/rdToolsDLL.h>
#include <NMBLTK/Simulation/rdSimulationDLL.h>
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
#include <NMBLTK/Simulation/SDFast/rdSDFastDLL.h>
#include <NMBLTK/Simulation/SDFast/rdSDFast.h>
#include <NMBLTK/Simulation/SDFast/rdActuatedModel_SDFast.h>
#include <NMBLTK/Models/SIMMPipeline/4.0/suPipeline40/suPipeline40.h>
#include <NMBLTK/Models/SIMMPipeline/4.0/Pipeline40Workflow/suPipeline40Workflow.h>
#include "_MODEL_NAME_HERE_.h"
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
/* rest of header files to be wrapped */
%include <NMBLTK/Tools/rdToolsDLL.h>
%include <NMBLTK/Simulation/rdSimulationDLL.h>
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

%template(rdArrayBool) rdArray<bool>;
%template(rdArrayDouble) rdArray<double>;
%template(rdArrayInt) rdArray<int>;
%template(rdArrayStr) rdArray<std::string>;
%template(rdArrayPtrsObj) rdArrayPtrs<rdObject>;

%include <NMBLTK/Simulation/SDFast/rdSDFastDLL.h>
%include <NMBLTK/Simulation/SDFast/rdSDFast.h>
%include <NMBLTK/Simulation/SDFast/rdActuatedModel_SDFast.h>
%include <NMBLTK/Models/SIMMPipeline/4.0/suPipeline40/suPipeline40.h>
%include <NMBLTK/Models/SIMMPipeline/4.0/Pipeline40Workflow/suPipeline40Workflow.h>
%include "_MODEL_NAME_HERE_.h"
