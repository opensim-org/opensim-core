//%module _MODEL_NAME_HERE_Module
%module Model
%{
#include "rdSimulationDLL.h"
#include "rdToolsDLL.h"
#include "rdObject.h"
#include "rdVisibleProperties.h"
#include "rdTransform.h"
#include "rdVisibleObject.h"
#include "rdBody.h"
#include "rdModel.h"
//#include "rdSDFastDLL.h"
//#include "rdSDFast.h"
//#include "rdActuatedModel_SDFast.h"
//#include "_MODEL_NAME_HERE_.h"
%}

/* This file is for creation/handling of arrays */
%include "sarrays.i"

/* This interface file is for better handling of pointers and references */
%include "typemaps.i"

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
%include "rdSimulationDLL.h"
%include "rdToolsDLL.h"
%include "rdObject.h"
%include "rdVisibleProperties.h"
%include "rdTransform.h"
%include "rdVisibleObject.h"
%include "rdBody.h"
%include "rdModel.h"
//%include "rdSDFastDLL.h"
//%include "rdSDFast.h"
//%include "rdActuatedModel_SDFast.h"
//%include "_MODEL_NAME_HERE_.h"
