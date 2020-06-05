%module(directors="1") opensimActuatorsAnalysesTools
%module opensimActuatorsAnalysesTools

#pragma SWIG nowarn=822,451,503,516,325,401

%{
#include <Bindings/OpenSimHeaders_common.h>
#include <Bindings/OpenSimHeaders_simulation.h>
#include <Bindings/OpenSimHeaders_actuators.h>
#include <Bindings/OpenSimHeaders_analyses.h>
#include <Bindings/OpenSimHeaders_tools.h>

#include <Bindings/Java/OpenSimJNI/OpenSimContext.h>

using namespace OpenSim;
using namespace SimTK;
%}

%feature("director") OpenSim::AnalysisWrapper;
%feature("director") OpenSim::SimtkLogCallback;

%include "java_preliminaries.i";

%include "arrays_java.i";


%import "java_simulation.i"

%include <Bindings/actuators.i>
%include <Bindings/analyses.i>
        
// When used from GUI or matlab ModelScaler takes ownership on these calls, 
// communicate that fact to the interpreter to avoid Garbage Collection issues
%javamethodmodifiers OpenSim::ModelScaler::addMeasurement "private";
%javamethodmodifiers OpenSim::ModelScaler::addScale "private";

%rename OpenSim::ModelScaler::addMeasurement private_addMeasurement;
%rename OpenSim::ModelScaler::addScale private_addScale;

%typemap(javacode) OpenSim::ModelScaler %{
    public void addScale(Scale scale){
        scale.markAdopted();
        private_addScale(scale);
    }

    public void addMeasurement(Measurement meas){
        meas.markAdopted();
        private_addMeasurement(meas);
    }
%}

%include <Bindings/tools.i>
%include <Bindings/Java/OpenSimJNI/OpenSimContext.h>

