%module(directors="1") opensimWrapsAnalysesTools
%module opensimWrapsAnalysesTools

#pragma SWIG nowarn=822,451,503,516,325,401

%{
#include <Bindings/OpenSimHeaders_common.h>
#include <Bindings/OpenSimHeaders_simulation.h>
#include <Bindings/OpenSimHeaders_wraps.h>
#include <Bindings/OpenSimHeaders_analyses.h>
#include <Bindings/OpenSimHeaders_tools.h>
#include <OpenSim/Utilities/simmFileWriterDLL/SimmFileWriter.h>

#include <Bindings/Java/OpenSimJNI/Hooks/SimtkLogCallback.h>
#include <Bindings/Java/OpenSimJNI/OpenSimContext.h>

using namespace OpenSim;
using namespace SimTK;
%}


%include "java_preliminaries.i";

%include "arrays_java.i";

%include <Bindings/wraps.i>
%include <Bindings/analyses.i>
%include <Bindings/tools.i>
%include <OpenSim/Utilities/simmFileWriterDLL/SimmFileWriter.h>

%include <Bindings/Java/OpenSimJNI/OpenSimContext.h>

%include <Bindings/Java/OpenSimJNI/Hooks/SimtkLogCallback.h>

