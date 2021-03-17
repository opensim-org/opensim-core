%module(directors="1") opensimJAM

#pragma SWIG nowarn=822,451,503,516,325,401

%{
#include <Bindings/OpenSimHeaders_common.h>
#include <Bindings/OpenSimHeaders_simulation.h>
#include <Bindings/OpenSimHeaders_actuators.h>
#include <Bindings/OpenSimHeaders_analyses.h>
#include <Bindings/OpenSimHeaders_tools.h>
#include <Bindings/OpenSimHeaders_moco.h>
#include <Bindings/OpenSimHeaders_jam.h>

#include <Bindings/Java/OpenSimJNI/OpenSimContext.h>

using namespace OpenSim;
using namespace SimTK;
%}
%include <Bindings/preliminaries.i>
%include "java_preliminaries.i";
%include <Bindings/common.i>
%include "java_common.i";

%include <Bindings/jam.i>


