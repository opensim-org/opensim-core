%module(directors="1") opensimModelSimulation2
%module opensimModelSimulation2

#pragma SWIG nowarn=822,451,503,516,325,401

%{
#include <Bindings/OpenSimHeaders_common.h>
#include <Bindings/OpenSimHeaders_simulation.h>
#include <Bindings/Java/OpenSimJNI/OpenSimContext.h>

using namespace OpenSim;
using namespace SimTK;
%}

%include "arrays_java.i";


%import "java_simulation1.i"

%include <Bindings/simulation2.i>
