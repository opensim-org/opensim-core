%module(directors="1") opensimJAM

#pragma SWIG nowarn=822,451,503,516,325,401

%{
#include <Bindings/OpenSimHeaders_common.h>
#include <Bindings/OpenSimHeaders_simulation.h>
#include <Bindings/OpenSimHeaders_actuators.h>
#include <Bindings/OpenSimHeaders_analyses.h>
#include <Bindings/OpenSimHeaders_tools.h>
#include <Bindings/OpenSimHeaders_jam.h>

using namespace OpenSim;
using namespace SimTK;
%}

%include "java_preliminaries.i";

%import "arrays_java.i";
%import <java_common.i>
%import <java_simulation.i>
%import <java_actuators.i>

%include <Bindings/jam.i>


