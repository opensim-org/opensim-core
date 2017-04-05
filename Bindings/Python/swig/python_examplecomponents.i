%module(directors="1") examplecomponents
#pragma SWIG nowarn=822,451,503,516,325, 401

%{
#define SWIG_FILE_WITH_INIT
#include <Bindings/OpenSimHeaders_common.h>
#include <Bindings/OpenSimHeaders_simulation.h>
#include <Bindings/OpenSimHeaders_actuators.h>
#include <Bindings/OpenSimHeaders_examplecomponents.h>

using namespace OpenSim;
using namespace SimTK;
%}

%import "python_actuators.i"

%include <Bindings/examplecomponents.i>
