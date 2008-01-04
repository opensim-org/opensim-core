// RegisterTypes_osimCommon.cpp
// author: Frank C. Anderson
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#include <string>
#include <iostream>
#include "Object.h"
#include "VisibleObject.h"
#include "RegisterTypes_osimCommon.h"
#include "FunctionSet.h"
#include "GCVSplineSet.h"
#include "ScaleSet.h"
#include "GCVSpline.h"
#include "Transform.h"
#include "VectorGCVSplineR1R3.h"
#include "Scale.h"
#include "NatCubicSpline.h"
#include "Constant.h"
#include "StepFunction.h"
#include "LinearFunction.h"
#include "VisibleObject.h"
#include "ObjectGroup.h"



using namespace OpenSim;
using namespace std;


//_____________________________________________________________________________
/**
 * The purpose of this routine is to register all class types exported by
 * the osimCommon library.
 */
OSIMCOMMON_API void RegisterTypes_osimCommon()
{
	//cout<<"RegisterTypes_osimCommon  \n";

	Object::RegisterType( FunctionSet() );
	Object::RegisterType( GCVSplineSet() );
	Object::RegisterType( ScaleSet() );

	Object::RegisterType( GCVSpline() );
	// Object::RegisterType( Transform() );
	Object::RegisterType( VectorGCVSplineR1R3() );
	//Object::RegisterType( RootSolver() );
	Object::RegisterType( Scale() );
	Object::RegisterType( NatCubicSpline() );
	Object::RegisterType( Constant() );
	Object::RegisterType( StepFunction() );
	Object::RegisterType( LinearFunction() );
	Object::RegisterType( VisibleObject() );
	Object::RegisterType( ObjectGroup() );
}

