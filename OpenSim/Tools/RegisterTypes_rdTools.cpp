// RegisterTypes_rdTools.cpp
// author: Frank C. Anderson
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#include <string>
#include <iostream>
#include <OpenSim/Tools/Object.h>
#include <OpenSim/Tools/VisibleObject.h>
#include "RegisterTypes_rdTools.h"
#include "FunctionSet.h"
#include "GCVSplineSet.h"
#include "ScaleSet.h"
#include "GCVSpline.h"
//#include "Transform.h"
#include "VectorGCVSplineR1R3.h"
#include "Scale.h"
#include "NatCubicSpline.h"
#include "Constant.h"
#include "VisibleObject.h"



using namespace OpenSim;
using namespace std;


//_____________________________________________________________________________
/**
 * The purpose of this routine is to register all class types exported by
 * the rdTools library.
 */
RDTOOLS_API void RegisterTypes_rdTools()
{
	//cout<<"RegisterTypes_rdTools  \n";

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
	Object::RegisterType( VisibleObject() );
}

