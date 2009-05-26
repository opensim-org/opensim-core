// RegisterTypes_SimbodyEngine.cpp
// author: Frank C. Anderson
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
/*
* Copyright (c)  2005, Stanford University. All rights reserved. 
* Use of the OpenSim software in source form is permitted provided that the following
* conditions are met:
* 	1. The software is used only for non-commercial research and education. It may not
*     be used in relation to any commercial activity.
* 	2. The software is not distributed or redistributed.  Software distribution is allowed 
*     only through https://simtk.org/home/opensim.
* 	3. Use of the OpenSim software or derivatives must be acknowledged in all publications,
*      presentations, or documents describing work in which OpenSim or derivatives are used.
* 	4. Credits to developers may not be removed from executables
*     created from modifications of the source.
* 	5. Modifications of source code must retain the above copyright notice, this list of
*     conditions and the following disclaimer. 
* 
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
*  EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
*  OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
*  SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
*  TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; 
*  HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
*  OR BUSINESS INTERRUPTION) OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY
*  WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <string>
#include <iostream>
#include <OpenSim/Common/Object.h>
#include "RegisterTypes_osimSimbodyEngine.h"
#include "SimbodyEngine.h"
#include "Body.h"
#include "Constraint.h"
#include "WeldConstraint.h"
#include "CoordinateCouplerConstraint.h"
#include "UnilateralConstraint.h"
#include "RollingOnSurfaceConstraint.h"
#include "CustomJoint.h"
//#include "EllipsoidJoint.h"
#include "WeldJoint.h"
#include "TransformAxis.h"
#include "Coordinate.h"
#include "Speed.h"
#include "SimbodyEngine01_05.h"
#include "SimbodyBody01_05.h"
#include "SimbodyJoint01_05.h"
#include "SimbodyRotationDof01_05.h"
#include "SimbodyTranslationDof01_05.h"
#include "SimbodyCoordinate01_05.h"
#include "SimbodySpeed01_05.h"


using namespace std;
using namespace OpenSim;

static osimSimbodyEngineInstantiator instantiator; 

//_____________________________________________________________________________
/**
 * The purpose of this routine is to register all class types exported by
 * the rdSimulation library.
 */
OSIMSIMBODYENGINE_API void RegisterTypes_SimbodyEngine()
{
	cout<<"RegisterTypes_SimbodyEngine\n";

	// CURRENT RELEASE
	Object::RegisterType( SimbodyEngine() );
	Object::RegisterType( Body() );
	Object::RegisterType( Constraint() );
	Object::RegisterType( WeldConstraint() );
	Object::RegisterType( CoordinateCouplerConstraint() );
	//Object::RegisterType( UnilateralConstraint() );
	Object::RegisterType( RollingOnSurfaceConstraint() );
	Object::RegisterType( CustomJoint() );
	Object::RegisterType( WeldJoint() );
	//Object::RegisterType( EllipsoidJoint() );
	Object::RegisterType( TransformAxis() );
	Object::RegisterType( Coordinate() );
	Object::RegisterType( Speed() );

	// RELEASE 1.5
	Object::RegisterType( SimbodyEngine01_05() );
	Object::RegisterType( SimbodyBody01_05() );
	Object::RegisterType( SimbodyJoint01_05() );
	Object::RegisterType( SimbodyCoordinate01_05() );
	Object::RegisterType( SimbodySpeed01_05() );
	Object::RegisterType( SimbodyRotationDof01_05() );
	Object::RegisterType( SimbodyTranslationDof01_05() );

}

osimSimbodyEngineInstantiator::osimSimbodyEngineInstantiator() 
{ 
        registerDllClasses(); 
} 
    
void osimSimbodyEngineInstantiator::registerDllClasses() 
{ 
        RegisterTypes_SimbodyEngine(); 
} 
    
