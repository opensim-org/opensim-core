#ifndef _ModelTestSuite_h_
#define _ModelTestSuite_h_
// ModelTestSuite.h
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
/*
* Copyright (c) 2005, Stanford University. All rights reserved. 
* Redistribution and use in source and binary forms, with or without 
* modification, are permitted provided that the following conditions
* are met: 
*  - Redistributions of source code must retain the above copyright 
*    notice, this list of conditions and the following disclaimer. 
*  - Redistributions in binary form must reproduce the above copyright 
*    notice, this list of conditions and the following disclaimer in the 
*    documentation and/or other materials provided with the distribution. 
*  - Neither the name of the Stanford University nor the names of its 
*    contributors may be used to endorse or promote products derived 
*    from this software without specific prior written permission. 
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS 
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE 
* COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, 
* BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; 
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER 
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
* LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN 
* ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE 
* POSSIBILITY OF SUCH DAMAGE. 
*/

/* Note: This code was originally developed by Realistic Dynamics Inc. 
 * Author: Frank C. Anderson 
 */


//============================================================================
// INCLUDE
//============================================================================
#include "rdModelTestSuiteDLL.h"
#include <OpenSim/Simulation/Model/Model.h>


//=============================================================================
//=============================================================================
/**
 * A class for testing the basic functionality of an Model instance.
 *
 * @author Frank C. Anderson, James M. Ziegler
 * @version 1.0
 */
namespace OpenSim { 

class RDMODELTESTSUITE_API ModelTestSuite  
{
//=============================================================================
// DATA
//=============================================================================
private:
	/** File pointer to the output file. */
	FILE *_outFPT;


//=============================================================================
// METHODS
//=============================================================================
public:
	//--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------
	ModelTestSuite();


	//--------------------------------------------------------------------------
	// MODEL
	//--------------------------------------------------------------------------
	bool Test(Model *aModel);
	bool TestNumbers(Model *aModel);
	bool TestNames(Model *aModel);
	bool TestStates(Model *aModel);
	bool TestPseudoStates(Model *aModel);
	bool TestGravity(Model *aModel);
	bool TestBodies(Model *aModel);
	bool TestKinematics(Model *aModel);
	bool TestLoads(Model *aModel);
	bool TestDerivatives(Model *aModel);
	bool TestMassMatrix(Model *aModel);
	bool TestJacobian(Model *aModel);
	bool TestOrientationUtilities(Model *aModel);
	bool TestContact(Model *aModel);
	bool TestActuation(Model *aModel);

	// CONTACTS
	bool TestContacts(Model *aModel);

	// ACTUATORS
	bool TestActuators(Model *aModel);


//=============================================================================
};	// END of class ModelTestSuite

}; //namespace
//=============================================================================
//=============================================================================

#endif // __ModelTestSuite_h__


