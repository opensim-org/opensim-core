#ifndef _ModelTestSuite_h_
#define _ModelTestSuite_h_
// ModelTestSuite.h
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


