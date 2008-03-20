#ifndef _BodyConstraint_h_
#define _BodyConstraint_h_
// BodyConstraint.h
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//	AUTHOR: Frank C. Anderson
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


//==============================================================================
// INCLUDES
//==============================================================================
#include <OpenSim/Simulation/osimSimulationDLL.h>
#include "PointConstraint.h"
#include "AbstractBody.h"

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
/**
 * A class for specifiying point constraints on a body.
 *
 * @author Frank C. Anderson
 * @version 1.0
 */
namespace OpenSim {
class OSIMSIMULATION_API BodyConstraint
{
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//==============================================================================
// DATA
//==============================================================================
public:
	AbstractBody *_body;							// BODY ID
	OpenSim::PointConstraint _pc[3];	// POINT CONSTRAINTS (3 MAX, for now)

//==============================================================================
// METHODS
//==============================================================================
public:
	//---------------------------------------------------------------------------
	// CONSTRUCTION
	//---------------------------------------------------------------------------
	virtual ~BodyConstraint();
	BodyConstraint();

	//---------------------------------------------------------------------------
	// SET AND GET
	//---------------------------------------------------------------------------
	int getNC();
	void setBody(OpenSim::AbstractBody *aBody);
	OpenSim::AbstractBody* getBody();
	OpenSim::PointConstraint* getPC(int aI);

	//---------------------------------------------------------------------------
	// CONSTRAINT CONSTRUCTION
	//---------------------------------------------------------------------------
	void setValues(int aN,int aID[],double aV[][3]);
	void constructConstraintsForPoint1();
	void constructConstraintsForPoint2();
	double* findMostOrthogonal(OpenSim::PointConstraint *aPC,SimTK::Vec3& aV);

	//---------------------------------------------------------------------------
	// UTILITY
	//---------------------------------------------------------------------------
	void clear();
	void clearValues();

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
};	// END class BodyConstraint
}; //namespace
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


#endif // #ifndef __BodyConstraint_h__
