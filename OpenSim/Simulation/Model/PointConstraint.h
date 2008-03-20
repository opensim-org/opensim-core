#ifndef _PointConstraint_h_
#define _PointConstraint_h_
// PointConstraint.h
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
#ifdef WIN32
#pragma warning( disable : 4251 )
#endif

#include <OpenSim/Simulation/osimSimulationDLL.h>


//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
/**
 * A class for specifiying constraints at a point.
 *
 * The class supports constraining a point in up to 3 non-parallel directions.
 *
 * Normally it is best to specify constraint directions as unit vectors;
 * however, it is also allowable for the constraint direction vectors to have
 * any magnitude.  The magnitude acts a weight.  When the magnitude of a
 * constraint direction vector is 0.0 there is essentially no constraint on
 * the point specified by that constraint direction vector.
 *
 * @author Frank C. Anderson
 * @version 1.0
 */
namespace OpenSim { 
class OSIMSIMULATION_API PointConstraint
{
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//==============================================================================
// DATA
//==============================================================================
public:
	int _id;				// CONTACT POINT ID
	SimTK::Vec3 _p;		// POINT
	SimTK::Vec3 _v;		// VALUE
	SimTK::Vec3 _c0;		// CONSTRAINT DIRECTION 0
	SimTK::Vec3 _c1;		// CONSTRAINT DIRECTION 1
	SimTK::Vec3 _c2;		// CONSTRAINT DIRECTION 2

//==============================================================================
// METHODS
//==============================================================================
public:
	//---------------------------------------------------------------------------
	// CONSTRUCTION
	//---------------------------------------------------------------------------
	virtual ~PointConstraint();
	PointConstraint(int aID=0);
	PointConstraint(SimTK::Vec3& aP,SimTK::Vec3& aV,
		SimTK::Vec3& aC0,SimTK::Vec3& aC1,SimTK::Vec3& aC2,int aID=0);

	//---------------------------------------------------------------------------
	// SET AND GET
	//---------------------------------------------------------------------------
	int getNC();
	void setID(int aID);
	int getID();
	void setPoint(const SimTK::Vec3& aP);
	void setPoint(double aP0,double aP1,double aP2);
	void getPoint(SimTK::Vec3& aP);
	SimTK::Vec3& getPoint();
	void setValue(const SimTK::Vec3& aP);
	void setValue(double aP0,double aP1,double aP2);
	void getValue(SimTK::Vec3& aP);
	SimTK::Vec3& getValue();
	void setC0(const SimTK::Vec3& aC);
	void setC0(double aC0,double aC1,double aC2);
	void getC0(SimTK::Vec3& aC);
	SimTK::Vec3& getC0();
	void setC1(const SimTK::Vec3& aC);
	void setC1(double aC0,double aC1,double aC2);
	void getC1(SimTK::Vec3& aC);
	SimTK::Vec3& getC1();
	void setC2(const SimTK::Vec3& aC);
	void setC2(double aC0,double aC1,double aC2);
	void getC2(SimTK::Vec3& aC);
	SimTK::Vec3& getC2();

	//---------------------------------------------------------------------------
	// EVALUATE
	//---------------------------------------------------------------------------
	double evaluateC0(SimTK::Vec3& aV);
	double evaluateC1(SimTK::Vec3& aV);
	double evaluateC2(SimTK::Vec3& aV);

	//---------------------------------------------------------------------------
	// UTILITY
	//---------------------------------------------------------------------------
	void constructOrthoNormalConstraints(SimTK::Vec3& aV,SimTK::Vec3& aC0);
	void constructOrthoNormalC1(SimTK::Vec3& aV);
	void constructOrthoNormalC2();
	void normalizeConstraints();
	void zeroConstraints();
	void clear();

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
};	// END class PointConstraint
}; //namespace
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


#endif // #ifndef __PointConstraint_h__
