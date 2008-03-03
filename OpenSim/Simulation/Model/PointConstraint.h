#ifndef _PointConstraint_h_
#define _PointConstraint_h_
// PointConstraint.h
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//	AUTHOR: Frank C. Anderson
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
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
