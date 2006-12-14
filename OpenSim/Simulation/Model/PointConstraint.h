#ifndef _PointConstraint_h_
#define _PointConstraint_h_
// PointConstraint.h
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//	AUTHOR: Frank C. Anderson
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#include <OpenSim/Simulation/rdSimulationDLL.h>


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
class RDSIMULATION_API PointConstraint
{
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//==============================================================================
// DATA
//==============================================================================
public:
	int _id;				// CONTACT POINT ID
	double _p[3];		// POINT
	double _v[3];		// VALUE
	double _c0[3];		// CONSTRAINT DIRECTION 0
	double _c1[3];		// CONSTRAINT DIRECTION 1
	double _c2[3];		// CONSTRAINT DIRECTION 2

//==============================================================================
// METHODS
//==============================================================================
public:
	//---------------------------------------------------------------------------
	// CONSTRUCTION
	//---------------------------------------------------------------------------
	virtual ~PointConstraint();
	PointConstraint(int aID=0);
	PointConstraint(double aP[3],double aV[3],
		double aC0[3],double aC1[3],double aC2[3],int aID=0);

	//---------------------------------------------------------------------------
	// SET AND GET
	//---------------------------------------------------------------------------
	int getNC();
	void setID(int aID);
	int getID();
	void setPoint(double aP[3]);
	void setPoint(double aP0,double aP1,double aP2);
	void getPoint(double aP[3]);
	double* getPoint();
	void setValue(double aP[3]);
	void setValue(double aP0,double aP1,double aP2);
	void getValue(double aP[3]);
	double* getValue();
	void setC0(double aC[3]);
	void setC0(double aC0,double aC1,double aC2);
	void getC0(double *aC);
	double* getC0();
	void setC1(double aC[3]);
	void setC1(double aC0,double aC1,double aC2);
	void getC1(double *aC);
	double* getC1();
	void setC2(double aC[3]);
	void setC2(double aC0,double aC1,double aC2);
	void getC2(double *aC);
	double* getC2();

	//---------------------------------------------------------------------------
	// EVALUATE
	//---------------------------------------------------------------------------
	double evaluateC0(double aV[3]);
	double evaluateC1(double aV[3]);
	double evaluateC2(double aV[3]);

	//---------------------------------------------------------------------------
	// UTILITY
	//---------------------------------------------------------------------------
	void constructOrthoNormalConstraints(double aV[3],double aC0[3]);
	void constructOrthoNormalC1(double aV[3]);
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
