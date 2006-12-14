#ifndef _DecompHardUTWalkNoComp_h_
#define _DecompHardUTWalkNoComp_h_
// DecompHardUTWalkNoComp.h
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//	AUTHOR: Frank C. Anderson
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


//=============================================================================
// INCLUDES
//=============================================================================
#include <OpenSim/Tools/rdTools.h>
#include <OpenSim/Models/UTWalking8/rdUTWalking8.h>
#include <OpenSim/Tools/BodyConstraint.h>
#include "Analyses.h"
#include "DecompNoComp.h"


//=============================================================================
//=============================================================================
/**
 * A class for computing a decomposition by hard constraints for the
 * UT Walking Model or classes derived from the UT Walking Model.
 */
namespace OpenSim { 

class SUANALYSES_API DecompHardUTWalkNoComp : public DecompNoComp
{
//=============================================================================
// DATA
//=============================================================================
public:
	/** Flag to set whether contact will be prescribed */
	bool _prescribedContact;
	/** Storage for the prescribed spring positions */
	Storage *_prescribedSpringPositionStorage;
	/** Storage for the prescribed spring velocities */
	Storage *_prescribedSpringVelocityStorage;
	/** Storage for the prescribed spring forces */
	Storage *_prescribedSpringForceStorage;

protected:
	/** Force component index. */
	int _c;
	/** Number of spring points in contact. */
	int _nxs;
	/** Map from spring points in contact to springs in the model. */
	int *_xsSprMap;
	/** Map from degrees of freedom (XYZ) of springs in contact to
	degrees of freedom of springs in the model. */
	int *_xsXYZMap;
	/** Constraints on the bodies: rhindfoot, rtoes, lhindfoot, ltoes. */
	BodyConstraint _bc[4];
	/** Performance criterion weight for the spring forces. */
	double _wXS;
	/** Performance criterion weight for the spring accelerations. */
	double _wAcc;
private:
	/** Point used for temporary storage of model spring point locations.
	See getContactPoint(). */
	double _point[3];
	/** Untouched copy of the states. */
	double *_yCopy;
	/** Manipulated copy of the states. */
	double *_yTmp;
	/** Untouched copy of the model controls. */
	double *_x;

//=============================================================================
// METHODS
//=============================================================================
public:
	DecompHardUTWalkNoComp(rdUTWalking8 *aModel);
	virtual ~DecompHardUTWalkNoComp();
private:
	void setNull();

	//--------------------------------------------------------------------------
	// GET AND SET
	//--------------------------------------------------------------------------
private:
	double* getContactPoint(int aIndex);
public:
	void setPrescribedContact(bool aTrueFalse, 
		Storage *aPrescribedSpringPositionStorage,
		Storage *aPrescribedSpringVelocityStorage,
		Storage *aPrescribedSpringForceStorage);
	bool getPrescribedContact() const;
	Storage* getPrescribedSpringPositionStorage();
	Storage* getPrescribedSpringVelocityStorage();
	Storage* getPrescribedSpringForceStorage();

	//--------------------------------------------------------------------------
	// DECOMPOSITION
	//--------------------------------------------------------------------------
public:
	virtual void
		compute(double *aXPrev,double *aYPrev,int aStep,
		double aDT,double aT,double *aX,double *aY);
	int suComputePerformance(double *x,double *p);
	int suComputePerformanceGradient(double *x,double *dpdx);
	int suComputeConstraint(double *x,int ic,double *c);
	int suComputeContactPointAccelerations(double *x,int c,double cpa[][3]);
protected:
	int getNumberConstraints();
	void clearBodyConstraints();
	void clearBodyConstraintValues();
	void setBodyConstraintValues(int aN,int aID[],double aAcc[][3]);
	void decompose(int step,double dt,double t,double *xt,double *y,void *cd);
	void determineControls(double sfrc[][3]);
	void determineConstraints();
	void setSpringForces(double *aXS,double aFS[][3]);
	void applyComponentForce(int aC);

//=============================================================================
};	// END of class DecompHardUTWalkNoComp

}; //namespace
//=============================================================================
//=============================================================================


#endif // #ifndef __DecompHardUTWalkNoComp_h__
