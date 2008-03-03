#ifndef _ContactForce_h_
#define _ContactForce_h_
// ContactForce.h
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


#include "Force.h"


//=============================================================================
//=============================================================================
/**
 * A abstract class that supports the application of contact force between
 * two bodies, BodyA and BodyB.  Actuators of this type have no states
 * and no controls.  That is, contact is assumed to be entirely passive
 * and depends only on the states of a model or models.
 *
 * @author Frank C. Anderson
 * @version 1.0
 */
namespace OpenSim { 

class OSIMSIMULATION_API ContactForce : public Force
{

//=============================================================================
// DATA
//=============================================================================
protected:
	// PROPERTIES
	/** Surface normal on BodyA expressed in the BodyA frame. */
	PropertyDblVec3 _propNormalA;
	/** Surface normal on BodyB expressed in the BodyB frame. */
	PropertyDblVec3 _propNormalB;

	// REFERENCES
	SimTK::Vec3 &_nA;
	SimTK::Vec3 &_nB;


	/** Normal displacement vector expressed in the BodyA frame. */
	SimTK::Vec3 _rnA;
	/** Normal distance from PointA to PointB. */
	double _rn;
	/** Normal velocity of PointB relative to PointA expressed in the BodyA
	frame. */
	SimTK::Vec3 _vnA;
	/** Normal speed of PointB relative to PointA. */
	double _vn;
	/** Tangent displacement unit vector expressed in the BodyA frame. */
	SimTK::Vec3 _tA;
	/** Tangential displacement vector expressed in the BodyA frame. */
	SimTK::Vec3 _rtA;
	/** Tangential distance from PointA to PointB. */
	double _rt;
	/** Tangential velocity of PointB relative to PointA expressed in the
	BodyA frame. */
	SimTK::Vec3 _vtA;
	// FORCE GEOMETRY
	/** Total normal force magnitude. */
	double _fnMag;
	/** Elastic normal force applied to BodyB expressed in the local frame
	of BodyA. */
	SimTK::Vec3 _fnp;
	/** Viscous normal force applied to BodyB expressed in the local frame
	of BodyA. */
	SimTK::Vec3 _fnv;
	/** Total ormal force applied to BodyB expressed in the local frame of BodyA. */
	SimTK::Vec3 _fn;
	/** Total tangential force magnitude corrected to be consistent with
	friction constraints. */
	double _ftMag;
	/** Elastic tangential force NOT corrected to enforce friction
	constraints expressed in the local frame of BodyA. */
	SimTK::Vec3 _ftp;
	/** Viscous tangential force NOT corrected to enforce friction
	constraints expressed in the local frame of BodyA. */
	SimTK::Vec3 _ftv;
	/** Total tangential force applied to BodyB expressed in the local frame
	of BodyA.  Note that _ftA is not necessarily in the same direction as _tA
	because of the viscosity component. */
	SimTK::Vec3 _ft;
	/** Correction in the spring force in order to enforce friction
	constraints.  It is the change made to the force applied to
   BodyA and is expressed in the local frame of BodyA. */
	SimTK::Vec3 _dfFric;
	/** Pointer to a vector function that contains the velocity of _pA,
	expressed in the inertial frame, as a function of time. */
	VectorFunction *_vAFunction;
	/** Pointer to a vector function that contains the velocity of _pB,
	expressed in the inertial frame, as a function of time. */
	VectorFunction *_vBFunction;


//=============================================================================
// METHODS
//=============================================================================
	//--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------
public:
	ContactForce(std::string aBodyA="",std::string aBodyB="");
	ContactForce(const ContactForce &aForce);
	virtual ~ContactForce();
private:
	void setNull();
	void setupProperties();
	

	//--------------------------------------------------------------------------
	// OPERATORS
	//--------------------------------------------------------------------------
public:
	ContactForce& operator=(const ContactForce &aContact);

	//--------------------------------------------------------------------------
	// GET AND SET
	//--------------------------------------------------------------------------
public:
	// NORMAL ON BODY A
	void setNormalA(const SimTK::Vec3& aNormal);
	void getNormalA(SimTK::Vec3& rNormal) const;
	// NORMAL ON BODY B
	void setNormalB(const SimTK::Vec3& aNormal);
	void getNormalB(SimTK::Vec3& rNormal) const;
	// NORMAL DISPLACEMENT VECTOR
	void getNormalDisplacement(SimTK::Vec3& rDisplacement) const;
	// NORMAL DISTANCE
	double getNormalDistance() const;
	// NORMAL VELOCITY
	void getNormalVelocity(SimTK::Vec3& rVelocity) const;
	// NORMAL SPEED
	double getNormalSpeed() const;
	// TANGENT ON BODY A
	void getTangent(SimTK::Vec3& rTangent) const;
	// TANGENTIAL DISPLACEMENT VECTOR
	void getTangentialDisplacement(SimTK::Vec3& rDisplacement) const;
	// TANGENTIAL DISTANCE
	double getTangentialDistance() const;
	// TANGENTIAL VELOCITY
	void getTangentialVelocity(SimTK::Vec3& rVelocity) const;
	// FORCE
	void getNormalForce(SimTK::Vec3& rFP,SimTK::Vec3& rFV,SimTK::Vec3& rF) const;
	void getTangentialForce(SimTK::Vec3& rFP,SimTK::Vec3& rFV,SimTK::Vec3& rF) const;
	// TANGENTIAL IMPEDANCE
	virtual double getInstantaneousTangentialStiffness() const;
	virtual double getInstantaneousTangentialViscosity() const;
	// NORMAL IMPEDANCE
	virtual double getInstantaneousNormalStiffness() const;
	virtual double getInstantaneousNormalViscosity() const;
	// FRICTION CORRECTION
	void getFrictionCorrection(SimTK::Vec3& rDF) const;
	// POINT A VELOCITY FUNCTION
	void setVelPointAFunction(VectorFunction* aVectorFunction);
	const VectorFunction* getVelPointAFunction() const;
	// POINT B VELOCITY FUNCTION
	void setVelPointBFunction(VectorFunction* aVectorFunction);
	const VectorFunction* getVelPointBFunction() const;



	//--------------------------------------------------------------------------
	// COMPUTATIONS
	//--------------------------------------------------------------------------
	virtual void updatePseudoStates();
	virtual void computeDisplacements();
	virtual void computeVelocities();

	//--------------------------------------------------------------------------
	// APPLICATION
	//--------------------------------------------------------------------------

	//--------------------------------------------------------------------------
	// CHECK
	//--------------------------------------------------------------------------
	virtual bool check() const;

	//--------------------------------------------------------------------------
	// UTILITY
	//--------------------------------------------------------------------------
	virtual void
		computeLineOfActionComponents(SimTK::Vec3& rNormal,
		SimTK::Vec3& rTangential) const;

	//--------------------------------------------------------------------------
	// XML
	//--------------------------------------------------------------------------
	virtual void updateFromXMLNode();

	OPENSIM_DECLARE_DERIVED(ContactForce, AbstractActuator);

//=============================================================================
};	// END of class ContactForce

}; //namespace
//=============================================================================
//=============================================================================

#endif // __ContactForce_h__


