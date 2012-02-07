#ifndef _CoordinateLimitForce_h_
#define _CoordinateLimitForce_h_
// CoordinateLimitForce.h
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//	AUTHOR: Ajay Seth
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
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


//=============================================================================
// INCLUDES
//=============================================================================
#include "osimActuatorsDLL.h"
#include <OpenSim/Common/PropertyDbl.h>
#include <OpenSim/Common/PropertyStr.h>
#include <OpenSim/Simulation/Model/Force.h>


//=============================================================================
//=============================================================================
/**
 * Generate a force that acts to limit the range of motion of a coordinate
 * Force experienced at upper and lower limits of the coordinate value 
 * is according to a linear siffneses K_upper and K_lower, with a C2 continuos
 * transition from 0 to K. The transition parameter defines how far beyond the 
 * limit the stiffness becomes purely linear. The integrator will like smoother
 * (i.e. larger transition regions).
 *
 * Damping factor is also phased in through the transiton region from 0 to the
 * to the value provided.
 *
 * Limiting force is guaranteed to be zero within the upper and lower limits.
 *
 * @author Ajay Seth
 * @version 2.0
 */
namespace OpenSim { 

class OSIMACTUATORS_API CoordinateLimitForce : public Force 
{
//=============================================================================
// DATA
//=============================================================================
protected:
	/** Corresponding generalized coordinate to which the coordinate actuator
    is applied. */
    mutable Coordinate *_coord;

private:

	SimTK::Function::Step *upStep;
	SimTK::Function::Step *loStep;

	// Scaling of coordinate value to strain in length or radians 
	double _w;

//=============================================================================
// METHODS
//=============================================================================
public:
	// Default constructor
	CoordinateLimitForce();
	// Convenience constructor using API
	/**
	 * Generate a force that acts to limit the range of motion of a coordinate
	 * Force experienced at upper and lower limits of the coordinate (q) value 
	 * is according to a linear siffneses K_upper and K_lower, with a C2 continuos
	 * transition from 0 to K. The transition parameter (dq) defines how far beyond the 
	 * limit the stiffness becomes purely linear. The integrator will like smoother
	 * (i.e. larger transition regions). */
	CoordinateLimitForce(const std::string &coordName, double q_upper, double K_upper, 
		double q_lower, double K_lower, double damping, double dq); 
	// Copy constructor
	CoordinateLimitForce(const CoordinateLimitForce &aForce);
	virtual ~CoordinateLimitForce();
	virtual Object* copy() const;
private:
	void setNull();
	void setupProperties();
	void copyData(const CoordinateLimitForce &aForce);

public:

	//--------------------------------------------------------------------------
	// OPERATORS
	//--------------------------------------------------------------------------
	CoordinateLimitForce&
		operator=(const CoordinateLimitForce &aForce);

	//--------------------------------------------------------------------------
	// GET AND SET
	//--------------------------------------------------------------------------
	// Parameters
	void setUpperStiffness(double aUpperStiffness);
	double getUpperStiffness() const;
	
	void setUpperLimit(double aUpperLimit);
	double getUpperLimit() const;
	
	void setLowerStiffness(double aLowerStiffness);
	double getLowerStiffness() const;
	
	void setLowerLimit(double aLowerLimit);
	double getLowerLimit() const;
	
	void setOffset(double aOffset);
	double getOffset() const;
	
	void setDamping(double aDamping);
	double getDamping() const;

	void setTransition(double aTransition);
	//--------------------------------------------------------------------------
	// COMPUTATIONS
	//--------------------------------------------------------------------------
	virtual void computeForce( const SimTK::State& s, 
							   SimTK::Vector_<SimTK::SpatialVec>& bodyForces, 
							   SimTK::Vector& mobilityForces) const;

	// Force calculation operator
	double calcLimitForce( const SimTK::State& s) const;

	//--------------------------------------------------------------------------
	// REPORTING
	//--------------------------------------------------------------------------
	/** 
	 * Methods to query a Force for the value actually applied during simulation
	 * The names of the quantities (column labels) is returned by this first function
	 * getRecordLabels()
	 */
	virtual Array<std::string> getRecordLabels() const ;
	/**
	 * Given SimTK::State object extract all the values necessary to report forces, application location
	 * frame, etc. used in conjunction with getRecordLabels and should return same size Array
	 */
	virtual Array<double> getRecordValues(const SimTK::State& state) const ;

	//--------------------------------------------------------------------------
	// Model Component
	//--------------------------------------------------------------------------
	virtual void setup(Model& aModel);

	OPENSIM_DECLARE_DERIVED(CoordinateLimitForce,Force);

//=============================================================================
};	// END of class CoordinateLimitForce

}; //namespace
//=============================================================================
//=============================================================================


#endif // #ifndef __CoordinateLimitForce_h__
