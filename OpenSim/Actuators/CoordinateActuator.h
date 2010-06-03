#ifndef _CoordinateActuator_h_
#define _CoordinateActuator_h_
// CoordinateActuator.h
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


#include <OpenSim/Actuators/osimActuatorsDLL.h>
#include <OpenSim/Common/PropertyStr.h>
#include <OpenSim/Common/PropertyDbl.h>
#include <OpenSim/Simulation/Model/CustomActuator.h>
#include "SimTKsimbody.h"


//=============================================================================
//=============================================================================
/**
 * A class that supports the application of a coordinate actuator to a model.
 * This actuator has no states; the control is simply the force to
 * be applied to the model.
 *
 * @author Frank C. Anderson
 * @version 1.0
 */
namespace OpenSim { 

class Coordinate;
class ForceSet;
class Model;

class OSIMACTUATORS_API CoordinateActuator : public CustomActuator
{
//=============================================================================
// DATA
//=============================================================================
protected:
	// PROPERTIES
	/** Name of coordinate to which the coordinate actuator is applied. */
	PropertyStr _propCoordinateName;
	/** Optimal force. */
	PropertyDbl _propOptimalForce;

	// REFERENCES
	std::string& _coordName;
	double &_optimalForce;

    /** Corresponding generalized coordinate to which the coordinate actuator
    is applied. */
    mutable Coordinate *_coord;

//=============================================================================
// METHODS
//=============================================================================
	//--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------
public:
	CoordinateActuator( std::string aCoordinateName="");
	CoordinateActuator( const CoordinateActuator &aGenForce);
	virtual ~CoordinateActuator();
	virtual Object* copy() const;
	void copyData(const CoordinateActuator &aGenForce);
private:
	void setNull();
	void setupProperties();
	

	//--------------------------------------------------------------------------
	// OPERATORS
	//--------------------------------------------------------------------------
public:
#ifndef SWIG
	CoordinateActuator& operator=(const CoordinateActuator &aGenForce);
#endif

	//--------------------------------------------------------------------------
	// GET AND SET
	//--------------------------------------------------------------------------
	// GENERALIZED COORDINATE
	void setCoordinate(Coordinate* aCoordinate);
	Coordinate* getCoordinate() const;
	// OPTIMAL FORCE
	void setOptimalForce(double aOptimalForce);
	double getOptimalForce() const;
	// STRESS
#ifndef SWIG
	double getStress( const SimTK::State& s ) const;

    // SIMTK STATE CACHE 
    virtual void initStateCache(SimTK::State& s, SimTK::SubsystemIndex subsystemIndex, Model& model);
	//--------------------------------------------------------------------------
	// APPLICATION
	//--------------------------------------------------------------------------
	virtual void computeForce( const SimTK::State& state, 
							   SimTK::Vector_<SimTK::SpatialVec>& bodyForces, 
							   SimTK::Vector& mobilityForces) const;

	//--------------------------------------------------------------------------
	// COMPUTATIONS
	//--------------------------------------------------------------------------
	virtual double  computeActuation( const SimTK::State& s) const;

	//--------------------------------------------------------------------------
	// UTILITY
	//--------------------------------------------------------------------------
	static ForceSet *CreateForceSetOfCoordinateActuatorsForModel(const SimTK::State& s, Model& aModel,double aOptimalForce = 1,bool aIncludeLockedAndConstrainedCoordinates = true);
#endif
	//--------------------------------------------------------------------------
	// CHECK
	//--------------------------------------------------------------------------
	virtual bool check() const;
	virtual bool isCoordinateValid() const;

	/** 
	 * Methods to query a Force for the value actually applied during simulation
	 * The names of the quantities (column labels) is returned by this first function
	 * getRecordLabels()
	 */
	virtual OpenSim::Array<std::string> getRecordLabels() const ;
	/**
	 * Given SimTK::State object extract all the values necessary to report forces, application location
	 * frame, etc. used in conjunction with getRecordLabels and should return same size Array
	 */
	virtual OpenSim::Array<double> getRecordValues(const SimTK::State& state) const ;
	/**
	 * For a CooridnateActuator, the speed is really the speed of the coordinate underneath it
	 * this overrides the default behavior (bug )
	 */
	virtual double getSpeed( const SimTK::State& s) const;

	//--------------------------------------------------------------------------
	// XML
	//--------------------------------------------------------------------------
	virtual void updateFromXMLNode();

	OPENSIM_DECLARE_DERIVED(CoordinateActuator, Actuator);

protected:
	// Setup method to initialize coordinate reference
	virtual void setup(Model &aModel);
	virtual void createSystem( SimTK::MultibodySystem& system) const ;

//=============================================================================
};	// END of class CoordinateActuator

}; //namespace
//=============================================================================
//=============================================================================

#endif // __CoordinateActuator_h__


