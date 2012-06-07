#ifndef OPENSIM_PATH_ACTUATOR_H_
#define OPENSIM_PATH_ACTUATOR_H_
// PathActuator.h
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
/*
* Copyright (c)  2011-12, Stanford University. All rights reserved. 
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
*
* Author: Ajay Seth
*/

#include <OpenSim/Common/PropertyObj.h>
#include <OpenSim/Common/PropertyDbl.h>
#include "Actuator.h"
#include "GeometryPath.h"

//=============================================================================
//=============================================================================

namespace OpenSim { 

class Coordinate;
class ForceSet;
class Model;

/**
 * This is the base class for actuators that apply controllable tension along 
 * a geometry path. %PathActuator has no states; the control is simply the 
 * tension to be applied along a geometry path (i.e. tensionable rope).
 *
 * @author Ajay Seth
 */
class OSIMSIMULATION_API PathActuator : public Actuator {
OpenSim_DECLARE_CONCRETE_OBJECT(PathActuator, Actuator);
public:
//=============================================================================
// PROPERTIES
//=============================================================================
    /** @name Property declarations
    These are the serializable properties associated with the %PathActuator
    class. Note that objects derived from this class inherit these
    properties. **/
    /**@{**/
    OpenSim_DECLARE_UNNAMED_PROPERTY(GeometryPath,
		"The set of points defining the path of the muscle.");
    OpenSim_DECLARE_PROPERTY(optimal_force, double,
        "The maximum force this actuator can produce.");
    /**@}**/

//=============================================================================
// PUBLIC METHODS
//=============================================================================
	PathActuator();

    // default destructor, copy constructor, copy assignment

	//--------------------------------------------------------------------------
	// GET AND SET
	//--------------------------------------------------------------------------
	// Path
	GeometryPath& updGeometryPath() { return upd_GeometryPath(); }
	const GeometryPath& getGeometryPath() const 
    {   return get_GeometryPath(); }
	virtual bool hasGeometryPath() const { return true;};

	// OPTIMAL FORCE
	void setOptimalForce(double aOptimalForce);
	double getOptimalForce() const;

	// Length and Speed of actuator
	virtual double getLength(const SimTK::State& s) const;
	virtual double getLengtheningSpeed(const SimTK::State& s) const;

	// Power: Since lengthening is positive and tension always shortens, positive power
	// is when muscle is shortening under tension.
	virtual double getPower(const SimTK::State& s) const 
    {   return -getForce(s)*getSpeed(s); }


	// STRESS
	virtual double getStress( const SimTK::State& s ) const;

    // Convenience method to add PathPoints
	 /** Note that this function does not maintain the State and so should be used only
		before a valid State is created */
	 void addNewPathPoint( const std::string& proposedName, OpenSim::Body& aBody, 
						   const SimTK::Vec3& aPositionOnBody);

	//--------------------------------------------------------------------------
	// APPLICATION
	//--------------------------------------------------------------------------
	virtual void computeForce( const SimTK::State& state, 
							   SimTK::Vector_<SimTK::SpatialVec>& bodyForces, 
							   SimTK::Vector& mobilityForces) const;

	//--------------------------------------------------------------------------
	// COMPUTATIONS
	//--------------------------------------------------------------------------
	virtual double computeActuation( const SimTK::State& s) const;
	virtual double computeMomentArm(SimTK::State& s, Coordinate& aCoord) const;

	//--------------------------------------------------------------------------
	// XML
	//--------------------------------------------------------------------------
	virtual void updateFromXMLNode(SimTK::Xml::Element& aNode, int versionNumber=-1);

	//--------------------------------------------------------------------------
	// SCALING
	//--------------------------------------------------------------------------
	virtual void preScale(const SimTK::State& s, const ScaleSet& aScaleSet);
	virtual void scale(const SimTK::State& s, const ScaleSet& aScaleSet);
	virtual void postScale(const SimTK::State& s, const ScaleSet& aScaleSet);

	//--------------------------------------------------------------------------
	// Visible Object Support
	//--------------------------------------------------------------------------
	virtual VisibleObject* getDisplayer() const;
	virtual void updateDisplayer(const SimTK::State& s);

protected:
	// Setup method to initialize coordinate reference
	void connectToModel(Model& aModel) OVERRIDE_11;


private:
	void setNull();
	void constructProperties();

//=============================================================================
};	// END of class PathActuator

}; //namespace
//=============================================================================
//=============================================================================

#endif // OPENSIM_PATH_ACTUATOR_H_


