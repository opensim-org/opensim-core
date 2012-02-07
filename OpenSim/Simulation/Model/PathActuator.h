#ifndef _PathActuator_h_
#define _PathActuator_h_
// PathActuator.h
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
/*
* Copyright (c)  2011, Stanford University. All rights reserved. 
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
/**
 * A class that applies controllable tension along a geometry path.
 * This actuator has no states; the control is simply the tension to
 * be applied along a geometry path (i.e. tensionable rope).
 *
 * @author Ajay Seth
 * @version 1.0
 */
namespace OpenSim { 

class Coordinate;
class ForceSet;
class Model;

class OSIMSIMULATION_API PathActuator : public Actuator
{
//=============================================================================
// METHODS
//=============================================================================
	//--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------
public:
	PathActuator();
	PathActuator( const PathActuator &aGenForce);
	virtual ~PathActuator();
	virtual Object* copy() const;
	void copyData(const PathActuator &aGenForce);
private:
	void setNull();
	void setupProperties();
	

	//--------------------------------------------------------------------------
	// OPERATORS
	//--------------------------------------------------------------------------
public:
#ifndef SWIG
	PathActuator& operator=(const PathActuator &aGenForce);
#endif

	//--------------------------------------------------------------------------
	// GET AND SET
	//--------------------------------------------------------------------------
	// Path
	GeometryPath& updGeometryPath() { return updPropertyValue<GeometryPath>("GeometryPath"); }
	const GeometryPath& getGeometryPath() const { return getPropertyValue<GeometryPath>("GeometryPath"); }
	virtual bool hasGeometryPath() const { return true;};

	// OPTIMAL FORCE
	void setOptimalForce(double aOptimalForce);
	double getOptimalForce() const;

	// Length and Speed of actuator
	virtual double getLength(const SimTK::State& s) const;
	virtual double getLengtheningSpeed(const SimTK::State& s) const;

	// Power: Since lengthening is positive and tension always shortens, positive power
	// is when muscle is shortening under tension.
	virtual double getPower(const SimTK::State& s) const { return -getForce(s)*getSpeed(s); }


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

	OPENSIM_DECLARE_DERIVED(PathActuator, Actuator);

protected:
	// Setup method to initialize coordinate reference
	virtual void setup(Model &aModel);



//=============================================================================
};	// END of class PathActuator

}; //namespace
//=============================================================================
//=============================================================================

#endif // __PathActuator_h__


