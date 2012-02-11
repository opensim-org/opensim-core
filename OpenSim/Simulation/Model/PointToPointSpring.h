#ifndef _PointToPointSpring_h_
#define _PointToPointSpring_h_
// PointToPointSpring.h
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
/*
* Copyright (c)  2009, Stanford University. All rights reserved. 
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

/*
 * Author: Ajay Seth
 */

#include "Force.h"
#include <OpenSim/Common/PropertyStr.h>
#include <OpenSim/Common/PropertyDbl.h>
#include <OpenSim/Common/PropertyDblVec.h>
#include <OpenSim/Common/VisibleObject.h>

//=============================================================================
//=============================================================================
/**
 * A simple point to point spring with a resting length and stiffness.
 * Points are connected to bodies and are defined in the body frame.
 *
 * @author Ajay Seth
 * @version 1.0
 */
namespace OpenSim { 

class OSIMSIMULATION_API PointToPointSpring : public Force
{
//=============================================================================
// DATA
//=============================================================================
protected:
	/** how to display the Spring */
	VisibleObject _displayer;

//=============================================================================
// METHODS
//=============================================================================
	//--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------
public:
	// Default
	PointToPointSpring();
	// Convenience constructor for API users
	PointToPointSpring( std::string body1Name, SimTK::Vec3 point1, 
		                std::string body2Name, SimTK::Vec3 point2, double stiffness, double restlength );
	PointToPointSpring( const PointToPointSpring &aPointToPointSpring);
	virtual ~PointToPointSpring();
	virtual Object* copy() const;
	void copyData(const PointToPointSpring &aPointToPointSpring);

	//--------------------------------------------------------------------------
	// Visible Object Support
	//--------------------------------------------------------------------------
	virtual VisibleObject* getDisplayer() const;
	virtual void updateDisplayer(const SimTK::State& s);
	virtual void updateGeometry(const SimTK::State& s);
	
private:
	void setNull();
	void setupProperties();
	

	//--------------------------------------------------------------------------
	// OPERATORS
	//--------------------------------------------------------------------------
public:
#ifndef SWIG
	PointToPointSpring& operator=(const PointToPointSpring &aGenForce);
#endif

	//-----------------------------------------------------------------------------
	// GET and SET Spring parameters
	//-----------------------------------------------------------------------------
	/**
	* Spring end point bodies 
	* 
	* @param std::string bodyName<1/2>
	*/
	void setBody1Name(std::string body1Name) {setPropertyValue("body1", body1Name);}
	void setBody2Name(std::string body2Name) {setPropertyValue("body2", body2Name);}
	std::string getBody1Name() const {return getPropertyValue<std::string>("body1");}
	std::string getBody2Name() const {return getPropertyValue<std::string>("body2");}

	/**
	* Spring end points 
	* 
	* @param Vec3 point<1/2> 
	*/
	void setPoint1(SimTK::Vec3 aPosition) { setPropertyValue("point1", aPosition); }
	SimTK::Vec3 getPoint1() const { return getPropertyValue<SimTK::Vec3>("point1"); }
	void setPoint2(SimTK::Vec3 aPosition) { setPropertyValue("point2", aPosition); }
	SimTK::Vec3 getPoint2() const { return getPropertyValue<SimTK::Vec3>("point2"); }

	/**
	* Spring stiffness
	* @param stiffness 
	*/
	void setStiffness(double stiffness) {setPropertyValue("stiffness", stiffness);}
	double getStiffness() const {return getPropertyValue<double>("stiffness");}
	/**
	* Spring rest length
	* @param restLength 
	*/
	void setRestlength(double restLength) {setPropertyValue("rest_length", restLength);}
	double getRestlength() const {return getPropertyValue<double>("rest_length");}

	//-----------------------------------------------------------------------------
	// Model setup and system creation
	//-----------------------------------------------------------------------------
	void setup(Model& model);
	void createSystem(SimTK::MultibodySystem& system) const;

	//-----------------------------------------------------------------------------
	// Reporting
	//-----------------------------------------------------------------------------
	/** 
	 * Provide name(s) of the quantities (column labels) of the force value(s) to be reported
	 */
	virtual OpenSim::Array<std::string> getRecordLabels() const ;
	/**
	*  Provide the value(s) to be reported that correspond to the labels
	*/
	virtual OpenSim::Array<double> getRecordValues(const SimTK::State& state) const ;

	OPENSIM_DECLARE_DERIVED(PointToPointSpring, Force);

//=============================================================================
};	// END of class PointToPointSpring

}; //namespace
//=============================================================================
//=============================================================================

#endif // __PointToPointSpring_h__
