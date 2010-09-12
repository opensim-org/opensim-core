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

//=============================================================================
//=============================================================================
/**
 * Asimple point to point spring with a resting length and stiffness.
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
	// PROPERTIES
	/** Name of Body to which 1 end of the spring is attached. */
	PropertyStr _propBody1Name;

	/** Name of Body to which the 2nd end of the spring is attached. */
	PropertyStr _propBody2Name;

	/** Point of spring application on each body */
	PropertyDblVec3 _propPoint1;
	PropertyDblVec3 _propPoint2;

	/** Spring stiffness property */
	PropertyDbl _propStiffness;

	/** Spring rest length property */
	PropertyDbl _propRestlength;

	// REFERENCES
	/** Names of bodies to which ends of the spring are attached. */
	std::string& _body1Name;
	std::string& _body2Name;

	/** Points of application on each body */
	SimTK::Vec3 &_point1;
	SimTK::Vec3 &_point2;
	
	/** Spring stiffness */
	double &_stiffness;

	/** Spring rest length */
	double &_restLength;

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
	void setBody1Name(std::string body1Name) {_body1Name = body1Name;} ;
	void setBody2Name(std::string body2Name) {_body2Name = body2Name;} ;
	std::string getBody1Name() const {return _body1Name;} ;
	std::string getBody2Name() const {return _body2Name;};

	/**
	* Spring end points 
	* 
	* @param Vec3 point<1/2> 
	*/
	void setPoint1(SimTK::Vec3 aPosition) { _point1 = aPosition; } ;
	SimTK::Vec3 getPoint1() const { return _point1; };
	void setPoint2(SimTK::Vec3 aPosition) { _point2 = aPosition; } ;
	SimTK::Vec3 getPoint2() const { return _point2; };

	/**
	* Spring stiffness
	* @param stiffness 
	*/
	void setStiffness(double stiffness) {_stiffness = stiffness;} ;
	double getStiffness() const {return _stiffness;} ;
	/**
	* Spring rest length
	* @param restLength 
	*/
	void setRestlength(double restLength) {_restLength = restLength;} ;
	double getRestlength() const {return _restLength;} ;

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
