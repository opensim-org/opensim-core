#ifndef OPENSIM_POINT_TO_POINT_SPRING_H_
#define OPENSIM_POINT_TO_POINT_SPRING_H_
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

#include <OpenSim/Common/PropertyStr.h>
#include <OpenSim/Common/PropertyDbl.h>
#include <OpenSim/Common/PropertyDblVec.h>
#include <OpenSim/Common/VisibleObject.h>
#include "Force.h"

//==============================================================================
//                          POINT TO POINT SPRING
//==============================================================================
/**
 * A simple point to point spring with a resting length and stiffness.
 * Points are connected to bodies and are defined in the body frame.
 *
 * @author Ajay Seth
 */
namespace OpenSim { 

class OSIMSIMULATION_API PointToPointSpring : public Force {
OpenSim_DECLARE_CONCRETE_OBJECT(PointToPointSpring, Force);
public:
//==============================================================================
// PROPERTIES
//==============================================================================
    /** @name Property declarations
    These are the serializable properties associated with this class. **/
    /**@{**/
    OpenSim_DECLARE_OPTIONAL_PROPERTY(body1, std::string,
		"Name of Body to which 1 end of the spring is attached.");
	OpenSim_DECLARE_OPTIONAL_PROPERTY(body2, std::string,
		"Name of Body to which the 2nd end of the spring is attached.");
	OpenSim_DECLARE_PROPERTY(point1, SimTK::Vec3,
		"Force application point on body1.");
	OpenSim_DECLARE_PROPERTY(point2, SimTK::Vec3,
		"Force application point on body2.");
	OpenSim_DECLARE_PROPERTY(stiffness, double,
		"Spring stiffness.");
	OpenSim_DECLARE_PROPERTY(rest_length, double,
		"Spring resting length.");
    /**@}**/


//==============================================================================
// PUBLIC METHODS
//==============================================================================
	/** Default constructor. **/
	PointToPointSpring();
    /** Convenience constructor for API users.
    @param body1Name    name of the first body to which the spring is attached
    @param point1       location where spring is attached on body1
    @param body2Name    name of the second body to which the spring is attached
    @param point2       location where spring is attached on body2 
    @param stiffness    spring stiffness
    @param restlength   the resting (zero force) length of the spring
    **/
	PointToPointSpring( std::string body1Name, SimTK::Vec3 point1, 
		                std::string body2Name, SimTK::Vec3 point2, 
                        double stiffness, double restlength );

    // default destructor, copy constructor, copy assignment

	//--------------------------------------------------------------------------
	// Visible Object Support
	//--------------------------------------------------------------------------
	virtual VisibleObject* getDisplayer() const;
	virtual void updateDisplayer(const SimTK::State& s);
	virtual void updateGeometry(const SimTK::State& s);
	
	//-----------------------------------------------------------------------------
	// GET and SET Spring parameters
	//-----------------------------------------------------------------------------
	/**
	* Spring end point bodies 
	* 
	* @param std::string bodyName<1/2>
	*/
	void setBody1Name(const std::string& body1Name) 
    {   set_body1(body1Name); }
	void setBody2Name(const std::string& body2Name) 
    {   set_body2(body2Name); }
	const std::string& getBody1Name() const {return get_body1();}
	const std::string& getBody2Name() const {return get_body2();}

	/**
	* Spring end points 
	* 
	* @param Vec3 point<1/2> 
	*/
	void setPoint1(SimTK::Vec3 aPosition) { set_point1(aPosition); }
	const SimTK::Vec3& getPoint1() const { return get_point1(); }
	void setPoint2(SimTK::Vec3 aPosition) { set_point2(aPosition); }
	const SimTK::Vec3& getPoint2() const { return get_point2(); }

	/**
	* Spring stiffness
	* @param stiffness 
	*/
	void setStiffness(double stiffness) {set_stiffness(stiffness);}
	double getStiffness() const {return get_stiffness();}
	/**
	* Spring resting length
	* @param restLength 
	*/
	void setRestlength(double restLength) {set_rest_length(restLength);}
	double getRestlength() const {return get_rest_length();}

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

protected:
	/** how to display the Spring */
	VisibleObject _displayer;

private:
	void setNull();
	void constructProperties();

//==============================================================================
};	// END of class PointToPointSpring

}; //namespace
//==============================================================================
//==============================================================================

#endif // OPENSIM_POINT_TO_POINT_SPRING_H_
