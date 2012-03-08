#ifndef __HuntCrossleyForce_h__
#define __HuntCrossleyForce_h__
// HuntCrossleyForce.h
// Author: Peter Eastman
/*
 * Copyright (c)  2009 Stanford University. All rights reserved. 
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
// INCLUDE
#include "Force.h"
#include "OpenSim/Common/PropertyDbl.h"
#include "OpenSim/Common/PropertyObj.h"
#include "OpenSim/Common/PropertyStrArray.h"
#include "OpenSim/Common/Set.h"
namespace OpenSim {

class Model;

/**
 * This force subclass implements a Hunt-Crossley contact model.  It uses Hertz contact theory
 * to model the interactions between a set of ContactSpheres and ContactHalfSpaces.
 *
 * @author Peter Eastman
 */
class OSIMSIMULATION_API HuntCrossleyForce : public Force
{
//=============================================================================
// MEMBER CLASSES
//=============================================================================
public:
    class ContactParameters;
    class ContactParametersSet;
//=============================================================================
// METHODS
//=============================================================================
public:
    HuntCrossleyForce();
    HuntCrossleyForce(const HuntCrossleyForce& copy);
    HuntCrossleyForce(ContactParameters* params);
	void copyData(const HuntCrossleyForce& copy);
	Object* copy() const;

    ContactParametersSet& updContactParametersSet();
    const ContactParametersSet& getContactParametersSet();
    void addContactParameters(ContactParameters* params);
    /**
     * Get the transition velocity for switching between static and dynamic friction.
     */
    double getTransitionVelocity() const;
    /**
     * Set the transition velocity for switching between static and dynamic friction.
     */
    void setTransitionVelocity(double velocity);
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

	/**
	 * Create a SimTK::Force which implements this Force.
	 */
	void createSystem(SimTK::MultibodySystem& system) const;


private:
    // INITIALIZATION
	void setupProperties();

//=============================================================================
};	// END of class HuntCrossleyForce
//=============================================================================
//=============================================================================

class OSIMSIMULATION_API HuntCrossleyForce::ContactParameters : public Object
{
private:
    void setupProperties();
public:
    ContactParameters();
    ContactParameters(const ContactParameters& copy);
    ContactParameters(double stiffness, double dissipation, double staticFriction, double dynamicFriction, double viscousFriction);
	void copyData(const ContactParameters& copy);
	Object* copy() const;
    const Array<std::string>& getGeometry() const;
    Array<std::string>& updGeometry();
    void addGeometry(const std::string& name);
    double getStiffness() const;
    void setStiffness(double stiffness);
    double getDissipation() const;
    void setDissipation(double dissipation);
    double getStaticFriction() const;
    void setStaticFriction(double friction);
    double getDynamicFriction() const;
    void setDynamicFriction(double friction);
    double getViscousFriction() const;
    void setViscousFriction(double friction);
};

class OSIMSIMULATION_API HuntCrossleyForce::ContactParametersSet :	public Set<HuntCrossleyForce::ContactParameters>
{
private:
	void setNull();
public:
	ContactParametersSet();
	ContactParametersSet(const ContactParametersSet& copy);
	~ContactParametersSet(void);
	Object* copy() const;
#ifndef SWIG
	ContactParametersSet& operator=(const ContactParametersSet &copy);
#endif
};

template<> struct PropertyTypeName<HuntCrossleyForce::ContactParametersSet> 
{   static const char* name() {return "ContactParametersSet";} };

template <> inline AbstractProperty::PropertyType 
Property2<HuntCrossleyForce::ContactParametersSet>::getPropertyType() const 
{   return Obj; }

} // end of namespace OpenSim

#endif // __HuntCrossleyForce_h__


