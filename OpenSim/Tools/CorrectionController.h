// CorrectionController.h
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Contributors: Chand T. John, Samuel R. Hamner, Ajay Seth
//
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

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#ifndef CorrectionController_h__
#define CorrectionController_h__

//============================================================================
// INCLUDE
//============================================================================
#include "osimToolsDLL.h"
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/Control/TrackingController.h>


namespace OpenSim {

//=============================================================================
//=============================================================================
/**
 * This is a simple feedback controller to be used in generating a forward
 * dynamic simulation.
 *
 * @author Chand T. John, Samuel R. Hamner, Ajay Seth
 * @version 1.0
 */

class OSIMTOOLS_API CorrectionController : public TrackingController {
OpenSim_DECLARE_CONCRETE_OBJECT(CorrectionController, TrackingController);

//=============================================================================
// DATA
//=============================================================================
protected:
	/** Gain for position errors.  This property can be specified in a
	 *  setup file. */
	PropertyDbl _kpProp;
	double &_kp;
	/** Gain for velocity errors.  This property can be specified in a
	 *  setup file. */
	PropertyDbl _kvProp;
	double &_kv;

	/** States for the simulation. */
	Storage *_yDesStore;


//=============================================================================
// METHODS
//=============================================================================
	//--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------
public:
	CorrectionController();
	CorrectionController(const std::string &aFileName, bool aUpdateFromXMLNode = true);
	CorrectionController(const CorrectionController &aController);
	virtual ~CorrectionController();

private:
	void setNull();
protected:
	void setupProperties();
	void copyData(const CorrectionController &aController);

	//--------------------------------------------------------------------------
	// OPERATORS
	//--------------------------------------------------------------------------
public:

#ifndef SWIG

	/**
	 * Assignment operator.  This method is called automatically whenever a
	 * command of the form "controller1 = controller2;" is made, where both
	 * controller1 and controller2 are both of type Controller.  Although
	 * Controller cannot be instantiated directly, a subclass of Controller
	 * could implement its own operator= method that calls Controller's
	 * operator= method.  If the subclass does not implement its own operator=
	 * method, then when a command of the form "controller1 = controller2" is
	 * made, where both controller1 and controller2 are instants of the
	 * subclass, the Controller class's operator= method will be called
	 * automatically.
	 *
	 * @param aController The controller to be copied.
	 * @return Reference to the altered object.
	 */
	CorrectionController& operator=(const CorrectionController &aController);

#endif

	//--------------------------------------------------------------------------
	// GET AND SET
	//--------------------------------------------------------------------------
	double getKp() const;
	void setKp(double aKp);
	double getKv() const;
	void setKv(double aKv);


	//--------------------------------------------------------------------------
	// COMPUTATION
	//--------------------------------------------------------------------------

    virtual void computeControls(const SimTK::State& s, SimTK::Vector& controls) const;


protected:
	// for any post XML deseraialization intialization
	virtual void setup(Model& model);

	// controller creation once the setup is complete 
	virtual void createSystem( SimTK::MultibodySystem& system) const;   

	// for any intialization requiring a state or the complete system 
	virtual void initState( SimTK::State& s) const;



//=============================================================================
};	// END of class CorrectionController
//=============================================================================
//=============================================================================

}; // end namespace

#endif // CorrectionController_h__


