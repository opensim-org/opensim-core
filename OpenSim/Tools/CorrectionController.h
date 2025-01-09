/* -------------------------------------------------------------------------- *
 *                      OpenSim:  CorrectionController.h                      *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
 * Author(s): Chand T. John, Samuel R. Hamner, Ajay Seth                      *
 *                                                                            *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may    *
 * not use this file except in compliance with the License. You may obtain a  *
 * copy of the License at http://www.apache.org/licenses/LICENSE-2.0.         *
 *                                                                            *
 * Unless required by applicable law or agreed to in writing, software        *
 * distributed under the License is distributed on an "AS IS" BASIS,          *
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   *
 * See the License for the specific language governing permissions and        *
 * limitations under the License.                                             *
 * -------------------------------------------------------------------------- */

#ifndef CorrectionController_h__
#define CorrectionController_h__

//============================================================================
// INCLUDE
//============================================================================
#include "osimToolsDLL.h"
#include <OpenSim/Simulation/Control/TrackingController.h>


namespace OpenSim {

class Model;
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
    void computeControls(const SimTK::State& s, SimTK::Vector& controls) const override;

protected:
    // for any post XML deserialization initialization
    void extendConnectToModel(Model& model) override;

    // for any initialization requiring a state or the complete system 
    void extendInitStateFromProperties( SimTK::State& s) const override;

//=============================================================================
};  // END of class CorrectionController
//=============================================================================
//=============================================================================

}; // end namespace

#endif // CorrectionController_h__


