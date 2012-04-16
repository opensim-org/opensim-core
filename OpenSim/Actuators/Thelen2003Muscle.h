#ifndef __Thelen2003Muscle_h__
#define __Thelen2003Muscle_h__

// Thelen2003Muscle.h
/*    Author: Matthew Millard
/*
 * Permission is hereby granted, free of charge, to any person obtaining a    *
 * copy of this software and associated documentation files (the "Software"), *
 * to deal in the Software without restriction, including without limitation  *
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,   *
 * and/or sell copies of the Software, and to permit persons to whom the      *
 * Software is furnished to do so, subject to the following conditions:       *
 *                                                                            *
 * The above copyright notice and this permission notice shall be included in *
 * all copies or substantial portions of the Software.                        *
 *                                                                            *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR *
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,   *
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL    *
 * THE AUTHORS, CONTRIBUTORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,    *
 * DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR      *
 * OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE  *
 * USE OR OTHER DEALINGS IN THE SOFTWARE.                                     *
 * -------------------------------------------------------------------------- */


// INCLUDE
#include <OpenSim/Actuators/osimActuatorsDLL.h>
#include <OpenSim/Simulation/Model/ActivationFiberLengthMuscle.h>
#include <OpenSim/Simulation/Model/MuscleFirstOrderActivationDynamicModel.h>
#include <OpenSim/Simulation/Model/MuscleFixedWidthPennationModel.h>

#ifdef SWIG
    #ifdef OSIMACTUATORS_API
        #undef OSIMACTUATORS_API
        #define OSIMACTUATORS_API
    #endif
#endif

namespace OpenSim {

/**
 * Implementation of a two state (activation and fiber-length) Muscle model by:
 * DG Thelen, Adjustment of muscle mechanics model parameters to simulate dynamic 
 * contractions in older adults. Journal of biomechanical engineering, 2003.
 * This a complete rewrite of a previous implementation by Peter Loan.
 *
 * @author Matt Millard
 * @author Ajay Seth
 * @author Peter Loan
 */
class OSIMACTUATORS_API Thelen2003Muscle : public ActivationFiberLengthMuscle {
OpenSim_DECLARE_CONCRETE_OBJECT(Thelen2003Muscle, ActivationFiberLengthMuscle);


public:
//=============================================================================
// Construction
//=============================================================================

    Thelen2003Muscle();
    Thelen2003Muscle(const std::string &aName,double aMaxIsometricForce,
                    double aOptimalFiberLength,double aTendonSlackLength,
                    double aPennationAngle);
    Thelen2003Muscle(const Thelen2003Muscle &aMuscle);
    virtual ~Thelen2003Muscle();

    #ifndef SWIG
        Thelen2003Muscle& operator=(const Thelen2003Muscle &aMuscle);
        #endif
            void copyData(const Thelen2003Muscle &aMuscle);
        #ifndef SWIG
    #endif

//==============================================================================
// Get and Set Properties
//==============================================================================
    // Properties
    virtual double getActivationTimeConstant() const;
    virtual double getActivationMinimumValue() const;
    virtual double getDeactivationTimeConstant() const;
    //virtual double getVmax() const;
    virtual double getFmaxTendonStrain() const;
    virtual double getFmaxFiberStrain() const;
    virtual double getKshapeActive() const;
    virtual double getKshapePassive() const;
    virtual double getAf() const;
    virtual double getFlen() const;
    virtual double getForceVelocityExtrapolationThreshold() const;

    virtual bool setActivationTimeConstant(double aActivationTimeConstant);
    virtual bool setActivationMinimumValue(double aActivationMinValue);
    virtual bool setDeactivationTimeConstant(double aDeactivationTimeConstant);
    //virtual bool setVmax(double aVmax);
    virtual bool setFmaxTendonStrain(double aFmaxTendonStrain);
    virtual bool setFmaxFiberStrain(double aFmaxMuscleStrain);
    virtual bool setKshapeActive(double aKShapeActive);
    virtual bool setKshapePassive(double aKshapePassive);
    virtual bool setAf(double aAf);
    virtual bool setFlen(double aFlen);;
    virtual bool setForceVelocityExtrapolationThreshold(double aFvThresh);

//==============================================================================
// Public Computations
//==============================================================================
    //Ajay: this is old. Can I stop calling it?
    virtual double computeActuation(const SimTK::State& s) const;


    /** Compute initial fiber length (velocity) such that muscle fiber and 
        tendon are in static equilibrium and update the state
        
        Part of the Muscle.h interface
    */
    void computeInitialFiberEquilibrium(SimTK::State& s) const; /*virtual*/
    double computeIsometricForce(   SimTK::State& s, 
                                    double activation) const;/*virtual*/

protected:
//==============================================================================
// Protected Computations
//==============================================================================

    /**calculate muscle's position related values such fiber and tendon lengths,
    normalized lengths, pennation angle, etc... */
    void calcMuscleLengthInfo(const SimTK::State& s, 
                              MuscleLengthInfo& mli) const; /*virtual*/ 


    /** calculate muscle's velocity related values such fiber and tendon 
        velocities,normalized velocities, pennation angular velocity, etc... */
    virtual void  calcFiberVelocityInfo(const SimTK::State& s, 
                                      FiberVelocityInfo& fvi) const; /*virtual*/ 

    /** calculate muscle's active and passive force-length, force-velocity, 
        tendon force, relationships and their related values */
    virtual void  calcMuscleDynamicsInfo(const SimTK::State& s, 
                                    MuscleDynamicsInfo& mdi) const; /*virtual*/ 

    /** Calculate activation rate */
    double calcActivationRate(const SimTK::State& s) const; /*virtual*/

    void createSystem(SimTK::MultibodySystem& system) const;


private:
    void setNull();
    void setupProperties();

    //=====================================================================
    // Private Utility Class Members
    //      -Computes activation dynamics and fiber kinematics
    //=====================================================================

    //Activation Dynamics
    mutable MuscleFirstOrderActivationDynamicModel *actMdl;

    //Fiber and Tendon Kinematics
    mutable MuscleFixedWidthPennationModel *penMdl;

    //=====================================================================
    // Private Accessor names
    //=====================================================================
    //This is so we can get some compiler checking on these string names


    //=====================================================================
    // Private Computation
    //      -Computes curve values, derivatives and integrals
    //=====================================================================

    //Initialization
    SimTK::Vector initMuscleState(SimTK::State& s, double aActivation,
                             double aSolTolerance, int aMaxIterations) const;

    //Stiffness related functions
    double calcFm(double ma, double fal, double fv, double fpe, double fiso) const;


    double calcDFmDlce(double lce, double a,  double fv, 
                      double fiso, double ofl) const;

    double calcDFmATDlce(double lce, double phi, double cosphi, 
    double Fm, double d_Fm_d_lce, double penHeight) const;

    double calcDFseDlce(double tl, double lce, double phi, double cosphi, 
                                    double fiso, double tsl, double vol) const;

    double calcDFseDtl(double tl, double fiso, double tsl) const;
    
    
    //Tendon related helper functions
    double calcfse(double tlN) const;
    double calcDfseDtlN(double tlN) const;
    double calcfsefisoPE(double tlN) const;

    //Active force length functions
    double calcfal( double lceN) const;
    double calcDfalDlceN( double lceN) const;

    //Parallel element functions    
    double calcfpe(double lceN) const;
    double calcDfpeDlceN(double lceN) const;
    double calcfpefisoPE(double lceN) const;    

    //Force velocity functions      
    double calcdlceN(double act,double fal, double actFalFv) const;
    double calcfv(double aFse, double aFpe, double aFal,
                  double aCosPhi, double aAct) const;
    SimTK::Vector calcfvInv(double aAct,  double aFal, double dlceN, 
                            double tolerance, int maxIterations) const;
    double calcDdlceDaFalFv(double aAct, double fal, 
                            double aFalFv) const;


};    
} // end of namespace OpenSim

#endif // __Thelen2003Muscle_h__
