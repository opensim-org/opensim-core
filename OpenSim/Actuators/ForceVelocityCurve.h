#ifndef OPENSIM_ForceVelocityCurve_h__
#define OPENSIM_ForceVelocityCurve_h__

/* Author: Matthew Millard
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

#include <OpenSim/Actuators/osimActuatorsDLL.h>


// INCLUDE
#include <simbody/internal/common.h>
#include <OpenSim/Simulation/Model/ModelComponent.h>
#include <OpenSim/Common/MuscleCurveFunctionFactory.h>
#include <OpenSim/Common/MuscleCurveFunction.h>

#ifdef SWIG
    #ifdef OSIMACTUATORS_API
        #undef OSIMACTUATORS_API
        #define OSIMACTUATORS_API
    #endif
#endif

namespace OpenSim {

/**
 This class serves as a serializable ForceVelocityCurve, for use in 
 muscle models. 
 
 \image html fig_SerializableCurve_ForceVelocityCurve.png

 
  @author Matt Millard

 */
class OSIMACTUATORS_API ForceVelocityCurve : public ModelComponent {
OpenSim_DECLARE_CONCRETE_OBJECT(ForceVelocityCurve, ModelComponent);
    
//    class ForceVelocityCurve : public ModelComponent {
//OpenSim_DECLARE_CONCRETE_OBJECT(ForceVelocityCurve, ModelComponent);



public:

    ///Default constructor
    ForceVelocityCurve();
    ///Default destructor
    ~ForceVelocityCurve();
    ///Default constructor
    ForceVelocityCurve(const ForceVelocityCurve& source);

    /**
     Constructs a C2 continuous force velocity curve. The force velocity
     curve requries 6 different properties and a name in order to 
     construct a curve:

     @param concentricMinSlope 
                The slope of the force velocity curve at a normalized (w.r.t.
                vmax * optimal fiber length) contraction velocity of -1, which
                is also the minimum slope value the concentric side of the 
                force velocity curve achieves. 
                A physiologically accurate value for this paramter is 0, 
                though values greater than 0 are necessary when this curve is 
                used in the context of an equilibrium muscle model because an 
                equilibrium muscle model requires that this curve be 
                invertible (this curve is not invertible when it has a 
                minConcentricSlope of 0).

                
     @param isometricMaxSlope   
                The slope of the force velocity curve at a normalized (w.r.t.
                vmax * optimal fiber length) contraction velocity of 0, which
                is also the maximum slope achieved by the force velocity curve.
                A physiologically accurate value for this parameter is 5 [1]
                (pp 55), which is the default value. Although this parameter 
                can be changed, it must be positive and greater than 
                max( (maxEccentricMultiplier-1)/1, 1). 
                The value of this parameter also affects how much the eccentric 
                and concentric curves can be bent by the 'eccentricCurviness', 
                and 'concentricCurviness' parameters, as it places an upper 
                limit on the maximum slope of the force velocity curve.

     @param eccentricMinSlope   
                The slope of the force velocity curve at a normalized (w.r.t.
                vmax * optimal fiber length) contraction velocity of 1, which
                is also the minimum slope the eccentric side of the force
                velocity curve achieves.
                The normalized fiber length where the descending limb 
                transitions to the minimum value and has a first and second 
                derivative of 0.
            
     @param maxEccentricVelocityForceMultiplier
                The value of the force velocity curve, force velocity 
                multiplier, at the maximum eccentric contraction velocity.
                Physiologically acccurate values for this parameter range 
                between 1.1 and 1.8, and may vary from subject to subject.
                

     @param concentricCurviness
                A dimensionless parameter between [0-1] that controls how 
                the concentric curve is drawn: 0 will create a curve that is
                very close to a straight line segment (like a fast twitch fiber)
                while a value of 1 will create a curve
                that smoothly fills the corner formed by the linear 
                extrapolation of 'concentricMinSlope' and 'isometricMaxSlope',
                as shown in the figure. Depending on how deep the corner formed
                by the linear extrapolation of 'concentricMinSlope' and 
                'isometricMaxSlope' is, a low value of 'concentricCurviness'
                will achieve a concentric curve consistent with a slow twitch
                fiber.

     @param eccentricCurviness
                A dimensionless parameter between [0-1] that controls how 
                the eccentric curve is drawn: 0 will create a curve that is
                very close to a straight line segment while a value of 1 will 
                create a curve that smoothly fills the corner formed by the 
                linear extrapolation of 'isometricMaxSlope' and 
                'eccentricMinSlope', as shown in the figure. 


     @param muscleName
                The name of the muscle this curve belongs to. This name is used
                to create the name of this curve, which is formed simply by 
                appending "_ForceVelocityCurve" to the string in muscleName.
                This name is used for making intelligible error messages and 
                also for naming the XML version of this curve when it is 
                serialized.

      <B> References: </B>
      \verbatim
      [1]        Leiber, R.L (2010). Skeletal Muscle Structure, Function and 
                Plasticity: The Physiological Basis of Rehabilitiation 
                - Third Edition. Baltimore, WD: Lippincott Williams & Wilkins.
        \endverbatim        

      <B>Conditions:</B>
        \verbatim
            1)  0 <= concentricMinSlope < 1            
            2a) 1 < isometricMaxSlope
            2b) (maxEccentricVelocityForceMultiplier-1)/1 < isometricMaxSlope            
            3)  0 <= eccentricMinSlope < (maxEccentricVelocityForceMultiplier-1)/1

            4) 1 < maxEccentricVelocityForceMultiplier 

            5) 0 <= concentricCurviness <= 1            
            6) 0 <= eccentricCurviness <= 1
        \endverbatim

        <B>Computational Costs</B>
        \verbatim 
            ~8,200 flops
        \endverbatim

        <B> Default Parameter Values </B>
        \verbatim
            concentricMinSlope ...................  = 0.1 
            isometricMaxSlope  ...................  = 5
            eccentricMinSlope  ...................  = 0.1
            maxEccentricVelocityForceMultiplier...  = 1.8
            concentricCurviness ..................  = 0.1
            eccentricCurviness  ..................  = 0.75
        \endverbatim

    <B> Example </B>



    */
    ForceVelocityCurve( double concentricMinSlope, 
                        double isometricMaxSlope,
                        double eccentricMinSlope,
                        double maxEccentricVelocityForceMultiplier,
                        double concentricCurviness,
                        double eccentricCurviness,
                        const std::string muscleName);



   #ifndef SWIG
        ///default assignment operator
        ForceVelocityCurve& operator=(const ForceVelocityCurve &source);
        #endif
            ///a function that copies all of the properties and data members
            ///of this class
            void copyData(const ForceVelocityCurve &source);
        #ifndef SWIG
    #endif

    /**
    @returns    The slope of the force velocity curve at the maximum
                normalized contraction velocity (-1). 
    */
     double getConcentricMinSlope();

     /**
     @returns   The slope of the force velocity curve at a normalized 
                contraction velocity of 0.
     */
     double getIsometricMaxSlope();

     /**
     @returns   The slope of the force velocity curve at the maximum eccentric
                (lengthening) contraction velocity (1).
     */
     double getEccentricMinSlope();

     /**
     @returns   The value of the force velocity multiplier at the maximum 
                eccentric (lengthening) velocity.
     */
     double getMaxEccentricVelocityForceMultiplier();

     /**
     @returns   The value of the curviness of the concentric curve, where 0
                represents a nearly straight line segment, and 1 represents 
                a curve with the maximum bend possible given the 
                concentricMinSlope, and the isometricMaxSlope. 
     */
     double getConcentricCurviness();

     /**
     @returns   The value of the curviness of the eccentric curve, where 0
                represents a nearly straight line segment, and 1 represents 
                a curve with the maximum bend possible given the 
                eccentricMinSlope, and the isometricMaxSlope.
     */
     double getEccentricCurviness();

     /**
     @param aConcentricMinSlope 
        the slope of the force velocity curve at the maximum concentric 
        contraction velocity (1).
     */
     void setConcentricMinSlope(double aConcentricMinSlope);
     
     /**
     @param aIsometricMaxSlope 
        the slope of the force velocity curve at a contraction velocity of 0.
     */
     void setIsometricMaxSlope(double aIsometricMaxSlope);

     /**
     @param aEccentricMinSlope 
        the slope of the force velocity curve at the maximum eccentric 
        (lengthening) contraction velocity (1).
     */

     void setEccentricMinSlope(double aEccentricMinSlope);

     /**
     @param aMaxForceMultiplier 
        the value of the force velocity curve, or the value of the force 
        velocity multiplier, at the maximum eccentric (lengthening) velocity
     */
     void setMaxEccentricVelocityForceMultiplier(double aMaxForceMultiplier);

     /**
     @param aConcentricCurviness      
        The value of the curviness of the concentric curve, where 0 represents a 
        nearly straight line segment, and 1 represents a curve with the maximum 
        bend possible given the concentricMinSlope, and the isometricMaxSlope. 
     */
     void setConcentricCurviness(double aConcentricCurviness);

     /**
     @param aEccentricCurviness
        The value of the curviness of the eccentric curve, where 0 represents a 
        nearly straight line segment, and 1 represents a curve with the maximum 
        bend possible given the eccentricMinSlope, and the isometricMaxSlope.
     */
     void setEccentricCurviness(double aEccentricCurviness);

    /**
    Calculates the value of the curve evaluated at 'normFiberVelocity'. Note 
    that if the curve is out of date it is rebuilt 
    (at a cost of ~20,500 flops). 

    @param normFiberVelocity : the normalized velocity of the muscle fiber 
            (fiber_velocity (m/s) / 
            (optimal_fiber_length (m) * max_lengths_sec (1/s))

    @return the value of the curve evaluated at normFiberVelocity 

    <B>Computational Costs</B>
    \verbatim
        x in curve domain  : ~282 flops
        x in linear section:   ~5 flops
    \endverbatim

    */
    double calcValue(double normFiberVelocity) const;

    /**
    Calculates the derivative of the force-velocity multiplier w.r.t. 
    normalized fiber velocity. Note that if the curve is out of date it is 
    rebuilt (at a cost of ~20,500 flops).

    @param normFiberVelocity : the normalized velocity of the muscle fiber 
            (fiber_velocity (m/s) / 
            (optimal_fiber_length (m) * max_lengths_sec (1/s))

    @param order           : the order of the derivative. Only values of 0,1 and
                             2 are acceptable.

    @return the derivative of the force-velocity curve w.r.t. normalized 
            fiber velocity
    
    <B>Computational Costs</B>       
    \verbatim
        x in curve domain  : ~391 flops
        x in linear section:   ~2 flops       
    \endverbatim

    */
    double calcDerivative(double normFiberVelocity, int order) const;

    /**
       This function returns a SimTK::Vec2 that contains in its 0th element
       the lowest value of the curve domain, and in its 1st element the highest
       value in the curve domain of the curve. Outside of this domain the curve
       is approximated using linear extrapolation. Note that  if the curve is 
       out of date is rebuilt (which will cost ~20,500 flops).

       @return The minimum and maximum value of the domain, x, of the curve 
                  y(x). Within this range y(x) is a curve, outside of this range
                  the function y(x) is a C2 (continuous to the second 
                  derivative) linear extrapolation*/
    SimTK::Vec2 getCurveDomain() const;

    /**This function will generate a csv file with a name that matches the 
       curve name (e.g. "bicepfemoris_fiberForceVelocityCurve.csv");
       Note that  if the curve is out of date is rebuilt 
       (which will cost ~20,500 flops).
       
       @param path The full path to the location. Note '/' slashes must be used,
            and do not put a '/' after the last folder.

       The file will contain the following columns:
       
       \verbatim
       Col# 1, 2,     3,       4,  
            x, y, dy/dx, d2y/dx2,
       \endverbatim
       
       The curve will be sampled from its linear extrapolation region
       (the region with normalized fiber velocities < -1), through 
       the curve, out to the other linear extrapolation region
       (the region with normalized fiber velocities > 1). The width of 
       each linear extrapolation region is 10% of the entire range of x, or 
       0.1*(x1-x0).

       The curve is sampled quite densely: there are 200+20 rows    

       <B>Computational Costs</B>
       \verbatim
            ~194,800 flops
       \endverbatim

       <B>Example</B>
       To read the csv file with a header in from Matlab, you need to use 
       csvread set so that it will ignore the header row. This is accomplished
       by using the extra two numerical arguments for csvread to tell the 
       function to begin reading from the 1st row, and the 0th index (csvread
       is 0 indexed). This is necessary to skip reading in the text header
       \verbatim
        data=csvread('bicepfemoris_fiberForceVelocityCurve.csv',1,0);
       \endverbatim

       */
       void printMuscleCurveToCSVFile(const std::string& path) const;

protected:
    /*
    This object extends the ModelComponent interface so that we can make use
    of the 'createSystem' function, which we are using to create the 
    curve (using MuscleCurveFunctionFactory), which is an 
    expensive operation, just once prior to simulation. 
    
    Thus the user is allowed to set the properties of this curve until just 
    before the simulation begins. Just prior to the simulation starts 
    'createSystem' is called, and then this object will build the 
    MuscleCurveFunction that defines the curve the user requested
    */

    ///ModelComponent Interface required function
  	void setup(Model& aModel) OVERRIDE_11;
    ///ModelComponent Interface required function
	void initState(SimTK::State& s) const OVERRIDE_11;
    /**
    ModelComponent is being used for this one function, which is called just
    prior to a simulation beginning. This is the ideal time to actually
    create the curve because

    \li The curve parameters cannot change anymore
    \li This function is only called just prior to simulation, so the expensive
        task of creating the curve will only be done when it is absolutely 
        necessary

    */
	void createSystem(SimTK::MultibodySystem& system) const OVERRIDE_11;

    ///ModelComponent Interface required function
    void setDefaultsFromState(const SimTK::State& state){};
    

private:
  void setNull();
  void addProperties();

  /**
        This function will take all of the current parameter values and use 
        them to build a curve.

        <B>Computational Costs</B>
        \verbatim 
            Curve Construction Costs :   ~20,500 flops
        \endverbatim

    */
  void buildCurve();


  MuscleCurveFunction m_curve;
  bool m_curveUpToDate;
};

}

#endif //OPENSIM_ForceVelocityCurve_h__