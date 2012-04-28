#ifndef OPENSIM_FiberForceLengthCurve_h__
#define OPENSIM_FiberForceLengthCurve_h__

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
 This class serves as a serializable FiberForceLengthCurve, commonly used
 to model the parallel elastic element, for use in muscle models. 
 
 \image html fig_SerializableCurve_FiberForceLengthCurve.png
 
  @author Matt Millard

 */
class OSIMACTUATORS_API FiberForceLengthCurve : public ModelComponent {
OpenSim_DECLARE_CONCRETE_OBJECT(FiberForceLengthCurve, ModelComponent);

//class OSIMACTUATORS_API FiberForceLengthCurve : public ModelComponent {
//OpenSim_DECLARE_CONCRETE_OBJECT(FiberForceLengthCurve, ModelComponent);

public:

    ///Default constructor
    FiberForceLengthCurve();
    ///Default destructor
    ~FiberForceLengthCurve();
    ///Default constructor
    FiberForceLengthCurve(const FiberForceLengthCurve& source);

    /**
     Constructs a C2 continuous fiber force length curve. This curve has the 
     advantage of being C2 continuous which results in faster simulations when
     compared to the popular method of using a linearly extrapolated exponential
     curve to parameterize the fiber force length curve, which is only C0 
     continuous. Details to appear in Millard et al. 2012.
     
    
        @param strainAtOneNormForce
                    The fiber strain at which the fiber develops 1 unit of 
                    normalized force. The definition of strain used for this 
                    quantity is consistent with the Cauchy or engineering 
                    definition of strain: strain = (l-l0)/l0, where l is length,
                    and l0 is resting length. In this context 
                    strainAtOneNormForce = 0.6 means that the fiber will 
                    develop a tension of 1 normalized force when it is strained 
                    by 60% of its resting length, or equivalently is stretched 
                    to 1.60 times its resting length.

        @param stiffnessAtOneNormForce     
                The normalized stiffness (or slope) of the fiber curve when the 
                fiber is strained by strainAtOneNormForce under a load of 1 
                normalized unit of force.

        @param curviness    
                A dimensionless parameter between [0-1] that controls how 
                the curve is drawn: 0 will create a curve that is
                very close to a straight line segment while a value of 1 will 
                create a curve that smoothly fills the corner formed by the 
                linear extrapolation of 'stiffnessAtOneNormForce' and the
                x axis as shown in the figure.

        @param muscleName
                The name of the muscle this curve belongs to. This name is used
                to create the name of this curve, which is formed simply by 
                appending "_FiberForceLengthCurve" to the string in muscleName.
                This name is used for making intelligible error messages and 
                also for naming the XML version of this curve when it is 
                serialized.

      <B>Conditions:</B>
        \verbatim
            strainAtOneNormForce > 0
            stiffnessAtOneNormForce > 1/strainAtOneNormForce
            0 <= curviness <= 1
        \endverbatim

        <B>Computational Costs</B>
        \verbatim 
            ~174,100 flops
        \endverbatim

    <B> Default Parameter Values </B>

         The default curve has parameters that closely approximate the linearly 
         extrapolated exponential curve that is more commonly used in the 
         literature (Thelen, DG (2003). Adjustment of Muscle Mechanics Model 
         Parameters to Simulate Dynamic Contractions in Older Adults. 
         ASME J.Biomech. Eng., 125, 75-77). Although the linearily extrapolated 
         exponential curve is popular, this curve is not C1 continuous at the 
         engagement point. This lack of smoothness and incurrs a high simulation 
         cost during numerical integration. This curve is formed using a quintic
         Bezier curve, is C2 continuous reducing the required simulation time 
         (Millard et al. 2012, yet to appear).

         \verbatim
             strainAtOneNormForce    = 0.60, 
             stiffnessAtOneNormForce = 8.4 
             curviness               = 0.65)
         \endverbatim


        <B>Example:</B>
        @code
            FiberForceLengthCurve fpeCurve3(0.10,50,0.75,"soleus");
            double fpeVal  = fpeCurve3.calcValue(0.02);
            double dfpeVal = fpeCurve3.calcDerivative(0.02,1);
        @endcode
    */
    FiberForceLengthCurve( double strainAtOneNormForce, 
                            double stiffnessAtOneNormForce,
                            double curviness,
                            const std::string muscleName);



   #ifndef SWIG
        ///default assignment operator
        FiberForceLengthCurve& operator=(const FiberForceLengthCurve &source);
        #endif
            ///a function that copies all of the properties and data members
            ///of this class
            void copyData(const FiberForceLengthCurve &source);
        #ifndef SWIG
    #endif

    /**
    @returns    The fiber strain at which the Fiber fevelops 1 unit of 
                normalized force. The definition of strain used for this 
                quantity is consistent with the Cauchy or engineering 
                definition of strain: strain = (l-l0)/l0, where l is length,
                and l0 is resting length. In this context 
                strainAtOneNormForce = 0.6 means that 
                the fiber will develop a tension of 1 normalized force when 
                it is strained by 60% of its resting length, or 
                equivalently is stretched to 1.6 times its resting length.
    */
     double getStrainAtOneNormForce();

     /**
     @returns   The normalized stiffness (or slope) of the fiber curve 
                when the fiber is strained by strainAtOneNormForce
                under a load of 1 normalized unit of force.
     */
     double getStiffnessAtOneNormForce();

     /**
     @returns   A dimensionless parameter between [0-1] that controls how 
                the curve is drawn: 0 will create a curve that is
                very close to a straight line segment while a value of 1 will 
                create a curve that smoothly fills the corner formed by the 
                linear extrapolation of 'stiffnessAtOneNormForce' and the
                x axis as shown in the figure.
     */
     double getCurviness();

     /**
    @param aStrainAtOneNormForce     
                The fiber strain at which the fiber develops 1 unit of 
                normalized force. The definition of strain used for this 
                quantity is consistent with the Cauchy or engineering 
                definition of strain: strain = (l-l0)/l0, where l is length,
                and l0 is resting length. In this context 
                strainAtOneNormForce = 0.6 means that 
                the fiber will develop a tension of 1 normalized force when 
                it is strained by 60% of its resting length, or 
                equivalently is stretched to 1.6 times its resting length.
    */
     void setStrainAtOneNormForce(double aStrainAtOneNormForce);

     /**
     @param aStiffnessAtOneNormForce
                The normalized stiffness (or slope) of the fiber curve 
                when the fiber is strained by strainAtOneNormForce
                under a load of 1 normalized unit of force.
     */
     void setStiffnessAtOneNormForce(double aStiffnessAtOneNormForce);

     /**
     @param aCurviness  
                A dimensionless parameter between [0-1] that controls how 
                the curve is drawn: 0 will create a curve that is
                very close to a straight line segment while a value of 1 will 
                create a curve that smoothly fills the corner formed by the 
                linear extrapolation of 'stiffnessAtOneNormForce' and the
                x axis as shown in the figure.
     */
     void setCurviness(double aCurviness);

    /**
    Calculates the value of the curve evaluated at the desired normalized fiber
    length. Note that if the curve is out of date it is rebuilt 
    (at a cost of ~20,500 flops). 

    @param aNormLength: 
                The normalized fiber length used to evaluate the tendon force 
                length curve for the corresponding normalized force. Here 
                aNormLength = l/l0, where l is the length of the fiber and l0 
                is the resting length of the fiber.  Thus normalized length of 
                1.0 means the fiber is at its resting length.      

    @return the value of the normalized force generated by the fiber

    <B>Computational Costs</B>
    \verbatim
        x in curve domain  : ~282 flops
        x in linear section:   ~5 flops
    \endverbatim

    */
    double calcValue(double aNormLength) const;

    /**
    Calculates the derivative of the fiber force length curve w.r.t. 
    to the normalized fiber length. Note that if the curve is out of date it is 
    rebuilt (at a cost of ~20,500 flops).

    @param aNormLength: 
                The normalized fiber length used to evaluate the tendon force 
                length curve for the corresponding normalized force. Here 
                aNormLength = l/l0, where l is the length of the fiber and l0 
                is the resting length of the fiber.  Thus normalized length of 
                1.0 means the fiber is at its resting length.

    @param order: the order of the derivative. Only values of 0,1 and 2 are 
                  acceptable.

    @return the derivative of the normalized fiber force length curve w.r.t. 
        normalized fiber length

    <B>Computational Costs</B>       
    \verbatim
        x in curve domain  : ~391 flops
        x in linear section:   ~2 flops       
    \endverbatim

    */
    double calcDerivative(double aNormLength, int order) const;

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
       curve name (e.g. "bicepfemoris_FiberForceLengthCurve.csv");
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
        data=csvread('bicepfemoris_fiberFiberForceLengthCurve.csv',1,0);
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

#endif //OPENSIM_FiberForceLengthCurve_h__