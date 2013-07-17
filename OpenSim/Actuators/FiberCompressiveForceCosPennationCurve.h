#ifndef OPENSIM_FiberCompressiveForceCosPennationCurve_h__
#define OPENSIM_FiberCompressiveForceCosPennationCurve_h__
/* -------------------------------------------------------------------------- *
 *             OpenSim:  FiberCompressiveForceCosPennationCurve.h             *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2012 Stanford University and the Authors                *
 * Author(s): Matthew Millard                                                 *
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

#include <OpenSim/Actuators/osimActuatorsDLL.h>


// INCLUDE
#include <simbody/internal/common.h>
#include <OpenSim/Common/Function.h>
#include <OpenSim/Common/SmoothSegmentedFunctionFactory.h>
#include <OpenSim/Common/SmoothSegmentedFunction.h>

#ifdef SWIG
    #ifdef OSIMACTUATORS_API
        #undef OSIMACTUATORS_API
        #define OSIMACTUATORS_API
    #endif
#endif

namespace OpenSim {

/**
 This class serves as a serializable FiberCompressiveForceCosPennationCurve, 
 which is used to ensure that the pennation angle approaches but never reaches
 an angle of 90 degrees. Preventing the fibers from achivieving a pennation 
 angle of 90 degrees is important for equilibrium muscle models which
 have a singularity at this value.

 This curve is designed to work with the muscle model
 in such a way that it acts like a spring that the pennated muscle fibers 
 contact it as the fiber rotates (circuled in red). When the spring engages it 
 will exert a force on the fiber that will prevent it from shortening further, 
 thus preventing the pennation angle from reaching 90 degrees.
  
 Note that this object should be updated through the set methods provided. 
 These set methods will take care of rebuilding the curve correctly. If you
 modify the properties directly, the curve will not be rebuilt, and upon
 calling a function like calcValue, calcDerivative, or printCurveToCSVFile
 an exception will be thrown because the curve is out of date with its 
 properties.

 \image html fig_FiberCompressiveForceCosPennationCurve.png
 
  @author Matt Millard

 */
class OSIMACTUATORS_API FiberCompressiveForceCosPennationCurve : 
    public Function {OpenSim_DECLARE_CONCRETE_OBJECT(
                                FiberCompressiveForceCosPennationCurve, 
                                Function);

//class OSIMACTUATORS_API FiberCompressiveForceCosPennationCurve : public ModelComponent {
//OpenSim_DECLARE_CONCRETE_OBJECT(FiberCompressiveForceCosPennationCurve, ModelComponent);

public:
//==============================================================================
// PROPERTIES
//==============================================================================
    /** @name Property declarations
    These are the serializable properties associated with this class. **/
    /**@{**/
    OpenSim_DECLARE_PROPERTY(engagement_angle_in_degrees, double, 
        "Engagement angle of the compressive pennation spring in degrees");
    OpenSim_DECLARE_OPTIONAL_PROPERTY(stiffness_at_perpendicular, double, 
        "Stiffness of the curve at pennation angle of 90 degrees");
    OpenSim_DECLARE_OPTIONAL_PROPERTY(curviness, double, 
        "Fiber curve bend, from linear to maximum bend (0-1)");
    /**@}**/

//==============================================================================
// PUBLIC METHODS
//==============================================================================

    /** Default constructor creates an curve with the default property values,
    and assigns it a default name **/
    FiberCompressiveForceCosPennationCurve();

   
    /**
     Constructs a C2 continuous compressive fiber force cos pennation curve. The
     sole purpose of this curve is to prevent the pennation angle from reaching
     an angle of 90 degrees. Details to appear in Millard et al. 2012.
     
        @param engagementAngleInDegrees
                The pennation angle engagement angle of the fiber compressive
                force pennation curve. Making the spring engage too
                far from 90 degrees may unrealistically limit the force 
                production capability of the muscle. An engagement angle of 
                80 degrees is a good place to start.
    
        @param stiffnessAtPerpendicular
                This is the stiffness of the compressive elastic force length
                spring when the pennation angle reaches 90 degrees. Note that 
                the units of this stiffness are 
                (normalized force) / cos(engagmentAngleInDegrees). If the 
                engagement angle is 80 degrees, a good stiffness to start with
                is -2*(1/cosd(engagementAngleInDegrees))

        @param curviness    
                A dimensionless parameter between [0-1] that controls how 
                the curve is drawn: 0 will create a curve that is
                very close to a straight line segment while a value of 1 will 
                create a curve that smoothly fills the corner formed by the 
                linear extrapolation of 'stiffnessAtPerpendicularFiber' and the
                x axis as shown in the figure. A good curviness parameter value
                to start with is 0.5.

        @param muscleName
                The name of the muscle this curve belongs to. This name is used
                to create the name of this curve, which is formed simply by 
                appending "_FiberCompressiveForceCosPennationCurve" to the 
                string in muscleName. This name is used for making intelligible 
                error messages and also for naming the XML version of this curve 
                when it is serialized.

      <B>Conditions:</B>
        \verbatim
            0 < engagmentAngleInDegrees < 90
            stiffnessAtPerpendicular < -1/engagmentAngleInDegrees
            0 <= curviness <= 1
        \endverbatim

        <B>Computational Costs</B>
        \verbatim 
            ~174,100 flops
        \endverbatim

    <B> Default Parameter Values </B>

         \verbatim
             engagmentAngleInDegrees = 80 
         \endverbatim

    <B>Example:</B>

    */
    FiberCompressiveForceCosPennationCurve(double engagementAngleInDegrees, 
                                           double stiffnessAtPerpendicular,
                                           double curviness,
                                           const std::string& muscleName);

    /**
     Constructs a C2 continuous compressive fiber force cos pennation curve 
     using only the manditory property, engagmentAngleInDegrees. The
     sole purpose of this curve is to prevent the pennation angle from reaching
     an angle of 90 degrees. Details to appear in Millard et al. 2012.
     
    @param engagementAngleInDegrees
                The pennation angle engagement angle of the fiber compressive
                force pennation curve. Making the spring engage too
                far from 90 degrees may unrealistically limit the force 
                production capability of the muscle. An engagement angle of 
                80 degrees is a good place to start.
    
    @param muscleName
                The name of the muscle this curve belongs to. This name is used
                to create the name of this curve, which is formed simply by 
                appending "_FiberCompressiveForceCosPennationCurve" to the 
                string in muscleName. This name is used for making intelligible 
                error messages and also for naming the XML version of this curve 
                when it is serialized.
    
    <B> Optional Parameters </B>
        If the optional parameters have not yet been set, they are computed when
        functions getStiffnessAtPerpendicularInUse(), and getCurvinessInUse()
        are called. See the documentation for these functions for details

    <B>Conditions:</B>
        \verbatim
            0 < engagmentAngleInDegrees < 90
        \endverbatim

    <B>Computational Costs</B>
        \verbatim 
            ~174,100 flops
        \endverbatim

    <B> Default Parameter Values </B>
        

         \verbatim
             engagmentAngleInDegrees = 80 
         \endverbatim

    <B>Example:</B>

    */
    FiberCompressiveForceCosPennationCurve( double engagementAngleInDegrees,                                             
                                            const std::string& muscleName);

   
                       
    /**
    @returns    The pennation angle engagement angle of the fiber compressive
                force pennation curve. 
    */
    double getEngagementAngleInDegrees() const;


     /**
     @returns   This is the stiffness of the compressive elastic force length
                spring when the pennation angle reaches 90 degrees. If this 
                property has been set, the property value is returned. If this
                property is empty, then a value is computed and returned. The
                value is computed using the following:

                \verbatim
                stiffnessAtPerpendicular = -2 * 1/cosd(engagementAngleInDegrees)        
                \endverbatim

                where cosd is a cosine function that takes its argument in units
                of degrees

     */
     double getStiffnessAtPerpendicularInUse() const;


     /**
     @returns   A dimensionless parameter between [0-1] that controls how 
                the curve is drawn: 0 will create a curve that is
                very close to a straight line segment while a value of 1 will 
                create a curve that smoothly fills the corner formed by the 
                linear extrapolation of 'StiffnessAtPerpendicularFiber'.

                If this property is empty, then a value is computed and 
                returned. The value is computed using the following:

                \verbatim
                curviness = 0.1       
                \endverbatim
     */
     double getCurvinessInUse() const;

     /**
     @returns true if the internal fitting routine (which takes only one 
     argument, the engagementAngleInDegrees) is being used. False is returned if
     the user has set the optional parameters.
     */
     bool isFittedCurveBeingUsed() const;

     /**
    @param aEngagementAngleInDegrees
            Sets the pennation angle engagement angle of the fiber compressive
            force pennation curve. 
    <B>Cost </B>
     The curve is rebuilt at a cost of ~174,100 flops
    */
     void setEngagementAngleInDegrees(double aEngagementAngleInDegrees);

    
     /**
     @param aStiffnessAtPerpendicular
            This is the stiffness of the compressive elastic force length
            spring when the pennation angle reaches 90 degrees.

     @param aCurviness  
                A dimensionless parameter between [0-1] that controls how 
                the curve is drawn: 0 will create a curve that is
                very close to a straight line segment while a value of 1 will 
                create a curve that smoothly fills the corner formed by the 
                linear extrapolation of 'stiffnessAtOneNormForce' and the
                x axis as shown in the figure.

     <B>Cost </B>
     The curve is rebuilt at a cost of ~174,100 flops
     */
     void setOptionalProperties(double aStiffnessAtPerpendicular,
                                double aCurviness);

    /**
    Calculates the value of the curve evaluated at cosPennationAngle.     
    @param cosPennationAngle: The cosine of the fiber pennation angle  

    @return the normalized force generated by the compressive force element 
            

    <B>Computational Costs</B>
    \verbatim
        x in curve domain  : ~282 flops
        x in linear section:   ~5 flops
    \endverbatim

    */
    double calcValue(double cosPennationAngle) const;


	/** Implement the generic OpenSim::Function interface **/
    double calcValue(const SimTK::Vector& x) const OVERRIDE_11
    {
        return calcValue(x[0]);
    }

    /**
    Calculates the derivative of the fiber compressive force pennation angle 
    curve w.r.t. to cosPennationAngle. 

    @param cosPennationAngle: 
                The cosine of the fiber pennation angle  

    @param order: the order of the derivative. Only values of 0,1 and 2 are 
                  acceptable.

    @return the derivative of the fiber compressive force pennation angle 
    curve w.r.t. to cosPennationAngle

    <B>Computational Costs</B>       
    \verbatim
        x in curve domain  : ~391 flops
        x in linear section:   ~2 flops       
    \endverbatim

    */
    double calcDerivative(double cosPennationAngle, int order) const;

    /**     
    @param cosPennationAngle
                The cosine of the pennation angle
    
    @return Computes the normalized area under the curve. For this curve, 
            this quantity corresponds to the normalized potential energy stored 
            in the fiber compressive force cos pennation spring - simply 
            multiply this quantity by the number of NormForce
            (where NormForce corresponds to the number of
            Newtons that 1 normalized force corresponds to) to obtain 
            the potental energy stored in the fiber in units of Joules. Note 
            that NormDistance is omitted because the length dimension of this 
            curve is not normalized, only the force dimension.

    <B>Computational Costs</B>    

    \verbatim
        x in curve domain  : ~13 flops
        x in linear section: ~19 flops
    \endverbatim

    */
    double calcIntegral(double cosPennationAngle) const;


    /**
       This function returns a SimTK::Vec2 that contains in its 0th element
       the lowest value of the curve domain, and in its 1st element the highest
       value in the curve domain of the curve. Outside of this domain the curve
       is approximated using linear extrapolation.

       @return The minimum and maximum value of the domain, x, of the curve 
                  y(x). Within this range y(x) is a curve, outside of this range
                  the function y(x) is a C2 (continuous to the second 
                  derivative) linear extrapolation*/
    SimTK::Vec2 getCurveDomain() const;

    /**This function will generate a csv file with a name that matches the 
       curve name (e.g. "bicepfemoris_FiberCompressiveForceCosPennationCurve.csv").
      This function is not const to permit the curve to be rebuilt if it is out of
       date with its properties.
       
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
        data=csvread('bicepfemoris_FiberCompressiveForceCosPennationCurve.csv',1,0);
       \endverbatim

       */
       void printMuscleCurveToCSVFile(const std::string& path);

       void ensureCurveUpToDate();
    

private:
	/**
	//--------------------------------------------------------------------------
	<B> OpenSim::Function Interface </B>
	//--------------------------------------------------------------------------
	Create the underlying SimTK::Function that implements the calculations
	necessary for this curve. */
	SimTK::Function* createSimTKFunction() const OVERRIDE_11;

	void setNull();
	void constructProperties();

	/**
	  This function will take all of the current property values, and if they
	  have changed since the last time the curve was built, and build a curve.

	  <B>Computational Costs</B>
	  \verbatim 
	  Curve Construction Costs :   ~20,500 flops
	  \endverbatim

	 */
	void buildCurve( bool computeIntegral = false );




	SmoothSegmentedFunction m_curve;
	double m_stiffnessAtPerpendicularInUse;
	double m_curvinessInUse;
	bool  m_isFittedCurveBeingUsed;
	};

	}

#endif //OPENSIM_FiberCompressiveForceCosPennationCurve_h__
