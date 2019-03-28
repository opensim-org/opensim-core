#ifndef _OPENSIM_SMITH_2016_LIGAMENT_H_
#define _OPENSIM_SMITH_2016_LIGAMENT_H_
/* -------------------------------------------------------------------------- *
 *                     OpenSim:  Smith2016Ligament.h                          *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2019 Stanford University and the Authors                *
 * Author(s): Colin Smith                                                     *
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

#include <OpenSim/Simulation/Model/Force.h>
#include <OpenSim/Simulation/Model/GeometryPath.h>



namespace OpenSim {

//==============================================================================
//                         Smith2016LigamentModel
//==============================================================================
/**
This class implements a nonlinear spring ligament model as described in
Smith et al.\ (2016). This model is based on the formulation orginally 
proposed by Wismans et al.\ (1980) and Blankevoort et al.\ (1991). The ligament
is represented as a nonlinear spring with a quadratic "toe" region and a linear
region. Additionally, the ligament includes a normalized damping force.   
\image html fig_Smith2016Ligament.png
<B>Governing Equations</B>
Spring Force:

Damping Force:

Total Force:

An equilibrium model assumes that the forces generated
by the fiber and tendon are equal:
\f[
 f_{ISO}\Big(\mathbf{a}(t) \mathbf{f}_L(\hat{l}_{CE}) \mathbf{f}_V(\hat{v}_{CE})
+ \mathbf{f}_{PE}(\hat{l}_{CE}) + \beta \hat{v}_{CE}\Big) \cos \phi
-  f_{ISO}\mathbf{f}_{SE}(\hat{l}_{T}) = 0
\f]
\image html fig_Millard2012EquilibriumMuscle.png
This model can be simulated in several configurations by adjusting three flags:
\li ignore_tendon_compliance: set to <I>true</I> to make the tendon rigid. This
assumption is usually reasonable for short tendons, and can result in a
performance improvement by eliminating high-frequency dynamics and removing the
fiber length from the state vector.
\li ignore_activation_dynamics: set to <I>true</I> to use the excitation input
as the activation signal. This results in faster simulations by reducing the
size of the state vector.
\li fiber_damping: set to a value greater than 0.001 to include fiber damping in
the model. The addition of damping reduces simulation time while allowing the
muscle model to be more physiological (it can have an activation of zero, its
active-force-length curve can go to zero, and its force-velocity curve can be
asymptotic).
<B>Elastic Tendon, No Fiber Damping</B>
The most typical configuration used in the literature is to simulate a muscle
with an elastic tendon, full fiber dynamics, and activation dynamics. The
resulting formulation suffers from three singularities: \f$\mathbf{a}(t)
\rightarrow 0\f$, \f$\phi \rightarrow 90^\circ\f$, and
\f$ \mathbf{f}_L(\hat{l}_{CE}) \rightarrow 0 \f$. These situations are all
handled in this model to ensure that it does not produce singularities and does
not result in intolerably long simulation times.
Numerical singularities arise from the manner in which the equilibrium equation
is rearranged to yield an ordinary differential equation (ODE). The above
equation is rearranged to isolate \f$ \mathbf{f}_V(\hat{v}_{CE}) \f$. We then
invert to solve for \f$ \hat{v}_{CE} \f$, which is then numerically integrated
during a simulation:
\f[
 \hat{v}_{CE} = \mathbf{f}_V ^{-1} \Big(
 \frac{ ( \mathbf{f}_{SE}(\hat{l}_{T}) ) /
 \cos \phi
  -  \mathbf{f}_{PE}(\hat{l}_{CE}) }
  { \mathbf{a}(t) \mathbf{f}_L(\hat{l}_{CE})} \Big)
\f]
The above equation becomes numerically stiff when terms in the denominator
approach zero (when \f$\mathbf{a}(t) \rightarrow 0\f$, \f$\phi
\rightarrow 90^\circ\f$, or \f$ \mathbf{f}_L(\hat{l}_{CE}) \rightarrow 0 \f$)
or, additionally, when the slope of \f$\mathbf{f}_V ^{-1}\f$ is steep (which
occurs at fiber velocities close to the maximum concentric and maximum
eccentric fiber velocities).
Singularities can be managed by ensuring that the muscle model is always
activated (\f$\mathbf{a}(t) > 0\f$), the fiber will stop contracting when a
pennation angle of 90 degrees is approached (\f$\phi < 90^\circ\f$), and the
fiber will also stop contracting as its length approaches a lower bound
(\f$ \hat{l}_{CE} > lowerbound\f$), which is typically around half the fiber's
resting length (to ensure \f$ \mathbf{f}_L(\hat{l}_{CE}) > 0 \f$). The fiber is
prevented from reaching unphysiological lengths or its maximum pennation angle
using a unilateral constraint. Additionally, the force-velocity curve is
modified so that it is invertible.
When an elastic tendon without fiber damping is selected, the minimum
active-force-length value is set to 0.1, the minimum permissible activation is
set to 0.01, and the maximum permissible pennation angle is set to acos(0.1) or
84.3 degrees. This is done as a convenience for the user to prevent the model
from taking an unreasonable amount of time to simulate.
<B>(Rigid Tendon) or (Elastic Tendon with Fiber Damping)</B>
Neither of these formulations has any singularities. The lower bound of the
active-force-length curve can be zero (min( \f$ \mathbf{f}_L(\hat{l}_{CE})) = 0
\f$), activation can be zero (i.e., the muscle can be turned off completely),
and the force-velocity curve need not be invertible.
The rigid tendon formulation removes the singularities by ignoring the
elasticity of the tendon. This assumption is reasonable for many muscles, but it
is up to the user to determine whether this assumption is valid.
The formulation that uses an elastic tendon with fiber damping removes
singularities by solving the equilibrium equation with Newton's method. This is
possible because the partial derivative of the equilibrium equation with respect
to fiber velocity is always positive if \f$ \beta > 0\f$ and, thus, Newton's
method can find a solution to the equilibrium equation.
When either of these singularity-free formulations is selected, the minimum
active-force-length value and the minimum permissible activation are set to
zero. This is done as a convenience for the user, as these changes make the
results of the model more realistic yet incur no performance penalty. The
maximum pennation angle is left as acos(0.1) or 84.3 degrees, as allowing higher
pennation angles results in an increasingly stiff fiber velocity state as
pennation angle increases.

<B>Reference</B>
	Smith, C.R., Lenhart, R.L., Kaiser, J., Vignos, M.F. and Thelen, D.G.,
	(2016). Influence of ligament properties on tibiofemoral mechanics
	in walking. J Knee Surg, 29(02), 99-106.
	
	Wismans, J.A.C., Veldpaus, F., Janssen, J., Huson, A. and Struben, P., 
	(1980). A three-dimensional mathematical model of the knee-joint. 
	J Biomech, 13(8), 677-685.\n\n"

	Blankevoort, L. and Huiskes, R., (1991).
	Ligament-bone interaction in a three-dimensional model of the knee.
	J Biomech Eng, 113(3), 263-269
@author Colin Smith

*/

	class OSIMSIMULATION_API Smith2016Ligament : public Force  {
		OpenSim_DECLARE_CONCRETE_OBJECT(Smith2016Ligament, Force)

	public:
	//=============================================================================
	// PROPERTIES
	//=============================================================================

		OpenSim_DECLARE_UNNAMED_PROPERTY(GeometryPath,
			"The set of points defining the path of the ligament")
		OpenSim_DECLARE_PROPERTY(linear_stiffness, double,
			"Slope of the linear portion of the force-strain curve of ligament")
		OpenSim_DECLARE_PROPERTY(transition_strain, double,
			"Strain at which ligament force-strain curve transitions from"
			"quadratic to linear. Commonly 0.06 in literature.")
		OpenSim_DECLARE_PROPERTY(normalized_damping_coefficient, double,
			"Coefficient for normalized damping of ligament")
		OpenSim_DECLARE_PROPERTY(slack_length, double,
			"Length at which ligament begins developing tension")

	//=============================================================================
	// OUTPUTS
	//=============================================================================

		OpenSim_DECLARE_LIST_OUTPUT(dynamic_quantities, double, getDynamicQuantities, SimTK::Stage::Dynamics)
	


	//=============================================================================
	// METHODS
	//=============================================================================
	public:
		// Default Constructor
		Smith2016Ligament();
		Smith2016Ligament(PhysicalFrame& frame1, SimTK::Vec3 point1,
			PhysicalFrame& frame2, SimTK::Vec3 point2);
		Smith2016Ligament(PhysicalFrame& frame1, SimTK::Vec3 point1,
			PhysicalFrame& frame2, SimTK::Vec3 point2,
			double linear_stiffness, double reference_strain);

		//-------------------------------------------------------------------------
		//Outputs
		//-------------------------------------------------------------------------
		double getDynamicQuantities(const SimTK::State& state,
			const std::string& channel) const {
			if (channel == "force_spring") {
				return getCacheVariableValue<double>(state, "force_spring");
			}
			else if (channel == "force_damping") {
				return getCacheVariableValue<double>(state, "force_damping");
			}
			else if (channel == "force_total") {
				return getCacheVariableValue<double>(state, "force_total");
			}
			else if (channel == "length") {
				return getCacheVariableValue<double>(state, "length");
			}
			else if (channel == "lengthening_speed") {
				return getCacheVariableValue<double>(state, "lengthening_speed");
			}
			else if (channel == "strain") {
				return getCacheVariableValue<double>(state, "strain");
			}
			else if (channel == "strain_rate") {
				return getCacheVariableValue<double>(state, "strain_rate");
			}
			else {
				OPENSIM_THROW(Exception, "Output: " + channel + " is not a valid output for Smith2016Ligament.");
			}
		}


		//--------------------------------------------------------------------------
		// SET
		//--------------------------------------------------------------------------
		void setSlackLengthFromReferenceStrain(double referenceStrain, SimTK::State state);
		void setSlackLengthFromReferenceForce(double referenceForce, SimTK::State state);

		//--------------------------------------------------------------------------
		// GET
		//--------------------------------------------------------------------------
		double getTension(const SimTK::State& s) const;

		//--------------------------------------------------------------------------
		// COMPUTATIONS
		//--------------------------------------------------------------------------
		virtual double computeMomentArm(const SimTK::State& s, Coordinate& aCoord) const ;
		void computeForce(const SimTK::State& s,
								  SimTK::Vector_<SimTK::SpatialVec>& bodyForces,
								  SimTK::Vector& generalizedForces) const override;
		double computePotentialEnergy(const SimTK::State& state) const override;
				
		
		double computeSpringForce(double strain) const;
		double computeSpringStrain(double force) const;
		double computeReferenceForce(const SimTK::State& state) const;
		double computeReferenceStrain(const SimTK::State& state) const;
		double computeReferenceLength(SimTK::State state) const;
		
		//--------------------------------------------------------------------------
		// SCALE
		//--------------------------------------------------------------------------

		void extendPostScale(const SimTK::State& s, const ScaleSet& scaleSet) override;

		//-----------------------------------------------------------------------------
		// REPORTING
		//-----------------------------------------------------------------------------
		OpenSim::Array<std::string> getRecordLabels() const override;
		OpenSim::Array<double> getRecordValues(const SimTK::State& state) const override;
		
		//void printPropertiesToConsole();

	protected:

		void extendFinalizeFromProperties() override;
		void extendAddToSystem(SimTK::MultibodySystem& system) const override;

	private:
		void setNull();
		void constructProperties();

//=============================================================================
	};	// END of class Smith2016Ligament
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // _OPENSIM_SMITH_2016_LIGAMENT_H_
