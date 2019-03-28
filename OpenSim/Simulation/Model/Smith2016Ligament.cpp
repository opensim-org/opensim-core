/* ---------------------------------------------------------------------------- *
 *                             Smith2016Ligament.cpp                            *
 * ---------------------------------------------------------------------------- *

 * ---------------------------------------------------------------------------- */

#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/Model/PointForceDirection.h>
#include "Smith2016Ligament.h"

using namespace SimTK;
using namespace OpenSim;


//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________
/**
 * Default constructor
 */
Smith2016Ligament::Smith2016Ligament() : Force()
{
    constructProperties();
	setNull();
}
Smith2016Ligament::Smith2016Ligament(PhysicalFrame& frame1, Vec3 point1,
	PhysicalFrame& frame2, Vec3 point2) : Smith2016Ligament()
{
	upd_GeometryPath().appendNewPathPoint("p1", frame1, point1);
	upd_GeometryPath().appendNewPathPoint("p2", frame2, point2);
}

Smith2016Ligament::Smith2016Ligament(PhysicalFrame& frame1, Vec3 point1,
	PhysicalFrame& frame2, Vec3 point2,
	double linear_stiffness, double slack_length) :
	Smith2016Ligament(frame1, point1, frame2, point2)
{
	set_linear_stiffness(linear_stiffness);
	set_slack_length(slack_length);	
}
//_____________________________________________________________________________




void Smith2016Ligament::setNull()
{
	setAuthors("Colin Smith");
	setReferences(
	"Smith, C.R., Lenhart, R.L., Kaiser, J., Vignos, M.F. and Thelen, D.G.,"
	"(2016). Influence of ligament properties on tibiofemoral mechanics"
	" in walking. J Knee Surg, 29(02), 99-106.\n\n"
	
	"Wismans, J.A.C., Veldpaus, F., Janssen, J., Huson, A. and Struben, P.," 
	"(1980). A three-dimensional mathematical model of the knee-joint. "
	"J Biomech, 13(8), 677-685.\n\n"

	"Blankevoort, L. and Huiskes, R., (1991)."
	"Ligament-bone interaction in a three-dimensional model of the knee."
	"J Biomech Eng, 113(3), 263-269"
	);
}

void Smith2016Ligament::constructProperties()
{
	constructProperty_GeometryPath(GeometryPath());
	constructProperty_linear_stiffness(0.0);
	constructProperty_transition_strain(0.06);
	constructProperty_normalized_damping_coefficient(0.003);
	constructProperty_slack_length(0.0);
	
}

void Smith2016Ligament::extendFinalizeFromProperties()
{
	Super::extendFinalizeFromProperties();

	//Add channels to output list
	auto& dyn_quan = updOutput("dynamic_quantities");
	dyn_quan.addChannel("force_spring");
	dyn_quan.addChannel("force_damping");
	dyn_quan.addChannel("force_total");
	dyn_quan.addChannel("length");
	dyn_quan.addChannel("lengthening_speed");
	dyn_quan.addChannel("strain");
	dyn_quan.addChannel("strain_rate");	


	//Check that properties are valid
	OPENSIM_THROW_IF_FRMOBJ(get_slack_length() < 0.,
        InvalidPropertyValue, getProperty_slack_length().getName(),
        "Slack Length cannot be less than 0");

	OPENSIM_THROW_IF_FRMOBJ(get_linear_stiffness() < 0.,
        InvalidPropertyValue, getProperty_linear_stiffness().getName(),
        "Linear Stiffness cannot be less than 0");

	OPENSIM_THROW_IF_FRMOBJ(get_normalized_damping_coefficient() < 0.,
        InvalidPropertyValue, getProperty_normalized_damping_coefficient().getName(),
        "Normalized Damping Coefficient cannot be less than 0");

	OPENSIM_THROW_IF_FRMOBJ(get_transition_strain() < 0.,
        InvalidPropertyValue, getProperty_transition_strain().getName(),
        "Transistion Strain cannot be less than 0");

	//Set Ligament Color
	GeometryPath& path = upd_GeometryPath();
    path.setDefaultColor(SimTK::Vec3(0.99, 0.42, 0.01));

}

void Smith2016Ligament::extendAddToSystem(SimTK::MultibodySystem& system) const
{
	Super::extendAddToSystem(system);

	addCacheVariable<double>("force_spring",0.0, SimTK::Stage::Dynamics);
	addCacheVariable<double>("force_damping", 0.0, SimTK::Stage::Dynamics);
	addCacheVariable<double>("force_total", 0.0, SimTK::Stage::Dynamics);
	addCacheVariable<double>("length", 0.0, SimTK::Stage::Dynamics);
	addCacheVariable<double>("lengthening_speed", 0.0, SimTK::Stage::Dynamics);
	addCacheVariable<double>("strain", 0.0, SimTK::Stage::Dynamics);
	addCacheVariable<double>("strain_rate", 0.0, SimTK::Stage::Dynamics);
}


void Smith2016Ligament::setSlackLengthFromReferenceStrain(double reference_strain, SimTK::State state) {
	double reference_length = computeReferenceLength(state);
	double slack_length = reference_length / (1.0 + reference_strain);
};

void Smith2016Ligament::setSlackLengthFromReferenceForce(double reference_force, SimTK::State state) {
	double reference_strain = computeSpringStrain(reference_force);
	setSlackLengthFromReferenceStrain(reference_strain, state);
};

double Smith2016Ligament::computeReferenceLength(SimTK::State state) const {

	SimTK::Stage stage = state.getSystemStage();

	
	//Pose model to reference coordinates
	for (Coordinate coord : getModel().getComponentList<Coordinate>()) {
		coord.setValue(state, coord.getDefaultValue());
	}
	
	getModel().realizePosition(state);

	return get_GeometryPath().getLength(state);
}


double Smith2016Ligament::computeReferenceStrain(const SimTK::State& state) const {
	double ref_length = computeReferenceLength(state);
	double ref_strain = ref_length / get_slack_length() - 1;

	return ref_strain;
}

double Smith2016Ligament::computeReferenceForce(const SimTK::State& state) const {
	double ref_strain = computeReferenceStrain(state);
	double ref_force = computeSpringForce(ref_strain);

	return ref_force;
}

double Smith2016Ligament::computeSpringForce(double strain) const 
{
	double force_spring;
	if (strain <= 0) {
		force_spring = 0;
	}
	else if ((strain > 0) && (strain < (get_transition_strain()))) {
		force_spring = 0.5 * get_linear_stiffness() * strain * strain / get_transition_strain();
	}
	else if (strain >= (get_transition_strain())) {
		force_spring = get_linear_stiffness() * (strain - (get_transition_strain() / 2));
	}
	return force_spring;
}

double Smith2016Ligament::computeSpringStrain(double force) const {
	double transistion_force = get_transition_strain() * get_linear_stiffness();

	double strain;
	if (force < 0.0) {
		strain = 0.0;
	}
	else if (force > transistion_force) {
		strain = force / get_linear_stiffness() + get_transition_strain() / 2;
	}
	else {
		strain = sqrt(2 * get_transition_strain() * force / get_linear_stiffness());
	}
	
	return strain;
}

//=============================================================================
// SCALING
//=============================================================================

void Smith2016Ligament::extendPostScale(const SimTK::State& s, const ScaleSet& scaleSet)
{
    Super::extendPostScale(s, scaleSet);

	GeometryPath& path = upd_GeometryPath();
	double slack_length = get_slack_length();
    if (path.getPreScaleLength(s) > 0.0)
    {
        double scaleFactor = path.getLength(s) / path.getPreScaleLength(s);
		set_slack_length(scaleFactor * slack_length);

        // Clear the pre-scale length that was stored in the GeometryPath.
        path.setPreScaleLength(s, 0.0);
    }
}

//=============================================================================
// GET
//=============================================================================

double Smith2016Ligament::getTension(const SimTK::State& s) const
{
	if (get_appliesForce()) {
		return getCacheVariableValue<double>(s, "force_total");
	}
	else {
		return 0.0;
	}
}

//=============================================================================
// COMPUTATION
//=============================================================================

double Smith2016Ligament::computeMomentArm(const SimTK::State& s, Coordinate& aCoord) const
{
	return get_GeometryPath().computeMomentArm(s, aCoord);
}

void Smith2016Ligament::computeForce(const SimTK::State& s,
							  SimTK::Vector_<SimTK::SpatialVec>& bodyForces,
							  SimTK::Vector& generalizedForces) const
{
	const GeometryPath& path = get_GeometryPath();
	
    double slack_length = get_slack_length();

	double length = path.getLength(s);
	double lengthening_speed = path.getLengtheningSpeed(s);

	double strain = (length - slack_length)/slack_length;
	double strain_rate = lengthening_speed /slack_length;

	// Calculate Spring Force
    double force_spring = computeSpringForce(strain);

	// Calculate Damping Force
	double force_damping = 0.0;
	if (strain > 0) {
		force_damping = get_linear_stiffness()*get_normalized_damping_coefficient()*strain_rate;
	}
	else {
		force_damping = 0.0;
	}

	//Phase-out damping as strain goes to zero with smooth-step function
	SimTK::Function::Step step(0, 1, 0, 0.01);
	SimTK::Vector in_vec(1,strain);
	force_damping = force_damping*step.calcValue(in_vec);

	// total force
	double force_total = force_spring + force_damping;

	// make sure the ligament is only acting in tension
	if (force_total < 0.0) {
		force_total = 0.0;
	}

	setCacheVariableValue<double>(s,"force_spring", force_spring);
	setCacheVariableValue<double>(s,"force_damping", force_damping);
	setCacheVariableValue<double>(s,"force_total", force_total);
	setCacheVariableValue<double>(s,"length", length);
	setCacheVariableValue<double>(s,"lengthening_speed", lengthening_speed);
	setCacheVariableValue<double>(s,"strain", strain);
	setCacheVariableValue<double>(s,"strain_rate", strain_rate);


	OpenSim::Array<PointForceDirection*> PFDs;
	path.getPointForceDirections(s, &PFDs);

	for (int i=0; i < PFDs.getSize(); i++) {
		applyForceToPoint(s, PFDs[i]->frame(), PFDs[i]->point(),
                          force_total*PFDs[i]->direction(), bodyForces);
	}
	
	for(int i=0; i < PFDs.getSize(); i++)
		delete PFDs[i];
}

double Smith2016Ligament::computePotentialEnergy(const SimTK::State& state) const {
	double strain = getCacheVariableValue<double>(state, "strain");
	double lin_stiff = get_linear_stiffness();
	double trans_strain = get_transition_strain();
	double slack_len = get_slack_length();

	if (strain < trans_strain) {
		return 1 / 6 * lin_stiff / trans_strain*pow(strain, 3);
	}
	else {
		return 1 / 6 * lin_stiff / trans_strain*pow(trans_strain, 3)+1/2*lin_stiff*strain*(strain-trans_strain);
	}

}


//-----------------------------------------------------------------------------
	// Reporting
	//-----------------------------------------------------------------------------
	/**
	 * Methods to query a Force for the value actually applied during simulation
	 * The names of the quantities (column labels) is returned by this first function
	 * getRecordLabels()
	 */
	OpenSim::Array<std::string> Smith2016Ligament::getRecordLabels() const {
		OpenSim::Array<std::string> labels("");
		labels.append(getName()+".force_spring");
		labels.append(getName()+".force_damping");
		labels.append(getName()+".force_total");
		labels.append(getName()+".length");
		labels.append(getName()+".lengthening_speed");
		labels.append(getName() + ".strain");
		labels.append(getName() + ".strain_rate");
		return labels;
	}

	/**
	 * Given SimTK::State object extract all the values necessary to report forces, application location
	 * frame, etc. used in conjunction with getRecordLabels and should return same size Array
	 */
	OpenSim::Array<double> Smith2016Ligament::getRecordValues(const SimTK::State& s) const {
		OpenSim::Array<double> values(1);

		// Report values
		values.append(getCacheVariableValue<double>(s, "force_spring"));
		values.append(getCacheVariableValue<double>(s, "force_damping"));
		values.append(getCacheVariableValue<double>(s, "force_total"));
		values.append(getCacheVariableValue<double>(s, "length"));
		values.append(getCacheVariableValue<double>(s, "lengthening_speed"));
		values.append(getCacheVariableValue<double>(s, "strain"));
		values.append(getCacheVariableValue<double>(s, "strain_rate"));
		return values;
	}

	/*void Smith2016Ligament::printPropertiesToConsole() {
		std::string def_prop = get_defining_slack_length_property();
		double lin_stiff = get_linear_stiffness();
		double ref_strain = get_reference_strain();
		double ref_force = get_reference_force();
		double slack_len = get_slack_length();
		double damp_c = get_normalized_damping_coefficient();
		double trans_strain = get_transition_strain();

		std::cout << "Smith2016Ligament: " << getName() << std::endl;
		std::cout << "==============================" << std::endl;
		std::cout << "Linear Stiffness: " << lin_stiff << std::endl;
		std::cout << "Defining Slack Length Property:" << def_prop << std::endl;
		std::cout << "Reference Strain: " << ref_strain << std::endl;
		std::cout << "Reference Force: " << ref_force << std::endl;
		std::cout << "Slack Length: " << slack_len << std::endl;
		std::cout << "Transition Strain: " << trans_strain << std::endl;
		std::cout << "Normalized Damping Coeff: " << damp_c<< std::endl;
		std::cout << std::endl;


	}*/
