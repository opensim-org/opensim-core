// rdMuscleZajac.cpp
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//	AUTHORS:  Frank C. Anderson
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


//=============================================================================
// INCLUDES
//=============================================================================
#include <RD/Tools/rdIO.h>
#include <RD/Tools/rdMath.h>
#include <RD/Tools/rdMtx.h>
#include <RD/Tools/rdPropertyDbl.h>
#include "rdMuscleZajac.h"
#include <RD/Simulation/Model/rdMuscle.h>

//=============================================================================
// STATICS
//=============================================================================
// MUSCLE CONSTANTS
const double rdMuscleZajac::C1 = 18.0;
const double rdMuscleZajac::C2 = 0.020833;
const double rdMuscleZajac::PE1 = 0.1;
const double rdMuscleZajac::SIG0 = 0.01;
const double rdMuscleZajac::LMNORMMAX = 1.6;
const double rdMuscleZajac::AS = 40;
const double rdMuscleZajac::AF = 133;
const double rdMuscleZajac::MS = 74;
const double rdMuscleZajac::MF = 111;
const double rdMuscleZajac::CONPE1 = 111;


// FORCE-LENGTH CURVE
const double rdMuscleZajac::flcoef[][6] = {
	0.29999999E-01, 0.24252515E-01, 0.83434001E-01,
	0.00000000E+00, 0.00000000E+00, 0.93781357E+01,
	0.32000002E-01, 0.36303282E-01, 0.11122108E+00,
	0.41680610E+00, 0.31260455E+01,-0.75772491E+02,
	0.35000000E-01, 0.52911408E-01, 0.53432360E-01,
	-0.21172471E+01,-0.22131449E+02, 0.22750180E+03,
	0.37999999E-01, 0.28045202E-01,-0.28610954E+00,
	0.20922267E+01, 0.53702496E+02,-0.21170753E+03,
	0.39999999E-01, 0.60531657E-01, 0.93712115E+00,
	0.70036678E+01,-0.16866692E+02, 0.43728191E+02,
	0.50000001E-01, 0.26319203E+00, 0.20176411E+01,
	0.44493580E+01,-0.22906296E+01, 0.16808680E+04,
	0.79999998E-01, 0.75483233E+00, 0.78267779E+01,
	0.78543747E+02, 0.55799872E+03,-0.57448760E+04,
	0.19183697E+00, 0.29395888E+01, 0.21393633E+02,
	-0.27984400E+02,-0.13569598E+04, 0.58263604E+04,
	0.45546898E+00, 0.43861423E+01,-0.31255426E+01,
	-0.13089108E+03, 0.58515930E+03,-0.59455225E+03,
	0.70598048E+00, 0.28589883E+01,-0.15461145E+02,
	-0.12730635E+01, 0.38697504E+03,-0.11397749E+04,
	0.83362931E+00, 0.11265945E+01,-0.87735367E+01,
	0.51263630E+02, 0.70501127E+01,-0.66309363E+03,
	0.88419741E+00, 0.58316940E+00,-0.29753122E+00,
	0.23672834E+02,-0.21398106E+03, 0.57005804E+03,
	0.92529106E+00, 0.66183126E+00, 0.41993487E+00,
	-0.80528736E+01,-0.23961737E+02, 0.64066841E+02,
	0.96850455E+00, 0.58837938E+00,-0.16397921E+01,
	-0.11595254E+02,-0.26061234E+01, 0.71818405E+02,
	0.99704933E+00, 0.21914154E+00,-0.38155444E+01,
	-0.90982914E+01, 0.21333344E+02, 0.22131031E+01,
	0.99242932E+00,-0.38540557E+00,-0.50597558E+01,
	-0.33110399E+01, 0.22071045E+02,-0.34520439E+02,
	0.94365722E+00,-0.10814385E+01,-0.52356844E+01,
	0.10403328E+01, 0.10564221E+02, 0.69808632E+02,
	0.84890056E+00,-0.17462429E+01,-0.45390658E+01,
	0.69600554E+01, 0.33833744E+02,-0.20842517E+03,
	0.71476674E+00,-0.22391374E+01,-0.28623760E+01,
	0.67190380E+01,-0.35641376E+02, 0.43575076E+03,
	0.55463004E+00,-0.25304043E+01,-0.11778954E+01,
	0.16581345E+02, 0.10960875E+03,-0.10678167E+04,
	0.38637313E+00,-0.24419289E+01, 0.18973714E+01,
	-0.16482311E+01,-0.24633047E+03, 0.23621240E+04,
	0.22976723E+00,-0.22695737E+01, 0.19977926E+01,
	0.37646801E+02, 0.54104352E+03,-0.41256050E+04,
	0.10375030E+00,-0.12674731E+01, 0.11730966E+02,
	-0.14353340E+01,-0.83415924E+03, 0.41862578E+04,
	0.59999999E-01,-0.29765821E+00, 0.16033850E+01,
	-0.37822041E+02, 0.56125873E+03,-0.24013950E+04,
	0.44000000E-01,-0.16014636E+00, 0.18906338E+01,
	0.51182389E+01,-0.23920692E+03, 0.11255194E+04,
	0.39999999E-01,-0.12160726E-01,-0.12969553E+00,
	-0.86472273E+01, 0.13596582E+03,-0.55925067E+03,
	0.37999999E-01,-0.38839772E-01, 0.10957568E+00,
	0.27547290E+01,-0.50451214E+02, 0.21464819E+03,
	0.35999998E-01,-0.26094167E-01,-0.48849393E-01,
	-0.11590136E+01, 0.21098116E+02,-0.88555939E+02,
	0.34000002E-01,-0.31802028E-01, 0.19576361E-01,
	0.53132880E+00,-0.84205570E+01, 0.31915079E+02,
	0.32000002E-01,-0.28935270E-01,-0.41427822E-02,
	-0.29570580E+00, 0.22177913E+01,-0.66533685E+01
};

// FORCE-VELOCITY CURVE
const double rdMuscleZajac::fvcoef[][6] = {
	-0.99995804E+00,  0.50985093E+01, -0.12840916E+02, 
	0.00000000E+00,  0.00000000E+00,  0.24692949E+03, 
	-0.73122555E+00,  0.35122344E+01, -0.12238060E+02, 
	0.96456804E+01,  0.77165466E+02, -0.28950901E+02, 
	-0.55601108E+00,  0.21686606E+01, -0.86916122E+01, 
	0.27806149E+02,  0.68118301E+02, -0.63705542E+03, 
	-0.44720095E+00,  0.14259807E+01, -0.34367480E+01, 
	0.19950750E+02, -0.13096150E+03,  0.33955386E+03, 
	-0.36830568E+00,  0.11281989E+01, -0.19364038E+01, 
	0.47419998E+00, -0.24850937E+02,  0.20577457E+03, 
	-0.30542454E+00,  0.88313645E+00, -0.19275559E+01, 
	0.22995307E+01,  0.39453590E+02, -0.20102814E+03, 
	-0.25678629E+00,  0.69233119E+00, -0.10624915E+01, 
	0.43102646E+01, -0.23367722E+02,  0.52318295E+02, 
	-0.21692032E+00,  0.59120220E+00, -0.67426765E+00, 
	0.51201826E+00, -0.70182557E+01,  0.46904053E+02, 
	-0.18254140E+00,  0.50964373E+00, -0.62824291E+00, 
	0.58964401E+00,  0.76392608E+01, -0.35213955E+02, 
	-0.15291579E+00,  0.44279686E+00, -0.42461100E+00, 
	0.11239138E+01, -0.33651037E+01,  0.35358219E+01, 
	-0.12667322E+00,  0.39987490E+00, -0.28411430E+00, 
	0.42075619E+00, -0.22601600E+01,  0.10132744E+02, 
	-0.10271295E+00,  0.36785719E+00, -0.23345713E+00, 
	0.25152662E+00,  0.90632361E+00, -0.15478793E+01, 
	-0.80560051E-01,  0.34238961E+00, -0.16883321E+00, 
	0.41764352E+00,  0.42261115E+00, -0.23799617E+01, 
	-0.59714071E-01,  0.32641086E+00, -0.86430468E-01, 
	0.43032908E+00, -0.32112655E+00, -0.83081322E+01, 
	-0.39558768E-01,  0.31970248E+00, -0.33553813E-01, 
	0.25510818E-01, -0.29174187E+01,  0.22803499E+02, 
	-0.19724984E-01,  0.31469795E+00, -0.41474979E-01, 
	0.18691723E+00,  0.42086711E+01, -0.24812429E+01, 
	-0.11086452E-03,  0.31562474E+00,  0.86154908E-01, 
	0.11421622E+01,  0.34332817E+01, -0.65490707E+02, 
	0.20220956E-01,  0.33813503E+00,  0.22088873E+00, 
	-0.55773860E+00, -0.17032518E+02,  0.10814697E+03, 
	0.41924343E-01,  0.35082772E+00, -0.18856723E-01, 
	-0.59137255E+00,  0.16763449E+02,  0.34940689E+02, 
	0.63922137E-01,  0.36057681E+00,  0.34845823E+00, 
	0.49643559E+01,  0.27682409E+02, -0.36143289E+03, 
	0.89109048E-01,  0.46176851E+00,  0.10456773E+01, 
	-0.22335031E+01, -0.85265312E+02,  0.43564413E+03, 
	0.12062342E+00,  0.51627433E+00, -0.30792767E+00, 
	-0.65324678E+01,  0.50873634E+02,  0.46494720E+03, 
	0.15131254E+00,  0.48638496E+00,  0.79470813E+00, 
	0.24347912E+02,  0.19616957E+03, -0.19854454E+04, 
	0.19186006E+00,  0.91114461E+00,  0.51103878E+01, 
	-0.41661053E+01, -0.42428186E+03,  0.18109329E+04, 
	0.26300505E+00,  0.12249467E+01, -0.11936491E+01, 
	-0.39496960E+02,  0.14163533E+03,  0.30391128E+04, 
	0.33031818E+00,  0.98306721E+00,  0.21399465E+01, 
	0.11462707E+03,  0.10913577E+04, -0.49505034E+04, 
	0.44003567E+00,  0.32819295E+01,  0.37125008E+02, 
	0.19408749E+03, -0.45567386E+03, -0.29708398E+03, 
	0.83032483E+00,  0.97293663E+01,  0.62111267E+02, 
	0.68564049E+02, -0.54851270E+03,  0.17552415E+04
};

//*************************************************** 
//* Ilse added the following section
//*************************************************** 

// VELOCITY-FORCE CURVE

const double rdMuscleZajac::vfcoef[][6] = {
	0.00000000E+00,  0.19490500E+00,  0.11604053E+00,
	0.00000000E+00,  0.00000000E+00,  0.17118695E+01,
	0.20667715E-01,  0.21896866E+00,  0.13315850E+00,
	0.17118214E+00,  0.85592270E+00, -0.28726175E+01,
	0.44123858E-01,  0.25272262E+00,  0.20714137E+00,
	0.22629277E+00, -0.58036578E+00,  0.77730355E+01,
	0.71713097E-01,  0.30250385E+00,  0.31793433E+00,
	0.77143145E+00,  0.33060975E+01, -0.76533756E+01,
	0.10616777E+00,  0.39862949E+00,  0.67119020E+00,
	0.13285358E+01, -0.52053666E+00,  0.69259363E+00,
	0.15402523E+00,  0.57098478E+00,  0.10454397E+01,
	0.11895815E+01, -0.17424472E+00,  0.56420404E+00,
	0.22275476E+00,  0.81534141E+00,  0.13974965E+01,
	0.11763034E+01,  0.10785336E+00,  0.51013916E+02,
	0.31995946E+00,  0.11560618E+01,  0.22669711E+01,
	0.63206925E+01,  0.25614452E+02, -0.11930303E+03,
	0.46592188E+00,  0.18418704E+01,  0.45069957E+01,
	0.46363611E+01, -0.34036224E+02,  0.23893833E+01,
	0.69643164E+00,  0.27473993E+01,  0.38796613E+01,
	-0.87390070E+01, -0.32841553E+02, -0.62542439E+01,
	0.99787807E+00,  0.31266701E+01, -0.77498108E+00,
	-0.22500851E+02, -0.35968632E+02,  0.29059869E+03,
	0.12796004E+01,  0.22980924E+01, -0.67773333E+01,
	-0.78290467E+01,  0.10932867E+03, -0.13840590E+03,
	0.14433545E+01,  0.10758772E+01, -0.39504786E+01,
	0.22061613E+02,  0.40126709E+02, -0.43857874E+03,
	0.15331247E+01,  0.88884521E+00,  0.68984365E+00,
	-0.57445703E+01, -0.17915956E+03,  0.77121613E+03,
	0.16129586E+01,  0.52345812E+00, -0.40709386E+01,
	-0.28794375E+00,  0.20644308E+03, -0.72966547E+03,
	0.16376544E+01,  0.16156906E+00,  0.93256897E+00,
	0.93236351E+01, -0.15838451E+03,  0.48042551E+03,
	0.16614262E+01,  0.23446929E+00, -0.96913165E+00,
	-0.59880857E+01,  0.81824844E+02, -0.22818460E+03,
	0.16750942E+01,  0.74207850E-01, -0.13792819E+00,
	0.39235618E+01, -0.32265957E+02,  0.86415871E+02,
	0.16826967E+01,  0.78473255E-01, -0.32656990E-01,
	-0.34129956E+00,  0.10941332E+02, -0.36170631E+02,
	0.16906085E+01,  0.87382466E-01,  0.15972510E+00,
	0.41820762E+00, -0.71437464E+01,  0.18469364E+02,
	0.17008324E+01,  0.11253329E+00,  0.41258883E-01,
	-0.59236693E+00,  0.20907981E+01, -0.50816641E+01,
	0.17120640E+01,  0.10883658E+00, -0.61818872E-01,
	-0.26421174E+00, -0.45000064E+00,  0.42816858E+01,
	0.17220629E+01,  0.88887662E-01, -0.12526552E+00,
	-0.16053502E-01,  0.16908103E+01, -0.70726466E+01,
	0.17297813E+01,  0.66580139E-01, -0.99359214E-01,
	-0.46984266E-01, -0.18454661E+01,  0.11912358E+02,
	0.17353332E+01,  0.43873392E-01, -0.10506071E+00,
	0.40604055E+00,  0.41106238E+01, -0.22952692E+02,
	0.17392575E+01,  0.40008530E-01,  0.33862963E-01,
	-0.24494012E+00, -0.73655701E+01,  0.38545902E+02,
	0.17430009E+01,  0.29243873E-01, -0.96097238E-01,
	0.66335094E+00,  0.11907093E+02, -0.58248142E+02,
	0.17462358E+01,  0.48428640E-01,  0.23485379E+00,
	-0.39853555E+00, -0.17216595E+02,  0.67698494E+02,
	0.17519840E+01,  0.48426915E-01, -0.24071532E+00,
	-0.51542318E+00,  0.16632149E+02, -0.47319916E+02,
	0.17550941E+01,  0.27689446E-01,  0.12938190E+00,
	0.14054815E+01, -0.70274992E+01,  0.14055183E+02


};
 



//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________
/**
 * Destructor.
 */
rdMuscleZajac::~rdMuscleZajac()
{
}
//_____________________________________________________________________________
/**
 * Default constructor.
 */
rdMuscleZajac::
rdMuscleZajac(int aQID,int aNX,int aNY,int aNYP) :
	rdForce(aQID,aNX,aNY,aNYP),
	_tRise(_proptRise.getValueDbl()),
	_tFall(_proptFall.getValueDbl()),
	_fmOpt(_propfmOpt.getValueDbl()),
	_lmOpt(_proplmOpt.getValueDbl()),
	_lmRecipOpt(_proplmRecipOpt.getValueDbl()),
	_alphaOpt(_propalphaOpt.getValueDbl()),
	_vmMax(_propvmMax.getValueDbl()),
	_ltSlack(_propltSlack.getValueDbl())
{	setNull();
}
//_____________________________________________________________________________
/**
 * Construct the actuator from an XML Element.
 *
 * @param aElement XML element.
 * @param aNX Number of controls.
 * @param aNY Number of states.
 * @param aNYP Number of pseudo-states.
 */
rdMuscleZajac::
rdMuscleZajac(DOMElement *aElement,int aNX,int aNY,int aNYP) :
	rdForce(aElement,aNX,aNY,aNYP),
	_tRise(_proptRise.getValueDbl()),
	_tFall(_proptFall.getValueDbl()),
	_fmOpt(_propfmOpt.getValueDbl()),
	_lmOpt(_proplmOpt.getValueDbl()),
	_lmRecipOpt(_proplmRecipOpt.getValueDbl()),
	_alphaOpt(_propalphaOpt.getValueDbl()),
	_vmMax(_propvmMax.getValueDbl()),
	_ltSlack(_propltSlack.getValueDbl())
{
	setNull();
//	updateFromXMLNode();
}
//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 * @param aActuator Actuator to be copied.
 */
rdMuscleZajac::
rdMuscleZajac(const rdMuscleZajac &aActuator) :
	rdForce(aActuator),
	_tRise(_proptRise.getValueDbl()),
	_tFall(_proptFall.getValueDbl()),
	_fmOpt(_propfmOpt.getValueDbl()),
	_lmOpt(_proplmOpt.getValueDbl()),
	_lmRecipOpt(_proplmRecipOpt.getValueDbl()),
	_alphaOpt(_propalphaOpt.getValueDbl()),
	_vmMax(_propvmMax.getValueDbl()),
	_ltSlack(_propltSlack.getValueDbl())
{
	setNull();
	copyData(aActuator);
}


//=============================================================================
// CONSTRUCTION
//=============================================================================
//_____________________________________________________________________________
/**
 * Set the data members of this actuator to their null values.
 */
void rdMuscleZajac::
setNull()
{
	// TYPE
	setType("rdMuscleZajac");
	setupProperties();

	// APPLIES FORCE
	_appliesForce = true;

	// CONTROL NAMES
	//setControlName(0,"excitation");

	// STATE NAMES
	//setStateName(0,"activation");
	//setStateName(1,"force");

	// DATA MEMBERS
	_x = 0.0;
	_a = 0.0;
	_force = 0.0;
	_width = 0.0;
	_widthSquared = 0.0;
	_ktRecip = 1.0;
	_ktLmOptRecip = 1.0;
	_lmRecipOpt=0.0;
	_lmt = (_ltSlack) + (_lmOpt);
	_speed = 0.0;

}


void rdMuscleZajac::
setupProperties()
{
_proptRise.setName("Rise time");
_proptRise.setValue(0.01);
_propertySet.append( &_proptRise );

 printf("in setupproperties%.16lfn\n",_tRise);
	
_proptFall.setName("Fall time");
_proptFall.setValue(0.05);
_propertySet.append( &_proptFall );

_propfmOpt.setName("Optimal fiber force");
_propfmOpt.setValue(1000.0);
_propertySet.append( &_propfmOpt );

_proplmOpt.setName("Optimal fiber length");
_proplmOpt.setValue(0.1);
_propertySet.append( &_proplmOpt );

_proplmRecipOpt.setName("Recip Optimal fiber length");
_proplmRecipOpt.setValue(0.1);
_propertySet.append( &_proplmRecipOpt );

_propalphaOpt.setName("Optimal fiber angle");
_propalphaOpt.setValue(0.0);
_propertySet.append( &_propalphaOpt );

_propvmMax.setName("Optimal fiber shortening velocity");
_propvmMax.setValue(10.0);
_propertySet.append( &_propvmMax );

_propltSlack.setName("Tendon slack length");
_propltSlack.setValue(0.1);
_propertySet.append( &_propltSlack );


}



//_____________________________________________________________________________
/**
 * Set up the serializable member variables.  This involves generating
 * properties and connecting local variables to those properties.
 */
//void rdMuscleZajac::
//etupSerializedMembers()
//{
//	_propertySet.append( new rdPropertyDbl("activation_rise_time",0.010) );
//	_tRise = &_propertySet.get("activation_rise_time")->getValueDbl();
//	_propertySet.append( new rdPropertyDbl("activation_fall_time",0.050) );
//	_tFall = &_propertySet.get("activation_fall_time")->getValueDbl();

//	_propertySet.append( new rdPropertyDbl("optimal_force",1000.0) );
//	_fmOpt = &_propertySet.get("optimal_force")->getValueDbl();

//	_propertySet.append( new rdPropertyDbl("optimal_fiber_length",0.100) );
//	_lmOpt = &_propertySet.get("optimal_fiber_length")->getValueDbl();

//	_propertySet.append( new rdPropertyDbl("optimal_pennation_angle",0.000) );
//	_alphaOpt = &_propertySet.get("optimal_pennation_angle")->getValueDbl();

//	_propertySet.append( new rdPropertyDbl("tendon_slack_length",0.100) );
//	_ltSlack = &_propertySet.get("tendon_slack_length")->getValueDbl();
//	_propertySet.append( new rdPropertyDbl("max_shortening_velocity",10.0) );
//	_vmMax = &_propertySet.get("max_shortening_velocity")->getValueDbl();
//}

//_____________________________________________________________________________
/**
 * Copy the member data of the specified actuator.
 */
void rdMuscleZajac::
copyData(const rdMuscleZajac &aActuator)
{
	// CONTROLS
	aActuator.getControls(&_x);

	// STATES
	double y[2];
	aActuator.getStates(y);
	_a = y[0];
	_force = y[1];

	// PROPERTIES
	setRiseTime(aActuator.getRiseTime());
	setFallTime(aActuator.getFallTime());
	setOptimalForce(aActuator.getOptimalForce());
	setOptimalFiberLength(aActuator.getOptimalFiberLength());
	setOptimalPennationAngle(aActuator.getOptimalPennationAngle());
	setTendonSlackLength(aActuator.getTendonSlackLength());
	setMaxShorteningVelocity(aActuator.getMaxShorteningVelocity());
}

//_____________________________________________________________________________
/**
 * Copy this actuator and return a pointer to the copy.
 * The copy constructor for this class is used.
 *
 * @return Pointer to a copy of this actuator.
 */
rdObject* rdMuscleZajac::
copy() const
{
	rdActuator *act = new rdMuscleZajac(*this);
	return(act);
}
//_____________________________________________________________________________
/**
 * Copy this actuator and modify the copy so that it is consistent
 * with a specified XML element node.
 *
 * The copy is constructed by first using the contructor for the DOMElement
 * in order to establish the relationship of the rdForce object with the
 * XML node.  Then, the assignment operator is used to set all data members
 * of the copy to the values of this object.  Finally, the data members of
 * the copy are updated from the DOMElment using updateObject().
 *
 * @param aElement XML element. 
 * @return Pointer to a copy of this actuator.
 */
rdObject* rdMuscleZajac::
copy(DOMElement *aElement) const
{
	// ESTABLISH RELATIONSHIP WITH XML NODE
	rdMuscleZajac *act = new
		rdMuscleZajac(aElement,getNX(),getNY(),getNYP());

	// ASSIGNMENT OPERATOR
	*act = *this;

	// UPDATE BASED ON NODE
	act->updateFromXMLNode();

	return(act);
}


//=============================================================================
// OPERATORS
//=============================================================================
//-----------------------------------------------------------------------------
// ASSIGNMENT
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Assignment operator.
 *
 * @return  Reference to the altered object.
 */
rdMuscleZajac& rdMuscleZajac::
operator=(const rdMuscleZajac &aActuator)
{
	// BASE CLASS
	rdForce::operator =(aActuator);

	// DATA
	copyData(aActuator);

	return(*this);
}


//=============================================================================
// GET AND SET
//=============================================================================
//-----------------------------------------------------------------------------
// CONTROLS
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the controls.  This actuator has 1 control: excitation.
 *
 * @param aX Control array- should have a length of at least 1.
 */
void rdMuscleZajac::
setControls(const double aX[])
{
	if(aX==NULL) return;
	_x = aX[0];
}
//_____________________________________________________________________________
/**
 * Get the controls.  This actuator has 1 control: excitation.
 *
 * @param rX Control array- should have a length of at least 1.
 */
void rdMuscleZajac::
getControls(double rX[]) const
{
	if(rX==NULL) return;
	rX[0] = _x;
}

//-----------------------------------------------------------------------------
// STATES
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the states.  This actuator has 2 states: activation and actuator
 * force.
 *
 * @param aY States array- should have a length of at least 2.
 */
void rdMuscleZajac::
setStates(const double aY[])
{
	if(aY==NULL) return;
	_a = aY[0];
	_force = aY[1];
	printf("PRINTING _force %.16lfn\n",_force);
}
//_____________________________________________________________________________
/**
 * Get the states.  This actuator has 2 states: activation and actuator
 * force.
 *
 * @param rY States array- should have a length of at least 2.
 */
void rdMuscleZajac::
getStates(double rY[]) const
{
	if(rY==NULL) return;
	rY[0] = _a;
	rY[1] = _force;
}

//-----------------------------------------------------------------------------
// RISE TIME
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the activation rise time.
 *
 * @param aTime Rise time of activation dynamics.
 */
void rdMuscleZajac::
setRiseTime(double aTime)
{
	(_tRise) = aTime;

	printf("PRINTING in setRiseTime%.16lfn\n",_tRise);
	
}
//_____________________________________________________________________________
/**
 * Get the activation rise time.
 *
 * @return Rise time of activation dynamics.
 */
double rdMuscleZajac::
getRiseTime() const
{
	return((_tRise));
}

//-----------------------------------------------------------------------------
// FALL TIME
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the activation fall time.
 *
 * @param aTime Fall time of activation dynamics.
 */
void rdMuscleZajac::
setFallTime(double aTime)
{
	(_tFall) = aTime;
	printf("PRINTING in setFallTime%.16lfn\n",_tFall);
	
}
//_____________________________________________________________________________
/**
 * Get the activation fall time.
 *
 * @return Fall time of activation dynamics.
 */
double rdMuscleZajac::
getFallTime() const
{
	return((_tFall));
}

//-----------------------------------------------------------------------------
// OPTIMAL FORCE
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the optimal force of the muscle.
 *
 * @param aForce Optimal muscle force.
 */
void rdMuscleZajac::
setOptimalForce(double aForce)
{
	(_fmOpt) = aForce;

	printf("PRINTING in setOptimalForce%.16lfn\n",_fmOpt);
}
//_____________________________________________________________________________
/**
 * Get the optimal force of the muscle.
 *
 * @return Optimal muscle force.
 */
double rdMuscleZajac::
getOptimalForce() const
{
	return((_fmOpt));
}

//-----------------------------------------------------------------------------
// OPTIMAL FIBER LENGTH
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the optimal muscle fiber length.
 *
 * @param aLength Optimal muscle fiber length in meters- should be positive.
 */
void rdMuscleZajac::
setOptimalFiberLength(double aLength)
{
	(_lmOpt) = aLength;
	(_lmRecipOpt) = 1/aLength;
	printf("PRINTING in setOptimalfiberlength%.16lfn\n",_lmOpt);
	printf("PRINTING in setOptimalfiberlength _lmRecipOpt %.16lfn\n",_lmRecipOpt);
}
//_____________________________________________________________________________
/**
 * Get the optimal muscle fiber length.
 *
 * @return Optimal muscle fiber length.
 */
double rdMuscleZajac::
getOptimalFiberLength() const
{
	return((_lmOpt));
}

//-----------------------------------------------------------------------------
// OPTIMAL PENNATION ANGLE
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the optimal muscle fiber pennation angle.
 *
 * Note that during a simulation, the pennation angle of the muscle fibers
 * changes.  The optimal pennation angle is the pennation angle when the
 * muscle is at its optimal muscle fiber length.
 *
 * The width of the muscle is assumed to be constant.
 * The width is computed from the optimal pennation angle and the
 * optimal muscle fiber length (width = lmOpt * sin(pennation)) every time
 * this method is called.
 *
 * @param aAngleDeg Optimal pennation angle in degrees- should be positive.
 */
void rdMuscleZajac::
setOptimalPennationAngle(double aAngleDeg)
{
	(_alphaOpt) = rdMath::DTR*aAngleDeg;
	_width = (_lmOpt) * sin((_alphaOpt));
	_widthSquared = _width * _width;

	printf("PRINTING in setOptimalPennationAngle%.16lfn\n",_alphaOpt);
	printf("PRINTING in setOptimalPennationAngle%.16lfn\n",_width);

}
//_____________________________________________________________________________
/**
 * Get the optimal muscle fiber pennation angle.
 *
 * @return Optimal muscle fiber pennation angle in degrees.
 */
double rdMuscleZajac::
getOptimalPennationAngle() const
{
	return(rdMath::RTD*(_alphaOpt));
}

//-----------------------------------------------------------------------------
// MAXIMUM SHORTENING VELOCITY
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the maximum shortening velocity of the muscle.
 *
 * @param aVelocity Maximum shortening velocity in optimal muscle fiber
 * lengths per second- should be positive.
 */
void rdMuscleZajac::
setMaxShorteningVelocity(double aVelocity)
{
	(_vmMax) = aVelocity;
	printf("PRINTING in setMaxShorteningVelocity%.16lfn\n",_vmMax);
}
//_____________________________________________________________________________
/**
 * Get the maximum shortening velocity of the muscle.
 *
 * @return Maximum shortening velocity in optimal muscle fiber lengths
 * per second- should be positive.
 */
double rdMuscleZajac::
getMaxShorteningVelocity() const
{
	return((_vmMax));
}

//-----------------------------------------------------------------------------
// TENDON SLACK LENGTH
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the slack length of the tendon.
 *
 * @param aLength Tendon slack length in meters.
 */
void rdMuscleZajac::
setTendonSlackLength(double aLength)
{
	(_ltSlack) = aLength;
	_ktRecip = _ltSlack/37.5;
	_ktLmOptRecip = (1/_lmOpt)*_ktRecip;
	printf("PRINTING in setTendonSlackLength%.16lfn\n",_ltSlack);
	printf("PRINTING in setTendonSlackLength _ktRecip %.16lfn\n",_ktRecip);

}

/**
 * Set the maximum shortening velocity of the muscle.
 *
 * @param aVelocity Maximum shortening velocity in optimal muscle fiber
 * lengths per second- should be positive.
 */
void rdMuscleZajac::
setShorteningVelocity(double aSpeed)
{
	(_speed) = aSpeed;
	printf("PRINTING in setShorteningVelocity%.16lfn\n",_speed);
}
/**
 * Set the actuatorlength of the muscle.
 *
 */
void rdMuscleZajac::
setActuatorLength(double aLength)
{
	(_lmt) = aLength;
	printf("PRINTING in setActuatorLength%.16lfn\n",_lmt);
}

// 


//=============================================================================
// COMPUTATIONS
//=============================================================================

//_____________________________________________________________________________
/**
 * Set the activation level of this actuator equal to the neural excitation.
 */
void rdMuscleZajac::
promoteControlsToStates(const double aX[],double aDT)
{
	
       if(aX==NULL) return;
	if(aDT<=0) {
		_a = aX[0];
	} else {
		_a = rdMuscle::EstimateActivation((_tRise),(_tFall),_a,aX[0],aDT);
	}
}

//_____________________________________________________________________________
/**
 * Compute all quantities necessary for applying the actuator force to the
 * model.
 */
void rdMuscleZajac::
computeActuation()
{
	if(_model==NULL) return;

	// FORCE ACTUATION
	rdForce::computeActuation();

	// ACTUATOR SHORTENING VELOCITY
	// The shortening velocity is simply the speed of the force, which is
	// calculated in rdForce::computeActuation();
///	_lmtSpeed = _speed;

	// ACTUATOR LENGTH
	// The assumption is that the actuator length is the straight-line
	// distance from pointA to pointB.
	double r[3];
	computeLineOfAction(r);
	_lmt = rdMtx::Magnitude(3,r);

//?
	// ACTUATOR FORCE
	// Actuator force is a state, so _force is set in setStates().
}

//_____________________________________________________________________________
/**
 * Compute the time derivatives of the states for this actuator.
 *
 * @param rDYDT Time derivatives of the states-- should have a length of at
 * least the value returned by getNY().
 * @see getNY()
 */
void rdMuscleZajac::
computeStateDerivatives(double rDYDT[])
{
	printf("PRINTING rDYDT[0] in  computeStateDerivatives%.16lfn\n",rDYDT[0]);
	printf("PRINTING rDYDT[1] in  computeStateDerivatives%.16lfn\n",rDYDT[1]);
	if(rDYDT==NULL) return;

// Is this sufficient-relation with mmatvdot?
	// ACTIVATION DYNAMICS
	
	printf("PRINTING _tRise in  computeStateDerivatives%.16lfn\n", _tRise);
	printf("PRINTING _tFall in  computeStateDerivatives%.16lfn\n",_tFall);
	printf("PRINTING _x in  computeStateDerivatives%.16lfn\n",_x);
	printf("PRINTING _a in  computeStateDerivatives%.16lfn\n",_a);

	rDYDT[0] = rdMuscleZajac::computeDADT((_tRise),(_tFall),_x,_a);

        printf("PRINTING rDYDT[0] in  computeStateDerivatives na berekening %.16lfn\n",rDYDT[0]);

	// MUSCLE-TENDON DYNAMICS
	rDYDT[1] = computeDFDT();
	printf("PRINTING rDYDT[1] in computeStateDerivatives na berekening %.16lfn\n",rDYDT[1]); 
}
//_____________________________________________________________________________
/**
 * Compute the time derivative of an activation level given its excitation
 * signal, a rise-time, and a fall-time.
 * This method represents the rise or fall using a simple 1st order
 * differential equation which is linear in x and a.  The time constant is
 * chosen based on whether x is greater than or less than a.
 * 
 */
double rdMuscleZajac::
computeDADT(double aTRise,double aTFall,double aX,double aA)
{
printf("PRINTING in  DADT exc %.16lfn\n",aX);
printf("PRINTING in  DADT act %.16lfn\n",aA);
	// DETERMINE TIME CONSTANT
	double tau;
	if(aX>=aA) {
		tau = aTRise;
	} else {
		tau = aTFall;
	}

	// CHECK FOR ZERO TAU
	if(tau<=0) {
		printf("rdMuscle.dadt: ERROR- tau<=0.0\n");
		return(aX-aA);
	}
	printf("PRINTING in  DADT tau %.16lfn\n",tau);

	// COMPUTE DERIVATIVE
	// double dadt = (aX-aA)/tau;
 	double dadt=(aTRise*aX*aX) + (aTFall * aX) -(aTRise*aX+aTFall)*aA;
        printf("PRINTING in  DADT dadt %.16lfn\n",dadt);

	return(dadt);
}
//____
//_____________________________________________________________________________
/**
 * Compute the time derivative of actuator force.
 *
 * @return Time derivative of normalized muscle force.
 */
double rdMuscleZajac::
computeDFDT()
{
	// NORMALIZED ACTUATOR FORCE
	double ftNorm = _force / (_fmOpt);
	

	// TENDON LENGTH
	// _reckt must be normalized to fmOpt
	double lt = _ktRecip * _force + (_ltSlack);
	
	// MUSCLE LENGTH
	double tmp = _lmt - lt;  if(tmp<0.0) tmp=0.0;
 printf("PRINTING in computeDFDT _lmt%.16lfn\n",_lmt);
 printf("PRINTING in computeDFDT tmp%.16lfn\n",tmp);


	double lm = sqrt(_width*_width + tmp*tmp);



	 printf("PRINTING in computeDFDT lm%.16lfn\n",lm);
	double lmMin = _width + 1.0e-4;
	if(lm<lmMin) lm = lmMin;
	double lmNorm = lm / (_lmOpt);


	// PASSIVE FORCE
	double sigma = SIG0 * exp( (lmNorm-1.0) / PE1 );
	

	// COMPUTE DFDT
	double dfdt;
	double denom,term1,term3;
	double fmNorm,fmNormIso,coAlpha,coAlphaRecip,cossq;
	double vce,zeta;
        printf("PRINTING in computeDFDT lmNorm%.16lfn\n",lmNorm);
	// ONLY PASSIVE FORCES
	// lmNorm is beyond _lmNormMax.
	if(lmNorm > LMNORMMAX) {
          tmp = PE1 / (sigma * _ktLmOptRecip);
	  denom = _ktRecip * (1.0 + tmp);
	  dfdt = _speed / denom;
    
         printf("PRINTING in computeDFDT tmp%.16lfn\n",lmNorm);
         printf("PRINTING in computeDFDT alternate  dfdt%.16lfn\n",dfdt);


	// ACTIVE AND PASSIVE FORCES
	} else {

		// FORCE-LENGTH CURVE
		//f:pol
		fmNormIso = ComputeForceLengthCurve(lmNorm);
	
		// COSINE OF ALPHA
        	// F:co/muswidthsq()/lm/recco 
		coAlpha = sqrt(1.0 - _widthSquared / (lm*lm));
		coAlphaRecip = 1.0 / coAlpha;

		// NORMALIZED MUSCLE FORCE
		
	    	fmNorm = (_force*coAlphaRecip - sigma) / (fmNormIso*_a);
		

		if(fmNorm<0.0) fmNorm = 0.0;

		// CONTRACTILE ELEMENT SHORTENING VELOCITY
		vce = (_vmMax) * ComputeShorteningVelocity(fmNormIso,fmNorm);


		// DFDT
		cossq = coAlpha * coAlpha;
		zeta = _speed * _lmRecipOpt;
		
		printf("PRINTING in computeDFDT zeta %.16lfn\n",zeta);

		term1 = _force* (C1 + (1.0 - cossq)/cossq) +
			coAlpha * (C1*C2 + (1.0/PE1 - C1)*sigma);
		term3 = C1 * (_force *coAlphaRecip - sigma + C2);
		printf("PRINTING in computeDFDT term1%.16lfn\n",term1);
		printf("PRINTING in computeDFDT term3%.16lfn\n",term3);
		dfdt = (zeta*term1 - vce*term3) / (coAlphaRecip + _ktLmOptRecip*term1);
		printf("PRINTING in computeDFDT _ktLmOptRecip%.16lfn\n",_ktLmOptRecip);

	}
	printf("PRINTING in computeDFDT%.16lfn\n",dfdt);
	return(dfdt);
}



//*************************************************** 
//* Ilse added the following section
//*************************************************** 

/* ComputeMuscleForce 
* calculates muscle force given the muscle's activation, contractile length and shortening velocity
* ie the basic routine that calls the routines ComputeMuscleLength-ComputeMsForce()
*
* @return active_force // Is this the appropriate name?
*/

double rdMuscleZajac::
ComputeMuscleForce(double activ,double actlen,double actsv) 
{
 
printf("PRINTING IN activ %.16lf\n",activ);
printf("PRINTING IN width %.16lf\n",_width);
printf("PRINTING IN actlen %.16lf\n",actlen);
printf("PRINTING IN actsv %.16lf\n",actsv);

_lm=ComputeMuscleLength(activ,_width,1.0,0.00005,actlen,actsv); 

printf("PRINTING IN _lm %.16lf\n",_lm);
double active_force=ComputeMsForce(activ,_lm,actsv);

printf("PRINTING active_force %.16lf\n",active_force);
return (active_force); //Is this the appropriate name

}




//_____________________________________________________________________________
/*
* Calculate the length of the contractile section based on the Actuator length
* uses function ComputeMsZero()
* Input: 
* activation level _a
* ax, bx:
* tol:desired length of the interval of uncertainty of the final result (>=0.0)
* actuator length: _lmt [m]
* actuator shortening velocity: _speed (nl rdforce::computeSpeed())
* @ return: length of the contractile section
*/

double rdMuscleZajac::
ComputeMuscleLength(double activ, double ax, double bx, double tol, double actlen, double actsv)
{
    
	const double eps=6.93889e-18; //6.9389d-18
	double a,b,fa,fb,c,fc,d,e,toli,xm,q,r,s,p;


	a=ax;	//ax from input
	b=bx;	//bx from input
	printf("PRINTING IN ComputeMuscleLength a %.16lf\n",a);
	printf("PRINTING IN COmputeMuscleLength b %.16lf\n",b);	

	fa=ComputeMuscleZero(activ,a,actlen,actsv);  // Define function mszero 
	fb=ComputeMuscleZero(activ,b,actlen,actsv);	// Define function mszero 
	
	printf("PRINTING IN ComputeMuscleLength fa %.16lf\n",fa);
	printf("PRINTING IN COmputeMuscleLength fb %.16lf\n",fb);	

	printf("PRINTING IN ComputeMuscleLength a %.16lf\n",a);
	
	c=a;
	fc=fa;
	d=b-a;
	e=d;
	printf("PRINTING IN ComputeMuscleLength c %.16lf\n",c);
	printf("PRINTING IN ComputeMuscleLength fc %.16lf\n",fc);
	printf("PRINTING IN ComputeMuscleLength e %.16lf\n",e);

	for(;;)
	{

  	if (abs(fc)>= abs(fb))
	{
	toli=20.0*eps*abs(b)+0.5*tol;
	}
	else
	{
	a=b;
	b=c;
	c=a;
	fa=fb;
	fb=fc;
	fc=fa;
	toli=20.0*eps*abs(b)+0.5*tol;
	printf("PRINTING IN 30 ComputeMuscleLength a %.16lf\n",a);
	printf("PRINTING IN 30  ComputeMuscleLength b %.16lf\n",b);
	printf("PRINTING IN 30  ComputeMuscleLength c %.16lf\n",c);
	printf("PRINTING IN 30  ComputeMuscleLength fa %.16lf\n",fa);
	printf("PRINTING IN 30  ComputeMuscleLength fb %.16lf\n",fb);
	printf("PRINTING IN 30  ComputeMuscleLength fc%.16lf\n",fc);
	printf("PRINTING IN 30  ComputeMuscleLength tol%.16lf\n",tol);
	printf("PRINTING IN 30  ComputeMuscleLength toli %.16lf\n",toli);
	}
	printf("PRINTING IN 30  ComputeMuscleLength d %.16lf\n",d);
	printf("PRINTING IN 30  ComputeMuscleLength c %.16lf\n",c);
	xm=0.5*(c-b);
	printf("PRINTING IN 40 ComputeMuscleLength xm %.16lf\n",xm);
	

        if ((abs(xm)<=toli)||(fb==0.0))
	{
	printf(" IN if1\n");
	break;
	}

	if ((abs(e)<toli)||(abs(fa)<=abs(fb)))
 	{
	printf(" IN if3\n");
	d=xm;
	e=d;
	}

	if (a != c) 
	{
	printf(" IN if5\n");
	q=fa/fc;
	r=fb/fc;
	s=fb/fa;
	p=s*(2.0*xm*q*(q-r)-(b-a)*(r-1.0));
	q=(q-1.0)*(r-1.0)*(s-1.0); 
	}
	else
	{
	printf(" IN else\n");
	s=fb/fa;
	p=2.0*xm*s;
	q=1.0-s;
	;
        }

	if (p > 0.0) 
	{
	printf(" IN if6\n");
	q=-q;
	}
	p=abs(p);

	printf("PRINTING IN else ComputeMuscleLength s%.16lf\n",s);
	printf("PRINTING IN else ComputeMuscleLength p%.16lf\n",p);
	printf("PRINTING IN else ComputeMuscleLength q %.16lf\n",q);
 


	
	if (((2.0*p) >=(3.0*xm*q-abs(toli*q)))||(p >=abs(0.5*e*q)))
	{
	printf(" IN if7\n");
	d=xm;
	e=d;
	}
	else
	{
	e=d;
	d=p/q;
	}
//	printf("PRINTING IN after ComputeMuscleLength e%.16lf\n",e);
//	printf("PRINTING IN after ComputeMuscleLength d %.16lf\n",d);

	a=b;
	fa=fb;
	if (abs(d)>toli) 
	{
	printf(" IN if9\n");
//	printf("PRINTING before ADOPTED b %.16lf\n",b);
//	printf("PRINTING before ADOPTED d %.16lf\n",d);
	b=b+d;
//	printf("PRINTING ADOPTED b %.16lf\n",b);

	}
	if (abs(d)<=toli) 
	{
	printf(" IN if10\n");
		if (xm > 0.0) 
		{
		b=b+toli;
		}
		if (xm < 0.0 )
		{
		b=b-toli;
		}
	}

	fb=ComputeMuscleZero(activ,b,actlen,actsv);
	printf("PRINTING IN ComputeMuscleLength a %.16lf\n",a);
	printf("PRINTING IN ComputeMuscleLength b %.16lf\n",b);
	printf("PRINTING IN ComputeMuscleLength c %.16lf\n",c);
	printf("PRINTING IN ComputeMuscleLength fa %.16lf\n",fa);
	printf("PRINTING IN ComputeMuscleLength fb %.16lf\n",fb);
	printf("PRINTING IN ComputeMuscleLength fc %.16lf\n",fc);


	if (fb*fc/abs(fc) > 0.0)
	{
	printf(" IN if11\n");
	c=a;
	fc=fa;
	d=b-a;
	e=d;	
	
	}

	}	
double _lm=b;
return(_lm);
	
}


double rdMuscleZajac::
ComputeMuscleZero(double activ,double &rLength,double actlen,double actsv)
{
//	printf("PRINTING IN ComputeMuscleZero \n");
	double fmus;
	double dump=(_width+0.0001);
	double tLength;
	tLength=rLength;
	tLength=(tLength>dump)?tLength:dump;
	fmus = ComputeMsForce(activ,tLength,actsv);
	double tomp=_ktRecip*fmus;
	double lt=_ltSlack+tomp;
	double tmp=(double)(actlen-lt);
	double temp=(0.0 > tmp)?0.0:tmp;
	double mszero=temp*temp-tLength*tLength+_widthSquared;
	rLength=tLength;
	printf("PRINTING IN ComputeMuscleZero fmus %.16lf\n",mszero);
	return(mszero);

}

double rdMuscleZajac::
ComputeMsForce(double activ, double aLength, double actsv)
{
//	printf("PRINTING IN COMPUTEMsFORCE \n");
	double lmNorm = aLength / (_lmOpt);
	
	// PASSIVE FORCE
	double sigma = SIG0 * exp( (lmNorm-1.0) / PE1 );
	

	if(lmNorm > LMNORMMAX) {
	 double msforce=sigma;
	 return(msforce);
	}	
	
	// COSINE OF ALPHA


	double coAlpha = sqrt(1.0 - _widthSquared / (aLength*aLength));
	double coAlphaRecip = 1.0 / coAlpha;


	// FORCE-LENGTH CURVE
	//f:pol

	 double fmNormIso = ComputeForceLengthCurve(lmNorm);
	
	// FIND THE CHANGE IN FORCE DUE TO TEH SHORTENING VELOCITY
	// for simplicity we assume that the shortening velocity of the muscle is   			//the same as that of the actuator.
	// In other words assume that the shortening velocity of the tendon is zero

	double reclom=1/_lmOpt;
	double vbar=actsv*reclom*coAlpha/(_vmMax);
	
	vbar=(-1.0 > vbar)?-1:vbar;
	vbar=(2.0 > vbar)?vbar:2.0;
	
	double fce=ComputeVelocityEffect(vbar);
	double msforce=coAlpha*sigma + fmNormIso*coAlpha*activ*fce;
	return(msforce);
}

// I AM NOT SURE ABOUT THE ARGUMENTS LIST - NOT VERIFIED!!!!
double rdMuscleZajac:: 
ComputeActivation(double aForce, double aLength, double asvel)
{
	double lm;
	double Ractiv,pbar,vbar,coAlpha,fmNormIso,sigma;
	// lt is tendon length at aForce
	double lt= _lmOptRecip*aForce+_ltSlack;
	// lm is muscle length at aForce, if lm is too short, adjust it
	double var=aLength-lt;
	double temp=(0.0>var)?0.0:
	
         lm=sqrt(_widthSquared + temp*temp);
	// muscle length must be greater than the width
	double temp2=_width+0.0001;
	lm=(lm>temp2)?lm:temp2;
	
	double LmNorm=lm * _lmOptRecip;

	if (LmNorm > LMNORMMAX){
		Ractiv=0.5;
		goto END;
		}
	
	sigma = SIG0 * exp( (LmNorm-1.0) / PE1 );	
		
	fmNormIso = ComputeForceLengthCurve(LmNorm);	
	// COSINE OF ALPHA
    // F:co/muswidthsq()/lm/recco 
	coAlpha = sqrt(1.0 - _widthSquared / (lm*lm));
	
	vbar=asvel * _lmOptRecip*coAlpha / (_vmMax);
	vbar=(-1>vbar)?-1:vbar;
	vbar=(2>vbar)?vbar:2;
	pbar = ComputeVelocityEffect(vbar);
	pbar=(0.0>pbar)?pbar:0.0;
	fmNormIso=fmNormIso*pbar;
	
	Ractiv=(aForce/coAlpha - sigma)/fmNormIso;

END:

	return(Ractiv);
}


//_____________________________________________________________________________
/**
 * Compute the normalized isometric muscle force based on the force-length
 * curve.
 *
 * @param aLmNorm Normalized muscle force.
 * @return Muscle (or sarcomere) force. 
 */
double rdMuscleZajac::
ComputeForceLengthCurve(double aLmNorm)
{
//	printf("PRINTING IN ComputeForceLengthCurve \n");
	// SPLINE INTERVAL
        int i=1+ (int)(aLmNorm*15.0);
        if (i>30) i=30;
        
 	printf("PRINTING COMPUTEFORCELENGTHCURVE i %d\n",i);
	
	// LEFT OVER MUSCLE LENGTH
	double tmp = (double)(i - 1);
 	printf("PRINTING COMPUTEFORCELENGTHCURVE tmp %.16lf\n",tmp);
	double dlm = aLmNorm - (tmp/15.0);
 	printf("PRINTING COMPUTEFORCELENGTHCURVE dlm %.16lf\n",dlm);
	// EVALUATE THE SPLINE
	double fm;
	i=i-1; //different numbering of array in c
	fm = (((( flcoef[i][5] *dlm + flcoef[i][4] )*dlm +
				 flcoef[i][3] )*dlm + flcoef[i][2] )*dlm +
				 flcoef[i][1] )*dlm + flcoef[i][0];
 	
	return(fm);
}

//_____________________________________________________________________________
/**
 * Compute the normalized shortening velocity of the contractile element.
 * To get the actual shortening velocity the returned quantity must be
 * multiplied by the maximum shortening velocity.
 *
 * @param aFmNormIsometric Normalized force that would be generated by the
 * muscle under isometric conditions.  This value can be computed by calling
 * computeForceLengthCurve().
 * @param aFmNorm Normalized force currently being generated by the muscle.
 * @return Contractile shortening velocity in optimal fibers per second.
 * Positive values indicate shortening.
 */
double rdMuscleZajac::
ComputeShorteningVelocity(double aFmNormIsometric,double aFmNorm)
{
//	printf("PRINTING IN ComputeShorteningVelocity \n");
	// SPLINE INTERVAL
	int i = 1 + (int)(16.0*aFmNorm);
	if(i>28) i = 28;

	// LEFT OVER MUSCLE FORCE
	double tmp = (double)(i - 1);
	double dfm = aFmNorm - tmp/16.0;

	// EVALUATE THE SPLINE
	i=i-1; //different array structure in c
	double vce = (((( fvcoef[i][5]  *dfm + fvcoef[i][4] )*dfm
						 + fvcoef[i][3] )*dfm + fvcoef[i][2] )*dfm
						 + fvcoef[i][1] )*dfm + fvcoef[i][0] + 		2.1220e-03;

	return(vce);
}



//*************************************************** 
//* Ilse added the following section
//*************************************************** 
//_____________________________________________________________________________
/**
 * Compute the change in force due to the shortening velocity of thee muscle
* for simplicity assume that the shortening velocity of the muscle is the same 
*as that of the actuator, ie the shortening velocity of the tendon is zero. THis is called by 
* COmputemusclevelocity
*/
double rdMuscleZajac::
ComputeVelocityEffect(double aVNorm)
{
//	printf("PRINTING IN ComputeFVelocityEffect \n");
	// SPLINE INTERVAL
	int i = 11 + (int)(10.0*aVNorm);
	if(i>30) i = 30;


	// LEFT OVER MUSCLE FORCE
	double tmp = (double)(i - 11);
	double dpb = aVNorm - tmp*0.1;

	// EVALUATE THE SPLINE
	i=i-1;
	double fce = (((( vfcoef[i][5]  *dpb + vfcoef[i][4] )*dpb
						 + vfcoef[i][3] )*dpb + vfcoef[i][2] )*dpb
						 + vfcoef[i][1] )*dpb + vfcoef[i][0] + 			2.1220e-03;


	fce=(0.0>fce)?0.0:fce;
	return(fce);

}




//=============================================================================
// CHECK
//=============================================================================
//_____________________________________________________________________________
