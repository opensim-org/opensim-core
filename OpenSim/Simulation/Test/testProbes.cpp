/* -------------------------------------------------------------------------- *
 *                          OpenSim:  testProbes.cpp                          *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2012 Stanford University and the Authors                *
 * Author(s): Tim Dorn                                                        *
 *                                                                            *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may    *
 * not use this file except in compliance with the License. You may obtain a  *
 * copy of the License at http://www.apache.org/licenses/LICENSE-2.0.         *
 *                                                                            *
 * Unless required by applicable law or agreed to in writing, software        *
 * distributed under the License is distributed on an "AS IS" BASIS,          *
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied    *
 * See the License for the specific language governing permissions and        *
 * limitations under the License.                                             *
 * -------------------------------------------------------------------------- */

//==========================================================================================================
//	testProbes builds an OpenSim model containing a Millard2012Equilibrium model using the OpenSim API 
//  applys a bunch of probes to it
//		
//     Add more test cases to address specific problems with probes
//
//==========================================================================================================

#include <OpenSim/OpenSim.h>
#include <OpenSim/Simulation/Control/Controller.h>
#include <OpenSim/Actuators/FiberCompressiveForceCosPennationCurve.h>
#include <OpenSim/Auxiliary/auxiliaryTestFunctions.h>

#include <OpenSim/Actuators/Thelen2003Muscle.h>
#include <OpenSim/Actuators/Millard2012EquilibriumMuscle.h>
#include <OpenSim/Actuators/Millard2012AccelerationMuscle.h>

#include <OpenSim/Simulation/Model/ActuatorPowerProbe.h>
#include <OpenSim/Simulation/Model/ActuatorForceProbe.h>
#include <OpenSim/Simulation/Model/JointInternalPowerProbe.h>
#include <OpenSim/Simulation/Model/SystemEnergyProbe.h>

#include <OpenSim/Simulation/Model/MetabolicMuscle.h>
#include <OpenSim/Simulation/Model/MetabolicMuscleSet.h>
#include <OpenSim/Simulation/Model/MuscleMetabolicPowerProbeBhargava2004.h>
#include <OpenSim/Simulation/Model/MuscleMetabolicPowerProbeUmberger2003.h>

#include <OpenSim/Analyses/ProbeReporter.h>
#include <OpenSim/Analyses/ForceReporter.h>


using namespace std;
using namespace OpenSim;
using namespace SimTK;


/**
* This controller allows the user to apply one of a set of prescribed activation
* to the model
*
* @params aModel: a reference to the current model, as required by the parent class
* @params u: a parameter that modifies the activation type
* @params actType: the type of activation: 0 for constant, 1 for a sin wave
*/
class MuscleController : public Controller{
OpenSim_DECLARE_CONCRETE_OBJECT(MuscleController, Controller);
public:
	MuscleController(Model& aModel, double aU, int aActType):
	  Controller(), u(aU), actType(aActType){
		
		switch(actType){
			case 0:
				cout << endl<<"Constant Excitation: " << u << endl;
				break;
			case 1:
				cout << endl<<"Sin Excitation: 0.5+0.5*sin(2*Pi)" << endl; 
				break;
			default:
				cout << endl<<"Default Excitation: 0.1" << endl; 
		}

	  }

	virtual void computeControls(const SimTK::State& s, SimTK::Vector &controls)
	const{
		//Muscle* muscle1 = dynamic_cast<Muscle*>(&_actuatorSet.get(0));
		double time = s.getTime();
		switch(actType){
			case 0:
				controls[0] = u;
				break;
			case 1:
				controls[0] = 0.5 + 0.5*sin(2*Pi*time);	 
				break;
			default:
				controls[0] = 0.1;
		}
		 
	}

	double getInitialExcitation(double t0)
	const{
		double u0 = 0.0;
		switch(actType){
			case 0:
				u0 = u;
				break;
			case 1:
				u0 = 0.5 + 0.5*sin(2*Pi*t0);	 
				break;
			default:
				u0 = 0.1;
		}
		return u0;
	}

	void setInitialExcitation(double aU)
	{
		u = aU;
	}

private:
	double u;
	int actType;

};




//______________________________________________________________________________
/**
 * Create a muscle bench marking system. The bench mark consists of a single muscle 
 * spans a distance. The distance the muscle spans can be controlled, as can the 
 * excitation of the muscle.
 */
int main()
{
	try {
		//Bench Mark Configuration parameters
		//							 add muscles,  prescribed motion         										
		bool config_mdlComponents[2]	= {	true,  true };  
		int	config_blockmotion		= 2;    //0 - Muscle held at 1 resting length apart
                                            //1 - Ramp function 
										    //2 - Muscle stretched according to a full +/-200% strain sinewave
		int config_activationType	= 0;    //0 - Muscle activation set to a constant value
										    //1 - Muscle activated according to a sine wave
		double config_activationU = 0.5;    // atof(argv[1]); 
		double mcl_stretch = 0.3;           // atof(argv[2]);
		double mcl_bias    = 0.1;           // atof(argv[3]);
		double mcl_cycles = 1;              // atof(argv[4]);

		//Block Parameters
		double	blk_mass		= 20;       //kg
		double	blk_sidelen		= 0.1;      //0.1m^3
		double  gravityY		= -9.81;    //N/kg

		//Muscle Parameters
		double mcl_maxIsoF	= 100.0;	//N
		double mcl_optFibL	= 0.1;		//m
		double mcl_tenSlkL	= 0.2;		//m
		double mcl_penAng	= SimTK::Pi/4;      // atof(argv[9]);		//radians? degrees
		double mcl_tau_a	= 0.01;
		double mcl_tau_da	= 0.04;
        double mcl_tau_min  = 0.05;
		double mcl_muscleLength = mcl_optFibL*cos(mcl_penAng) + mcl_tenSlkL;
		double mcl_tmaxstrain = 0.04;
		//Simulation parameters
		double t0 = 0;
		double t1 = 4.0;                // atof(argv[5]);
		double int_maxstep	= 1;
		double int_minstep	= 1e-15;
		double int_accuracy	= 1e-6;     // atof(argv[6]);
		double int_tolerance= 1e-6;     // atof(argv[6]);

        int config_musclemodel = 1;     // atoi(argv[7]); 
        int config_printCurves = 0;     // atoi(argv[8]);

        std::string path = ".";

        //0: Thelen model
        //1: Millard2012Equilibrium Model
        //2: Millard2012Acceleration Model

        string config_filename = "";

        switch(config_musclemodel){
        case 0:
            config_filename = "Thelen2003Muscle";
            break;
        case 1: 
            config_filename = "Millard2012EquilibriumMuscle";
            break;
        case 2:
            config_filename = "Millard2012AccelerationMuscle";
            break;
        default:
            config_filename = "InvalidMuscleName";
        }
		
		// Create an OpenSim model and set its name
		cout<<"\nCreating OpenSim model\n" << endl;
		Model osimModel;
		osimModel.setName("Isolated muscle model for testing Probes");

		//******************************************
		// Ground
		//******************************************
		// Get a reference to the model's ground body
		cout<<"..adding ground and geometry" << endl;
		OpenSim::Body& ground = osimModel.getGroundBody();

		//Add display geometry to show the ground in the GUI
		ground.addDisplayGeometry("ground.vtp");
		ground.addDisplayGeometry("anchor1.vtp");
		//ground.addDisplayGeometry("anchor2.vtp");


		//******************************************
		// Block
		//******************************************

		Vec3	blockMassCenter(0);
		Inertia	blockInertia	= blk_mass
			*Inertia::brick(blk_sidelen,blk_sidelen,blk_sidelen);
		
		//Question 1: why use a reference for ground, but a pointer for the block?

		//Create the new block
		OpenSim::Body *block = new OpenSim::Body("block",blk_mass,blockMassCenter,
			blockInertia);

		//Add display geometry to the block to visualize it
		block->addDisplayGeometry("block.vtp");
		
		//Adding block parameters and geometry
		cout<<"..adding block parameters " << blk_mass << "kg " <<
			blk_sidelen << "m  and geometry" << endl;

		//******************************************
		// Block-Ground Joint
		//******************************************


		//Now join the block to the ground using a 6 dof joint
		Vec3 locationInParent(0, 0, 0),
			orientationInParent(0), locationInBody(0),
			orientationInBody(0);
		FreeJoint *blockToGround = new 
			FreeJoint("blockToGround",ground,locationInParent,
			orientationInParent, *block, locationInBody,
			orientationInBody);

        Array<string> jointNames;
        jointNames.append("blockToGround");

		//Get a reference to the coordinate set (6 dof) between block and ground
		CoordinateSet& jointCoordinateSet = blockToGround->upd_CoordinateSet();

		double angleRange[2]	= {-Pi/2,Pi/2};
		double positionRange[2]	= {-10, 10};
		jointCoordinateSet[0].setRange(angleRange);
		jointCoordinateSet[1].setRange(angleRange);
		jointCoordinateSet[2].setRange(angleRange);
		jointCoordinateSet[3].setRange(positionRange);
		jointCoordinateSet[4].setRange(positionRange);
		jointCoordinateSet[5].setRange(positionRange);

		//Add the block body to the model
		osimModel.addBody(block);

		cout<<"..adding a free joint between the block and the ground " << endl; 
		
		//******************************************
		// Gravity
		//******************************************
		osimModel.setGravity(Vec3(0,gravityY,0));



		//******************************************
		// Prescribed Motion
		//******************************************
		if(config_mdlComponents[1] == true)
		{
			int pmSize = 1000;			
			OpenSim::Array<double> zeroCoeff;
            OpenSim::Array<double> rampCoeff;
			OpenSim::Array<double> constCoeff;
			OpenSim::Array<double> pmTime;
			OpenSim::Array<double> pmPos;
			double delta = (t1-t0)/(double)(pmSize-1); 

			constCoeff.setSize(pmSize);
			zeroCoeff.setSize(2);
            rampCoeff.setSize(2);
			pmTime.setSize(pmSize);
			pmPos.setSize(pmSize);			
					
			const CoordinateSet &blkCrdSet = osimModel.getCoordinateSet();

			for(int i=0; i< pmSize; i++){
				pmTime.set(i,i*delta);
				pmPos.set(i, mcl_muscleLength
                    + mcl_bias*mcl_optFibL
                    + mcl_stretch*mcl_optFibL
                      *sin(i*delta*2*Pi*mcl_cycles/(t1-t0)));
				constCoeff.set(i,mcl_muscleLength + mcl_bias*mcl_optFibL);

                if(i==0){
                     double tmp = mcl_muscleLength
                        + mcl_bias*mcl_optFibL
                        + mcl_stretch*mcl_optFibL
                          *sin(i*delta*2*Pi*mcl_cycles/(t1-t0));
                     printf("\nInitial Gap Length: %f\n",tmp);
                }
			}

			zeroCoeff.set(0,0);
			zeroCoeff.set(1,0);

            rampCoeff.set(0,mcl_muscleLength + mcl_bias*mcl_optFibL);
            rampCoeff.set(1, mcl_muscleLength 
                           + mcl_bias*mcl_optFibL + mcl_stretch*mcl_optFibL);
			//constCoeff.set(0,mcl_muscleLength);
			//constCoeff.set(1,mcl_muscleLength);

			//LinearFunction *constVal  = new LinearFunction(constCoeff);

			LinearFunction *zero = new LinearFunction(zeroCoeff);
			LinearFunction *ramp = new LinearFunction(rampCoeff);
            SimmSpline *ncs_sinWave = new SimmSpline(   pmTime.getSize(),
                                                        &pmTime[0],
                                                        &pmPos[0],
                                                        "sine wave");

			SimmSpline *constVal = new SimmSpline(  pmTime.getSize(),
                                                    &pmTime[0],
                                                    &constCoeff[0],
                                                    "1 muscle length");

            SimTK::Vector time0(1);
            time0[0] = 0;

			for(int i=0; i< blkCrdSet.getSize(); i++){
				Coordinate &blkCrd = blkCrdSet.get(i);

				switch(i){
					case 0:		//rot x
						blkCrd.setPrescribedFunction(*zero);	
						blkCrd.setDefaultValue(0.0);
						cout<<"..adding a constant value of 0 to the " << 
                            blkCrd.getName() << endl;
						break;
					case 1:		//rot y
						blkCrd.setPrescribedFunction(*zero);					
						blkCrd.setDefaultValue(0.0);
						cout<<"..adding a constant value of 0 to the " << 
                            blkCrd.getName() << endl;
						break;
					case 2:		//rot z
						blkCrd.setPrescribedFunction(*zero);
						blkCrd.setDefaultValue(0.0);
						cout<<"..adding a constant value of 0 to the " << 
                            blkCrd.getName() << endl;
						break;
					case 3:		//trans x
						blkCrd.setPrescribedFunction(*zero);
						blkCrd.setDefaultValue(0.0);
						cout<<"..adding a constant value of 0 to the " << 
                            blkCrd.getName() << endl;
						break;
					case 4:		//trans y
						blkCrd.setPrescribedFunction(*zero);
						blkCrd.setDefaultValue(0.0);
						cout<<"..adding a constant value of 0 to the " << 
                            blkCrd.getName() << endl;
						break;
					case 5:		//trans z
						switch(config_blockmotion){
							case 0:
								blkCrd.setPrescribedFunction(*constVal);
								blkCrd.setDefaultValue(constVal->calcValue(time0));
								cout<<"..adding a constant separation to the " 
                                    << blkCrd.getName() << endl;
								break;
							case 1:
								blkCrd.setPrescribedFunction(*ramp);                                
                                blkCrd.setDefaultValue(ramp->calcValue(time0));
								cout<<"..adding a ramp function to the " 
                                    << blkCrd.getName() << endl;
								break;
                            case 2:
								blkCrd.setPrescribedFunction(*ncs_sinWave);                                
								blkCrd.setDefaultValue(ncs_sinWave->calcValue(time0));
								cout<<"..adding a sinewave function to the " 
                                    << blkCrd.getName() << endl;                                
                                printf("... span at t=0: %f\n", 
                                        ncs_sinWave->calcValue(time0));
								break;
							default:
								blkCrd.setPrescribedFunction(*zero);
								blkCrd.setDefaultValue(0.0);
								cout<<"..adding a constant value of 0 to the " 
                                    << blkCrd.getName() << endl;
						}
						
						break;
					default:
						cout<<"..Error: prescribedFunction Code, blockCoord index exceeded. " << endl;
				}
			
			}
		}
		
		//******************************************
		// Muscles
		//******************************************
        std::string mclAccName = "muscleAcceleration";
		Millard2012AccelerationMuscle* muscleAcc = new
               Millard2012AccelerationMuscle(mclAccName,
                                            mcl_maxIsoF,
                                            mcl_optFibL,
                                            mcl_tenSlkL,
                                            mcl_penAng);

        muscleAcc->set_fiber_force_length_damping(1e-3);//1e-1
        muscleAcc->set_fiber_compressive_force_length_damping(1e-3);
        muscleAcc->set_fiber_compressive_force_cos_pennation_damping(1e-3);
        muscleAcc->set_tendon_force_length_damping(1e-1);//5e-
        //muscleAcc->set_fiber_damping(1e-2);
        muscleAcc->set_mass(0.01);

        std::string mclEqName = "muscleEquilibrium";
        Millard2012EquilibriumMuscle* muscleEq = new
               Millard2012EquilibriumMuscle(mclEqName,
                                            mcl_maxIsoF,
                                            mcl_optFibL,
                                            mcl_tenSlkL,
                                            mcl_penAng);

        std::string mclThelenName = "muscleThelen";
        Thelen2003Muscle* muscleThEq = new
                            Thelen2003Muscle(mclThelenName,
                                            mcl_maxIsoF,
                                            mcl_optFibL,
                                            mcl_tenSlkL,
                                            mcl_penAng);

		MuscleController* muscleControl = new MuscleController(osimModel, 
                                                       config_activationU, 
                                                       config_activationType);

        
       
        Array<string> muscNames;
        
		if(config_mdlComponents[0] == true){
			

			//Specify the paths for the muscle
            switch(config_musclemodel){
                case 0: 
                {
                    muscleThEq->addNewPathPoint("muscle-point1",ground,Vec3(0,0,0));
			        muscleThEq->addNewPathPoint("muscle-point2",*block,Vec3(0,0,0));
			        muscleControl->setActuators(osimModel.updActuators());			
			        osimModel.addForce(muscleThEq);
                    muscNames.append(muscleThEq->getName());
                    cout<<"..adding one Thelen2003Muscle " << endl; 	
                }break;
                case 1: 
                {
                    muscleEq->addNewPathPoint("muscle-point1",ground,Vec3(0,0,0));
			        muscleEq->addNewPathPoint("muscle-point2",*block,Vec3(0,0,0));
			        muscleControl->setActuators(osimModel.updActuators());			
			        osimModel.addForce(muscleEq);
                    muscNames.append(muscleEq->getName());
                    cout<<"..adding one Millard2012EqulibriumMuscle " << endl;


                    if(config_printCurves == 1){
                        muscleEq->getActiveForceLengthCurve()
                            .printMuscleCurveToCSVFile(path);
                        muscleEq->getForceVelocityInverseCurve()
                            .printMuscleCurveToCSVFile(path);
                        //muscleEq->getFiberForceLengthCurve()
                        //    .printMuscleCurveToCSVFile(path);
                        //muscleEq->getFiberCompressiveForceLengthCurve()
                        //    .printMuscleCurveToCSVFile(path);
                        //muscleEq->getFiberCompressiveForceCosPennationCurve()
                        //    .printMuscleCurveToCSVFile(path);
                    }
                }break;
                case 2: 
                {
                    muscleAcc->addNewPathPoint("muscle-point1",ground,Vec3(0,0,0));
			        muscleAcc->addNewPathPoint("muscle-point2",*block,Vec3(0,0,0));
			        muscleControl->setActuators(osimModel.updActuators());			
			        osimModel.addForce(muscleAcc);
                    muscNames.append(muscleAcc->getName());

                    if(config_printCurves == 1){
                        muscleAcc->getActiveForceLengthCurve()
                            .printMuscleCurveToCSVFile(path);
                        muscleAcc->getForceVelocityCurve()
                            .printMuscleCurveToCSVFile(path);
                        //muscleAcc->getFiberForceLengthCurve()
                        //    .printMuscleCurveToCSVFile(path);
                        //muscleAcc->getFiberCompressiveForceLengthCurve()
                        //    .printMuscleCurveToCSVFile(path);
                        //muscleAcc->getFiberCompressiveForceCosPennationCurve()
                        //    .printMuscleCurveToCSVFile(path);
                    }

                    cout<<"..adding one Millard2012AccelerationMuscle " << endl; 	
                }break;
                default:
                    cout<<"..Invalid Muscle Model Selection " << endl;
            }

			
			//osimModel.addAnalysis(muscleReporter);
			osimModel.addController(muscleControl);

			
			cout<<"..and its analysis " << endl; 	
			cout<<"..and a controller " << endl; 	
		}
		

        // --------------------------------------------------------------------
        // ADD PROBES TO THE MODEL TO GET INTERESTING VALUES
        // --------------------------------------------------------------------
        Array<string> muscNamesTwice = muscNames;
        muscNamesTwice.append(muscNames.get(0));
        cout << "------------\nPROBES\n------------" << endl;
        int probeCounter = 1;

        // Add ActuatorPowerProbe to measure work done by the muscle 
        ActuatorPowerProbe* muscWorkProbe = new ActuatorPowerProbe(muscNames, false, 1);
        //muscWorkProbe->setName("ActuatorWork");
        muscWorkProbe->setOperation("integrate");
        osimModel.addProbe(muscWorkProbe);
        cout << probeCounter++ << ") Added ActuatorPowerProbe to measure work done by the muscle" << endl; 
        if (muscWorkProbe->getName() != "UnnamedProbe") {
            string errorMessage = "Incorrect default name for unnamed probe: " + muscWorkProbe->getName(); 
            throw (OpenSim::Exception(errorMessage.c_str()));
        }

        // Add ActuatorPowerProbe to measure power generated by the muscle 
        ActuatorPowerProbe* muscPowerProbe = new ActuatorPowerProbe(*muscWorkProbe);	// use copy constructor
        muscPowerProbe->setName("ActuatorPower");
        muscPowerProbe->setOperation("value");
        osimModel.addProbe(muscPowerProbe);
        cout << probeCounter++ << ") Added ActuatorPowerProbe to measure power generated by the muscle" << endl; 

        // Add ActuatorPowerProbe to measure the square of the power generated by the muscle 
        ActuatorPowerProbe* muscPowerSquaredProbe = new ActuatorPowerProbe(*muscPowerProbe);	// use copy constructor
        muscPowerSquaredProbe->setName("ActuatorPowerSquared");
        muscPowerSquaredProbe->setExponent(2.0);
        osimModel.addProbe(muscPowerSquaredProbe);
        cout << probeCounter++ << ") Added ActuatorPowerProbe to measure the square of the power generated by the muscle" << endl; 

        // Add JointInternalPowerProbe to measure work done by the joint 
        JointInternalPowerProbe* jointWorkProbe = new JointInternalPowerProbe(jointNames, false, 1);
        jointWorkProbe->setName("JointWork");
        jointWorkProbe->setOperation("integrate");
        osimModel.addProbe(jointWorkProbe);
        cout << probeCounter++ << ") Added JointPowerProbe to measure work done by the joint" << endl;

        // Add JointPowerProbe to measure power generated by the joint 
        JointInternalPowerProbe* jointPowerProbe = new JointInternalPowerProbe(*jointWorkProbe);	// use copy constructor
        jointPowerProbe->setName("JointPower");
        jointPowerProbe->setOperation("value");
        osimModel.addProbe(jointPowerProbe);
        cout << probeCounter++ << ") Added JointPowerProbe to measure power generated by the joint" << endl;

        // Add ActuatorForceProbe to measure the impulse of the muscle force 
        ActuatorForceProbe* impulseProbe = new ActuatorForceProbe(muscNames, false, 1);
        impulseProbe->setName("ActuatorImpulse");
        impulseProbe->setOperation("integrate");
        osimModel.addProbe(impulseProbe);
        cout << probeCounter++ << ") Added ActuatorForceProbe to measure the impulse of the muscle force" << endl;

        // Add ActuatorForceProbe to report the muscle force 
        ActuatorForceProbe* forceProbe = new ActuatorForceProbe(*impulseProbe);			// use copy constructor
        forceProbe->setName("ActuatorForce");
        forceProbe->setOperation("value");
        osimModel.addProbe(forceProbe);
        cout << probeCounter++ << ") Added ActuatorForceProbe to report the muscle force" << endl;

        // Add ActuatorForceProbe to report the square of the muscle force 
        ActuatorForceProbe* forceSquaredProbe = new ActuatorForceProbe(*forceProbe);			// use copy constructor
        forceSquaredProbe->setName("ActuatorForceSquared");
        forceSquaredProbe->setExponent(2.0);
        osimModel.addProbe(forceSquaredProbe);
        cout << probeCounter++ << ") Added ActuatorForceProbe to report the square of the muscle force " << endl;

        // Add ActuatorForceProbe to report the square of the muscle force for the same muscle repeated twice
        ActuatorForceProbe* forceSquaredProbeTwice = new ActuatorForceProbe(*forceSquaredProbe);			// use copy constructor
        forceSquaredProbeTwice->setName("ActuatorForceSquared_RepeatedTwice");
        forceSquaredProbeTwice->setSumForcesTogether(true);
        forceSquaredProbeTwice->setActuatorNames(muscNamesTwice);
        osimModel.addProbe(forceSquaredProbeTwice);
        cout << probeCounter++ << ") Added ActuatorForceProbe to report the square of the muscle force for the same muscle repeated twice" << endl;

        // Add ActuatorForceProbe to report the square of the muscle force for the same muscle repeated twice, SCALED BY 0.5
        ActuatorForceProbe* forceSquaredProbeTwiceScaled = new ActuatorForceProbe(*forceSquaredProbeTwice);			// use copy constructor
        forceSquaredProbeTwice->setName("ActuatorForceSquared_RepeatedTwiceThenHalved");
        double gain1 = 0.5;
        forceSquaredProbeTwiceScaled->setGain(gain1);
        osimModel.addProbe(forceSquaredProbeTwiceScaled);
        cout << probeCounter++ << ") Added ActuatorForceProbe to report the square of the muscle force for the same muscle repeated twice, SCALED BY 0.5" << endl;

        // Add ActuatorForceProbe to report -3.5X the muscle force 
        double gain2 = -3.50;
        ActuatorForceProbe* forceProbeScale = new ActuatorForceProbe(*impulseProbe);		// use copy constructor
        forceProbeScale->setName("ScaleActuatorForce");
        forceProbeScale->setOperation("value");
        forceProbeScale->setGain(gain2);
        osimModel.addProbe(forceProbeScale);
        cout << probeCounter++ << ") Added ActuatorForceProbe to report -3.5X the muscle force" << endl;

        // Add ActuatorForceProbe to report the differentiated muscle force 
        ActuatorForceProbe* forceProbeDiff = new ActuatorForceProbe(*impulseProbe);		// use copy constructor
        forceProbeDiff->setName("DifferentiateActuatorForce");
        forceProbeDiff->setOperation("differentiate");
        osimModel.addProbe(forceProbeDiff);
        cout << probeCounter++ << ") Added ActuatorForceProbe to report the differentiated muscle force" << endl;

        // Add SystemEnergyProbe to measure the system KE+PE
        SystemEnergyProbe* sysEnergyProbe = new SystemEnergyProbe(true, true);
        sysEnergyProbe->setName("SystemEnergy");
        sysEnergyProbe->setOperation("value");
        sysEnergyProbe->setComputeKineticEnergy(true);
        sysEnergyProbe->setComputePotentialEnergy(true);
        osimModel.addProbe(sysEnergyProbe);
        cout << probeCounter++ << ") Added SystemEnergyProbe to measure the system KE+PE" << endl;

        // Add SystemEnergyProbe to measure system power (d/dt system KE+PE)
        SystemEnergyProbe* sysPowerProbe = new SystemEnergyProbe(*sysEnergyProbe);	// use copy constructor
        sysPowerProbe->setName("SystemPower");
        sysPowerProbe->setDisabled(false);
        sysPowerProbe->setOperation("differentiate");
        osimModel.addProbe(sysPowerProbe);
        cout << probeCounter++ << ") Added SystemEnergyProbe to measure system power (d/dt system KE+PE)" << endl;

        // Add ActuatorForceProbe to report the muscle force value, twice -- REPORTED INDIVIDUALLY AS VECTORS
        ActuatorForceProbe* forceSquaredProbeTwiceReportedIndividually1 = new ActuatorForceProbe(*forceProbe);			// use copy constructor
        forceSquaredProbeTwiceReportedIndividually1->setName("MuscleForce_VALUE_VECTOR");
        forceSquaredProbeTwiceReportedIndividually1->setSumForcesTogether(false);    // report individually
        forceSquaredProbeTwiceReportedIndividually1->setActuatorNames(muscNamesTwice);
        //cout << forceSquaredProbeTwiceReportedIndividually1->getActuatorNames().size() << endl;
        forceSquaredProbeTwiceReportedIndividually1->setOperation("value");
        osimModel.addProbe(forceSquaredProbeTwiceReportedIndividually1);
        cout << probeCounter++ << ") Added ActuatorForceProbe to report the muscle force value, twice - REPORTED INDIVIDUALLY" << endl;

        // Add ActuatorForceProbe to report the differentiated muscle force value, twice -- REPORTED INDIVIDUALLY AS VECTORS
        ActuatorForceProbe* forceSquaredProbeTwiceReportedIndividually2 = new ActuatorForceProbe(*forceSquaredProbeTwiceReportedIndividually1);			// use copy constructor
        forceSquaredProbeTwiceReportedIndividually2->setName("MuscleForce_DIFFERENTIATE_VECTOR");
        forceSquaredProbeTwiceReportedIndividually2->setSumForcesTogether(false);    // report individually
        forceSquaredProbeTwiceReportedIndividually2->setOperation("differentiate");
        osimModel.addProbe(forceSquaredProbeTwiceReportedIndividually2);
        cout << probeCounter++ << ") Added ActuatorForceProbe to report the differentiated muscle force value, twice - REPORTED INDIVIDUALLY" << endl;

        // Add ActuatorForceProbe to report the integrated muscle force value, twice -- REPORTED INDIVIDUALLY AS VECTORS
        ActuatorForceProbe* forceSquaredProbeTwiceReportedIndividually3 = new ActuatorForceProbe(*forceSquaredProbeTwiceReportedIndividually1);			// use copy constructor
        forceSquaredProbeTwiceReportedIndividually3->setName("MuscleForce_INTEGRATE_VECTOR");
        forceSquaredProbeTwiceReportedIndividually3->setSumForcesTogether(false);    // report individually
        forceSquaredProbeTwiceReportedIndividually3->setOperation("integrate");
        SimTK::Vector initCondVec(2);
        initCondVec(0) = 0;
        initCondVec(1) = 10;
        forceSquaredProbeTwiceReportedIndividually3->setInitialConditions(initCondVec);
        osimModel.addProbe(forceSquaredProbeTwiceReportedIndividually3);
        cout << probeCounter++ << ") Added ActuatorForceProbe to report the integrated muscle force value, twice - REPORTED INDIVIDUALLY" << endl;
        cout << "initCondVec = " << initCondVec << endl;




        // Add muscle metabolic power probes
        // --------------------------------------------------
        bool addMuscleMetabolicProbes = true;
        if(addMuscleMetabolicProbes) {
            
            MetabolicMuscle m(0.5, 0.5, 40, 133, 74, 111);
            m.setName(muscNames.get(0));
            MetabolicMuscleSet mms;
            mms.cloneAndAppend(m);
            //cout << m << endl;
            //cout << mms << endl;
        
            // MuscleMetabolicPowerProbeBhargava2004 Power Probe: BASAL HEAT RATE
            MuscleMetabolicPowerProbeBhargava2004* BhargavaBasal = new MuscleMetabolicPowerProbeBhargava2004(false, false, false, true, false);
            BhargavaBasal->setName("BhargavaBASAL");
            BhargavaBasal->setMetabolicMuscleSet(mms);
            osimModel.addProbe(BhargavaBasal);
            cout << probeCounter++ << ") Added MuscleMetabolicPowerProbeBhargava2004 to measure BASAL muscle metabolic power" << endl;

            // MuscleMetabolicPowerProbeUmberger2003 Power Probe: BASAL HEAT RATE
            MuscleMetabolicPowerProbeUmberger2003* UmbergerBasal = new MuscleMetabolicPowerProbeUmberger2003(false, false, true, false);
            UmbergerBasal->setName("UmbergerBASAL");
            UmbergerBasal->setMetabolicMuscleSet(mms);
            osimModel.addProbe(UmbergerBasal);
            cout << probeCounter++ << ") Added MuscleMetabolicPowerProbeUmberger2003 to measure BASAL muscle metabolic power" << endl;

            // MuscleMetabolicPowerProbeBhargava2004 Power Probe: ACTIVATION HEAT RATE
            MuscleMetabolicPowerProbeBhargava2004* BhargavaAct = new MuscleMetabolicPowerProbeBhargava2004(true, false, false, false, false);
            BhargavaAct->setName("BhargavaAct");
            BhargavaAct->setMetabolicMuscleSet(mms);
            osimModel.addProbe(BhargavaAct);
            cout << probeCounter++ << ") Added MuscleMetabolicPowerProbeBhargava2004 to measure ACTIVATION muscle metabolic power" << endl;

            // MuscleMetabolicPowerProbeBhargava2004 Power Probe: MAINTENANCE HEAT RATE
            MuscleMetabolicPowerProbeBhargava2004* BhargavaMain = new MuscleMetabolicPowerProbeBhargava2004(false, true, false, false, false);
            BhargavaMain->setName("BhargavaMain");
            BhargavaMain->setMetabolicMuscleSet(mms);
            osimModel.addProbe(BhargavaMain);
            cout << probeCounter++ << ") Added MuscleMetabolicPowerProbeBhargava2004 to measure MAINTENANCE muscle metabolic power" << endl;

            // MuscleMetabolicPowerProbeBhargava2004 Power Probe: ACTIVATION+MAINTENANCE HEAT RATE
            MuscleMetabolicPowerProbeBhargava2004* BhargavaActMain = new MuscleMetabolicPowerProbeBhargava2004(true, true, false, false, false);
            BhargavaActMain->setName("BhargavaActMain");
            BhargavaActMain->setMetabolicMuscleSet(mms);
            osimModel.addProbe(BhargavaActMain);
            cout << probeCounter++ << ") Added MuscleMetabolicPowerProbeBhargava2004 to measure ACTIVATION+MAINTENANCE muscle metabolic power" << endl;

            // MuscleMetabolicPowerProbeUmberger2003 Power Probe: ACTIVATION+MAINTENANCE HEAT RATE
            MuscleMetabolicPowerProbeUmberger2003* UmbergerActMain = new MuscleMetabolicPowerProbeUmberger2003(true, false, false, false);
            UmbergerActMain->setName("UmbergerActMain");
            UmbergerActMain->setMetabolicMuscleSet(mms);
            osimModel.addProbe(UmbergerActMain);
            cout << probeCounter++ << ") Added MuscleMetabolicPowerProbeUmberger2003 to measure ACTIVATION+MAINTENANCE muscle metabolic power" << endl;

            // MuscleMetabolicPowerProbeBhargava2004 Power Probe: SHORTENING HEAT RATE (setUsingForceDepShorteningPropConstant = true)
            MuscleMetabolicPowerProbeBhargava2004* BhargavaShorteningForceDepTrue = new MuscleMetabolicPowerProbeBhargava2004(false, false, true, false, false);
            BhargavaShorteningForceDepTrue->setName("BhargavaShorteningForceDepTrue");
            BhargavaShorteningForceDepTrue->setUsingForceDepShorteningPropConstant(true);
            BhargavaShorteningForceDepTrue->setMetabolicMuscleSet(mms);
            osimModel.addProbe(BhargavaShorteningForceDepTrue);
            cout << probeCounter++ << ") Added MuscleMetabolicPowerProbeBhargava2004 to measure SHORTENING muscle metabolic power (setUsingForceDepShorteningPropConstant = true)" << endl;

            // MuscleMetabolicPowerProbeBhargava2004 Power Probe: SHORTENING HEAT RATE (setUsingForceDepShorteningPropConstant = false)
            MuscleMetabolicPowerProbeBhargava2004* BhargavaShorteningForceDepFalse = new MuscleMetabolicPowerProbeBhargava2004(*BhargavaShorteningForceDepTrue);
            BhargavaShorteningForceDepFalse->setName("BhargavaShorteningForceDepFalse");
            BhargavaShorteningForceDepFalse->setUsingForceDepShorteningPropConstant(false);
            osimModel.addProbe(BhargavaShorteningForceDepFalse);
            cout << probeCounter++ << ") Added MuscleMetabolicPowerProbeBhargava2004 to measure SHORTENING muscle metabolic power (setUsingForceDepShorteningPropConstant = false)" << endl;


            // MuscleMetabolicPowerProbeUmberger2003 Power Probe: SHORTENING HEAT RATE
            MuscleMetabolicPowerProbeUmberger2003* UmbergerShortening = new MuscleMetabolicPowerProbeUmberger2003(false, true, false, false);
            UmbergerShortening->setName("UmbergerShortening");
            UmbergerShortening->setMetabolicMuscleSet(mms);
            osimModel.addProbe(UmbergerShortening);
            cout << probeCounter++ << ") Added MuscleMetabolicPowerProbeUmberger2003 to measure SHORTENING muscle metabolic power" << endl;

            // MuscleMetabolicPowerProbeBhargava2004 Power Probe: MECHANICAL WORK RATE (unnormalized)
            MuscleMetabolicPowerProbeBhargava2004* BhargavaWorkUnnormalized = new MuscleMetabolicPowerProbeBhargava2004(false, false, false, false, true);
            BhargavaWorkUnnormalized->setName("BhargavaWorkUnnormalized");
            BhargavaWorkUnnormalized->setMetabolicMuscleSet(mms);
            osimModel.addProbe(BhargavaWorkUnnormalized);
            cout << probeCounter++ << ") Added MuscleMetabolicPowerProbeBhargava2004 to measure MECHANICAL WORK RATE (unnormalized)" << endl;

            // MuscleMetabolicPowerProbeUmberger2003 Power Probe: MECHANICAL WORK RATE (unnormalized)
            MuscleMetabolicPowerProbeUmberger2003* UmbergerWorkUnnormalized = new MuscleMetabolicPowerProbeUmberger2003(false, false, false, true);
            UmbergerWorkUnnormalized->setName("UmbergerWorkUnnormalized");
            UmbergerWorkUnnormalized->setMetabolicMuscleSet(mms);
            osimModel.addProbe(UmbergerWorkUnnormalized);
            cout << probeCounter++ << ") Added MuscleMetabolicPowerProbeUmberger2003 to measure MECHANICAL WORK RATE (unnormalized)" << endl;

            // MuscleMetabolicPowerProbeBhargava2004 Power Probe: MECHANICAL WORK RATE (normalized)
            MuscleMetabolicPowerProbeBhargava2004* BhargavaWorkNormalized = new MuscleMetabolicPowerProbeBhargava2004(false, false, false, false, true);
            BhargavaWorkNormalized->setName("BhargavaWorkNormalized");
            BhargavaWorkNormalized->setMechanicalWorkRateNormalizedToMuscleMass(true);
            BhargavaWorkNormalized->setMetabolicMuscleSet(mms);
            osimModel.addProbe(BhargavaWorkNormalized);
            cout << probeCounter++ << ") Added MuscleMetabolicPowerProbeBhargava2004 to measure MECHANICAL WORK RATE (normalized)" << endl;

            // MuscleMetabolicPowerProbeUmberger2003 Power Probe: MECHANICAL WORK RATE (normalized)
            MuscleMetabolicPowerProbeUmberger2003* UmbergerWorkNormalized = new MuscleMetabolicPowerProbeUmberger2003(false, false, false, true);
            UmbergerWorkNormalized->setName("UmbergerWorkNormalized");
            UmbergerWorkNormalized->setMechanicalWorkRateNormalizedToMuscleMass(true);
            UmbergerWorkNormalized->setMetabolicMuscleSet(mms);
            osimModel.addProbe(UmbergerWorkNormalized);
            cout << probeCounter++ << ") Added MuscleMetabolicPowerProbeUmberger2003 to measure MECHANICAL WORK RATE (normalized)" << endl;

            // -------------------------------------------------------------------------------------------------

            // MuscleMetabolicPowerProbeBhargava2004 Power Probe
            MuscleMetabolicPowerProbeBhargava2004* metabolicPowerProbeBhargava = new MuscleMetabolicPowerProbeBhargava2004(true, true, true, true, true);
            metabolicPowerProbeBhargava->setName("metabolicPowerBhargava");
            metabolicPowerProbeBhargava->setMetabolicMuscleSet(mms);
            metabolicPowerProbeBhargava->setOperation("value");
            osimModel.addProbe(metabolicPowerProbeBhargava);
            cout << probeCounter++ << ") Added MuscleMetabolicPowerProbeBhargava2004 to measure TOTAL muscle metabolic power" << endl;
            
            // MuscleMetabolicPowerProbeBhargava2004 Energy Probe
            MuscleMetabolicPowerProbeBhargava2004* metabolicEnergyProbeBhargava = new MuscleMetabolicPowerProbeBhargava2004(*metabolicPowerProbeBhargava);   // use copy constructor
            metabolicEnergyProbeBhargava->setOperation("integrate");
            osimModel.addProbe(metabolicEnergyProbeBhargava);
            cout << probeCounter++ << ") Added MuscleMetabolicPowerProbeBhargava2004 to measure TOTAL muscle metabolic energy" << endl;
            
            // MuscleMetabolicPowerProbeUmberger2003 Power Probe
            MuscleMetabolicPowerProbeUmberger2003* metabolicPowerProbeUmberger = new MuscleMetabolicPowerProbeUmberger2003(true, true, true, true);
            metabolicPowerProbeUmberger->setName("metabolicPowerUmberger");
            metabolicPowerProbeUmberger->setMetabolicMuscleSet(mms);
            metabolicPowerProbeUmberger->setOperation("value");
            osimModel.addProbe(metabolicPowerProbeUmberger);
            cout << probeCounter++ << ") Added MuscleMetabolicPowerProbeUmberger2003 to measure TOTAL muscle metabolic power" << endl;
            
            // MuscleMetabolicPowerProbeUmberger2003 Energy Probe
            MuscleMetabolicPowerProbeUmberger2003* metabolicEnergyProbeUmberger = new MuscleMetabolicPowerProbeUmberger2003(*metabolicPowerProbeUmberger);   // use copy constructor
            metabolicEnergyProbeUmberger->setOperation("integrate");
            osimModel.addProbe(metabolicEnergyProbeUmberger);
            cout << probeCounter++ << ") Added MuscleMetabolicPowerProbeUmberger2003 to measure TOTAL muscle metabolic energy" << endl;

            cout << "\n" << endl;
        }



        // --------------------------------------------------------------------
        // ADD REPORTERS
        // --------------------------------------------------------------------
        ProbeReporter* probeReporter = new ProbeReporter(&osimModel);
        osimModel.addAnalysis(probeReporter);
        ForceReporter* forceReporter = new ForceReporter(&osimModel);
        osimModel.addAnalysis(forceReporter);
        MuscleAnalysis* muscleReporter = new MuscleAnalysis(&osimModel);
        osimModel.addAnalysis(muscleReporter);
        osimModel.print("testProbesModel.osim");
        osimModel.printBasicInfo(cout);
        // --------------------------------------------------------------------
        
      
        





        // --------------------------------------------------------------------
        // SYSTEM INTEGRATION
        // --------------------------------------------------------------------
		//Initialize the MB system
		SimTK::State& si = osimModel.initSystem();
        SimTK::Vector testRealInitConditions = forceSquaredProbeTwiceReportedIndividually3->getProbeOutputs(si);

        cout << "System mass = " << osimModel.getMatterSubsystem().calcSystemMass(si) << " kg." << endl;

		//Initialize the muscles 
		if(config_mdlComponents[0] == true){
			     
            switch(config_musclemodel){
                case 0: 
                {
                    muscleThEq->setActivation(si,muscleControl->getInitialExcitation(0.0));
		            muscleThEq->setDefaultActivation(muscleControl->getInitialExcitation(0.0));		           
		            muscleThEq->setDefaultFiberLength(mcl_optFibL);
		            muscleThEq->setFiberLength(si,mcl_optFibL);
		            muscleThEq->setOptimalFiberLength(mcl_optFibL);	
                }break;
                case 1: 
                {
                    muscleEq->setActivation(si,muscleControl->getInitialExcitation(0.0));
		            muscleEq->setDefaultActivation(muscleControl->getInitialExcitation(0.0));		           
		            muscleEq->setDefaultFiberLength(mcl_optFibL);
		            muscleEq->setFiberLength(si,mcl_optFibL);
		            muscleEq->setOptimalFiberLength(mcl_optFibL);		
                }break;
                case 2: 
                {
                    muscleAcc->setActivation(si,muscleControl->getInitialExcitation(0.0));
		            muscleAcc->setDefaultActivation(muscleControl->getInitialExcitation(0.0));		           
		            muscleAcc->setDefaultFiberLength(mcl_optFibL);
		            muscleAcc->setFiberLength(si,mcl_optFibL);
		            muscleAcc->setOptimalFiberLength(mcl_optFibL);	 	
                }break;
                default:
                    cout<<"..Invalid Muscle Model Selection " << endl;
            }
        

			osimModel.getMultibodySystem().realize(si,SimTK::Stage::Velocity);
			//Compute initial conditions for muscles
            SimTK::Vector weights(si.getNY());
            weights = 1;
            SimTK::Vector cweights(si.getNYErr());
            cweights = 1;
            SimTK::Vector yerrest(si.getNY());
            yerrest = 1;
            SimTK::Real tol = 1e-6;

            //cout << si.getYErr() << endl;
            osimModel.getMultibodySystem().project(si,tol);

            //cout << si.getYErr() << endl;            
            osimModel.getMultibodySystem().realize(si,SimTK::Stage::Velocity);
			//cout << si.getY() << endl;

            osimModel.getMultibodySystem().realize(si,SimTK::Stage::Dynamics);
            osimModel.equilibrateMuscles(si);
            
		}
        SimTK::State siInit(si);


        // Get initial muscle and system energy
        // ---------------------------------------
        //double Emuscle0 = muscWorkProbe.getRecordValues(si).get(0);
        //double Ejoint0 = jointWorkProbe.getRecordValues(si).get(0);
        //double Esys0 = sysEnergyProbe.getRecordValues(si).get(0);
        //cout << "-----------------------------------------------------\n" << endl;
        //cout << "Muscle work at start of simulation = " << Emuscle0 << endl;
        //cout << "Joint work at start of simulation = " << Ejoint0 << endl;
        //cout << "System energy at start of simulation = " << Esys0 << endl;
        //cout << "Total energy = " << Esys0 + Emuscle0 + Ejoint0 << endl; 


		// Create the integrator and manager for the simulation	
        // --------------------------------------------------------
		//SimTK::RungeKuttaFeldbergIntegrator integrator(osimModel.getMultibodySystem());
		SimTK::RungeKuttaMersonIntegrator integrator(osimModel.getMultibodySystem());
		//CPodesIntegrator integrator(osimModel.getMultibodySystem(),CPodes::BDF,CPodes::Newton);
		integrator.setMaximumStepSize(int_maxstep);
		integrator.setMinimumStepSize(int_minstep);
		integrator.setAccuracy(int_accuracy);
		//integrator.setAbsoluteTolerance(int_tolerance);
		Manager manager(osimModel, integrator);

		// Print out details of the model
		//osimModel.printDetailedInfo(si, cout);

		// Print out the initial position and velocity states
		//si.getQ().dump("Initial q's"); // block positions
		//si.getU().dump("Initial u's"); // block velocities
		//cout << "Initial time: " << si.getTime() << endl;


        //Integrate from t0 to t1
		manager.setInitialTime(t0);
		manager.setFinalTime(t1);
		cout<<"\nIntegrating from " << t0 << " to " << t1 << endl;


		clock_t startTime = clock();
		manager.integrate(si);
		double simTime = 1.e3*(clock()-startTime)/(double)CLOCKS_PER_SEC;
        cout << "Simulation took " << simTime << " ms" << endl;

        
		// Save the simulation results (states, forces, probes)
        // -----------------------------------------------------
        Storage states(manager.getStateStorage());
        states.print("testProbes_states.sto");
        probeReporter->getProbeStorage().print("testProbes_probes.sto");
        forceReporter->getForceStorage().print("testProbes_forces.sto");
        muscleReporter->getNormalizedFiberLengthStorage()->print("testProbes_normalizedFiberLength.sto");
        cout << "\nDone with printing results..." << endl;
        

        // Test a bunch of probe outputs
        // -------------------------------
        osimModel.getMultibodySystem().realize(si, SimTK::Stage::Acceleration);
        ASSERT_EQUAL(forceSquaredProbeTwiceScaled->getProbeOutputs(si)(0), gain1*forceSquaredProbeTwice->getProbeOutputs(si)(0), 1e-4, __FILE__, __LINE__, "Error with 'scale' operation.");
        ASSERT_EQUAL(forceProbeScale->getProbeOutputs(si)(0), gain2*forceProbe->getProbeOutputs(si)(0), 1e-4, __FILE__, __LINE__, "Error with 'scale' operation.");
        ASSERT_EQUAL(forceSquaredProbe->getProbeOutputs(si)(0), forceSquaredProbeTwiceScaled->getProbeOutputs(si)(0), 1e-4, __FILE__, __LINE__, "forceSquaredProbeTwiceScaled != forceSquaredProbe.");
        ASSERT_EQUAL(forceSquaredProbe->getProbeOutputs(si)(0), pow(forceProbe->getProbeOutputs(si)(0), 2), 1e-4, __FILE__, __LINE__, "Error with forceSquaredProbe probe.");
        ASSERT_EQUAL(forceSquaredProbeTwice->getProbeOutputs(si)(0), 2*pow(forceProbe->getProbeOutputs(si)(0), 2), 1e-4, __FILE__, __LINE__, "Error with forceSquaredProbeTwice probe.");
        for (int i=0; i<initCondVec.size(); ++i)  {
            stringstream myError;
            myError << "Initial condition[" << i << "] for vector integration is not being correctly applied." << endl;
            ASSERT_EQUAL(testRealInitConditions(i), initCondVec(i), 1e-4, __FILE__, __LINE__, myError.str());
            //if (testRealInitConditions(i) != initCondVec(i))
            //    cout << "WARNING: Initial condition[" << i << "] for vector integration is not being correctly applied.\nThis is actually an error, but I have made it into a warning for now so that the test passes..." << endl;
        }

        // Test system energy - work using
        // "integrate" operation on muscWorkProbe, jointWorkProbe
        // "value" operation on sysEnergyProbe
        // Not quite working yet with Matt's muscles.
        // Will ignore for now until testMuscles passes.
        // ---------------------------------------------------------
        //double muscleWorkAtEnd = muscWorkProbe.getRecordValues(si).get(0);
        //double jointWorkAtEnd = jointWorkProbe.getRecordValues(si).get(0);
        //double systemEnergyAtEnd = sysEnergyProbe.getRecordValues(si).get(0);
        //double ESysMinusWork = systemEnergyAtEnd - muscleWorkAtEnd - jointWorkAtEnd; 
        //cout << "Muscle work at end of simulation = " << muscleWorkAtEnd << endl;
        //cout << "Joint work at end of simulation = " << jointWorkAtEnd << endl;
        //cout << "System energy at end of simulation = " << systemEnergyAtEnd << endl;
        //cout << "Total system energy - work = " << ESysMinusWork << endl; 
        //ASSERT_EQUAL(Esys0, ESysMinusWork, 1e-4, __FILE__, __LINE__, "System energy with muscle not conserved.");

        

	}
    catch (const std::exception& ex)
    {
        cout << ex.what() << endl;
		return 1;
    }
    catch (...)
    {
        cout << "UNRECOGNIZED EXCEPTION" << endl;
        return 1;
    }

	
    cout << "\ntestProbes completed successfully.\n";
	return 0;
}


