/* -------------------------------------------------------------------------- *
 *                         OpenSim:  testWrapping.cpp                         *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2012 Stanford University and the Authors                *
 * Author(s): Ajay Seth                                                       *
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
// INCLUDE
#include <OpenSim/OpenSim.h>
#include <OpenSim/Auxiliary/auxiliaryTestFunctions.h>

#include "simbody/internal/CableTrackerSubsystem.h"
#include "simbody/internal/CablePath.h"

#include <exception>
#include <set>
#include <string>
#include <iostream>
#include <stdio.h>

using namespace OpenSim;
using namespace SimTK;
using namespace std;


bool useVisualizer = true;
bool reportPathInfo = false; // if true write path stats to file after each step

Real restLengthRatio = 0.7;
Real damping = 1;
Real maxIsoForceToStiffness = 100;

Transform X_GC;
Vec3 X_GC_p(0.705118,0.170067,-0.240247);
Vec3 X_GC_R_XYZ(-2.28331,0.819462,2.29111);

void simulate(Model& osimModel, State& si, double initialTime, double finalTime, string simulationName="");
void simulateModelWithMuscles(const string &modelFile, double finalTime, double activation=0.5);
void simulateModelWithPassiveMuscles(const string &modelFile, double finalTime);
void simulateModelWithoutMuscles(const string &modelFile, double finalTime);
void simulateModelWithLigaments(const string &modelFile, double finalTime);
void simulateModelWithCables(const string &modelFile, double finalTime);

class TestInfo {
public:
    TestInfo(String modelFilename, double simulationDuration) :
        filename(modelFilename), duration(simulationDuration) {};
    TestInfo(String modelFilename) :
        filename(modelFilename), duration(1) {};
    String filename;
    Real duration;
};

int main_old()
{
    SimTK::Array_<std::string> failures;

    try{// performance with multiple muscles and wrapping in upper-exremity
        simulateModelWithMuscles("TestShoulderModel.osim", 0.02);}
    catch (const std::exception& e) {
        std::cout << "Exception: " << e.what() << std::endl;
        failures.push_back("TestShoulderModel (multiple wrap)"); }

    if (!failures.empty()) {
        cout << "Done, with failure(s): " << failures << endl;
        return 1;
    }

    cout << "Done" << endl;
    return 0;
}

int main()
{
    X_GC.updR().setRotationToBodyFixedXYZ(X_GC_R_XYZ);
    X_GC.updP() = X_GC_p;

    SimTK::Array_<TestInfo> tests;
    tests.push_back(TestInfo("wrap_jump.osim"));
//    tests.push_back(TestInfo("wrap_ellipsoid.osim"));


//    tests.push_back(TestInfo("models/test_wrapCylinder_vasint.osim"));
//    tests.push_back(TestInfo("models/test_wrapCylinder_vasint_bfsh.osim"));
//    tests.push_back(TestInfo("models/test_wrapEllipsoid_vasint.osim"));
//    tests.push_back(TestInfo("models/test_wrapEllipsoid_vasint_bfsh.osim",2));
//    tests.push_back(TestInfo("models/arm26_crop.osim"));
//    tests.push_back(TestInfo("models/gait2392_pelvisFixed.osim",0.05));
//    tests.push_back(TestInfo("models/Arnold2010_pelvisFixed_crop.osim"));
//    tests.push_back(TestInfo("models/TestShoulderModel_crop.osim"));
//    tests.push_back(TestInfo("models/Arnold2010_pelvisFixed.osim"));
//    tests.push_back(TestInfo("models/TestShoulderModel.osim"));

    SimTK::Array_<String> failures;

    for (unsigned int i = 0; i < tests.size(); ++i) {
        cout << "testing " << tests[i].filename << " for " << tests[i].duration << " s" << endl;
        try { // performance with cylnder wrapping
            simulateModelWithMuscles(tests[i].filename, tests[i].duration, 0.4);
//            simulateModelWithPassiveMuscles(tests[i].filename, tests[i].duration);
//            simulateModelWithoutMuscles(tests[i].filename, tests[i].duration);
            simulateModelWithLigaments(tests[i].filename, tests[i].duration);
            simulateModelWithCables(tests[i].filename, tests[i].duration);

        } catch (const std::exception& e) {
            std::cout << "Exception: " << e.what() << std::endl;
            failures.push_back(tests[i].filename);
        }
    }

    if (!failures.empty()) {
        cout << "Done, with failure(s): " << failures << endl;
        return 1;
    }

    cout << "Done" << endl;
    return 0;
}


class ObstacleInfo {
public:
    String bodyName;
    Transform X_BS; // X_BS.p used for via point location
    WrapObject* wrapObjectPtr;
    Vec3 P_S;
    Vec3 Q_S;
    bool isVia;
    bool isActive;
};

class CableInfo {
public:
    String orgBodyName;
    Vec3 orgLoc;
    String insBodyName;
    Vec3 insLoc;
    Real length;

    Array<ObstacleInfo> obstacles;
};

// This force element implements an elastic cable of a given nominal length,
// and a stiffness k that generates a k*x force opposing stretch beyond
// nominal. There is also a damping term c*xdot that applies only when the
// cable is stretched and is being extended (x>0 && xdot>0). We keep track
// of dissipated power here so we can use conservation of energy to check that
// the cable and force element aren't obviously broken.
class MyCableSpringImpl : public SimTK::Force::Custom::Implementation {
public:
    MyCableSpringImpl(const GeneralForceSubsystem& forces,
                      const CablePath& path,
                      Real stiffness, Real nominal, Real damping)
    :   forces(forces), path(path), k(stiffness), x0(nominal), c(damping)
    {   assert(stiffness >= 0 && nominal >= 0 && damping >= 0); }

    const CablePath& getCablePath() const {return path;}

    // Must be at stage Velocity. Evalutes tension if necessary.
    Real getTension(const State& state) const {
        ensureTensionCalculated(state);
        return Value<Real>::downcast(forces.getCacheEntry(state, tensionx));
    }

    // Must be at stage Velocity.
    Real getPowerDissipation(const State& state) const {
        const Real stretch = calcStretch(state);
        if (stretch == 0) return 0;
        const Real rate = path.getCableLengthDot(state);
        return k*stretch*std::max(c*rate, -1.)*rate;
    }

    // This integral is always available.
    Real getDissipatedEnergy(const State& state) const {
        return forces.getZ(state)[workx];
    }

    //--------------------------------------------------------------------------
    //                       Custom force virtuals

    // Ask the cable to apply body forces given the tension calculated here.
    void calcForce(const State& state, Vector_<SpatialVec>& bodyForces,
                   Vector_<Vec3>& particleForces, Vector& mobilityForces) const
                   OVERRIDE_11
    {   path.applyBodyForces(state, getTension(state), bodyForces); }

    // Return the potential energy currently stored by the stretch of the cable.
    Real calcPotentialEnergy(const State& state) const OVERRIDE_11 {
        const Real stretch = calcStretch(state);
        if (stretch == 0) return 0;
        return k*square(stretch)/2;
    }

    // Allocate the state variable for tracking dissipated energy, and a
    // cache entry to hold the calculated tension.
    void realizeTopology(State& state) const OVERRIDE_11 {
        Vector initWork(1, 0.);
        workx = forces.allocateZ(state, initWork);
        tensionx = forces.allocateLazyCacheEntry(state, Stage::Velocity,
                                             new Value<Real>(NaN));
    }

    // Report power dissipation as the derivative for the work variable.
    void realizeAcceleration(const State& state) const OVERRIDE_11 {
        Real& workDot = forces.updZDot(state)[workx];
        workDot = getPowerDissipation(state);
    }
    //--------------------------------------------------------------------------

private:
    // Return the amount by which the cable is stretched beyond its nominal
    // length or zero if the cable is slack. Must be at stage Position.
    Real calcStretch(const State& state) const {
        const Real stretch = path.getCableLength(state) - x0;
        return std::max(stretch, 0.);
    }

    // Must be at stage Velocity to calculate tension.
    Real calcTension(const State& state) const {
        const Real stretch = calcStretch(state);

//        cout << "cable stretch = " << stretch << endl;

        if (stretch == 0) return 0;
        const Real rate = path.getCableLengthDot(state);
        if (c*rate < -1)
            cout << "c*rate=" << c*rate << "; limited to -1\n";
        const Real tension = k*stretch*(1+std::max(c*rate,-1.));
//        cout << "tension=" << tension << endl;
        return tension;
    }

    // If state is at stage Velocity, we can calculate and store tension
    // in the cache if it hasn't already been calculated.
    void ensureTensionCalculated(const State& state) const {
        if (forces.isCacheValueRealized(state, tensionx))
            return;
        Value<Real>::updDowncast(forces.updCacheEntry(state, tensionx))
            = calcTension(state);
        forces.markCacheValueRealized(state, tensionx);
    }

    const GeneralForceSubsystem&    forces;
    CablePath                       path;
    Real                            k, x0, c;
    mutable ZIndex                  workx;
    mutable CacheEntryIndex         tensionx;
};

// A nice handle to hide most of the cable spring implementation. This defines
// a user's API.
class MyCableSpring : public SimTK::Force::Custom {
public:
    MyCableSpring(GeneralForceSubsystem& forces, const CablePath& path,
                  Real stiffness, Real nominal, Real damping)
    :   SimTK::Force::Custom(forces, new MyCableSpringImpl(forces,path,
                                                    stiffness,nominal,damping))
    {}

    // Expose some useful methods.
    const CablePath& getCablePath() const
    {   return getImpl().getCablePath(); }
    Real getTension(const State& state) const
    {   return getImpl().getTension(state); }
    Real getPowerDissipation(const State& state) const
    {   return getImpl().getPowerDissipation(state); }
    Real getDissipatedEnergy(const State& state) const
    {   return getImpl().getDissipatedEnergy(state); }

private:
    const MyCableSpringImpl& getImpl() const
    {   return dynamic_cast<const MyCableSpringImpl&>(getImplementation()); }
};

static Array_<State> saveStates;
// This gets called periodically to dump out interesting things about
// the cables and the system as a whole.
class PrintCableStats : public PeriodicEventReporter {
public:

    PrintCableStats(const MultibodySystem& mbs, const MyCableSpring& cable,
            string filename, Real interval)
    :   PeriodicEventReporter(interval),
        mbs(mbs), cable(cable) {
        outfile = fopen(filename.c_str(), "w");
        fprintf(outfile, "%8s %10s %10s %10s %10s %10s %10s %10s\n",
            "time", "length", "rate", "integ_rate", "unitpow", "tension", "disswork",
            "CPU_time");
    }

    /** This is the implementation of the EventReporter virtual. **/
    void handleEvent(const State& state) const OVERRIDE_11 {
        const CablePath& path = cable.getCablePath();
        fprintf(outfile, "%8g %10.4g %10.4g %10.4g %10.4g %10.4g %10.4g %g\n",
            state.getTime(),
            path.getCableLength(state),
            path.getCableLengthDot(state),
            path.getIntegratedCableLengthDot(state),
            path.calcCablePower(state, 1), // unit power
            cable.getTension(state),
            cable.getDissipatedEnergy(state),
            cpuTime());

        saveStates.push_back(state);
    }

    ~PrintCableStats() {
//        cout << "~PrintCableStats()" << endl;
        fclose(outfile);
    }
private:
    const MultibodySystem&  mbs;
    MyCableSpring           cable;
    FILE*                   outfile;
};

class PrintLigamentStats : public PeriodicEventReporter {
public:

    PrintLigamentStats(const MultibodySystem& mbs, const Ligament& ligament,
            string filename, Real interval)
    :   PeriodicEventReporter(interval),
        mbs(mbs), ligament(ligament) {
        outfile = fopen(filename.c_str(), "w");
        fprintf(outfile, "%8s %10s %10s %10s\n",
            "time", "length", "tension", "CPU_time");
    }

    /** This is the implementation of the EventReporter virtual. **/
    void handleEvent(const State& state) const OVERRIDE_11 {
        const GeometryPath& path = ligament.getGeometryPath();
        fprintf(outfile, "%8g %10.4g %10.4g %g\n",
            state.getTime(),
            path.getLength(state),
//            path.getCableLengthDot(state),
//            path.getIntegratedCableLengthDot(state),
//            path.calcCablePower(state, 1), // unit power
            ligament.getTension(state),
//            ligament.getDissipatedEnergy(state),
            cpuTime());

        saveStates.push_back(state);
    }

    ~PrintLigamentStats() {
//        cout << "~PrintLigamentStats()" << endl;
        fclose(outfile);
    }
private:
    const MultibodySystem&  mbs;
    const Ligament&         ligament;
    FILE*                   outfile;
};


void simulateModelWithMuscles(const string &modelFile, double finalTime, double activation)
{
    // Create a new OpenSim model
    Model osimModel(modelFile);
    osimModel.setUseVisualizer(useVisualizer);

    double initialTime = 0;

    // Create a prescribed controller that simply applies a function of the force
    PrescribedController actuatorController;
    actuatorController.setActuators(osimModel.updActuators());
    for (int i=0; i<actuatorController.getActuatorSet().getSize(); i++) {
        actuatorController.prescribeControlForActuator(i, new Constant(activation));
    }

    // add the controller to the model
    osimModel.addController(&actuatorController);
    osimModel.disownAllComponents(); // because PrescribedController is on stack

    // Initialize the system and get the state representing the state system
    SimTK::State& si = osimModel.initSystem();

    const Set<Muscle>& muscles = osimModel.getMuscles();
    for (int i=0; i<muscles.getSize(); i++){
        muscles[i].setActivation(si, activation); //setDisabled(si, true);
    }
    osimModel.equilibrateMuscles(si);

//    osimModel.printBasicInfo(cout);

    simulate(osimModel, si, initialTime, finalTime, "thelen");


}// end of simulateModelWithMusclesNoViz()



void simulateModelWithPassiveMuscles(const string &modelFile, double finalTime)
{
	// Create a new OpenSim model
	Model osimModel(modelFile);
	double initialTime = 0;

	// Show model visualizer
	osimModel.setUseVisualizer(useVisualizer);

	// Initialize the system and get the state representing the state system
	SimTK::State& si = osimModel.initSystem();

	const Set<Muscle>& muscles = osimModel.getMuscles();
	for (int i=0; i<muscles.getSize(); ++i){
		muscles[i].setActivation(si, 0); //setDisabled(si, true);
	}
	osimModel.equilibrateMuscles(si); 

    simulate(osimModel, si, initialTime, finalTime, "muscles");


}// end of simulateModelWithMusclesViz()

void simulateModelWithoutMuscles(const string &modelFile, double finalTime)
{
    // Create a new OpenSim model
    Model osimModel(modelFile);
    double initialTime = 0;

    // Show model visualizer
    osimModel.setUseVisualizer(useVisualizer);

    // remove all forces
    osimModel.updForceSet().clearAndDestroy();

    SimTK::State& si = osimModel.initSystem();

    simulate(osimModel, si, initialTime, finalTime, "noMuscles");

}// end of simulateModelWithoutMuscles()


void simulateModelWithLigaments(const string &modelFile, double finalTime)
{
    // Create a new OpenSim model
    Model osimModel(modelFile);
    double initialTime = 0;

    SimTK::State& si = osimModel.initSystem(); // init system so that we can get the initial path lengths

    // Show model visualizer
    osimModel.setUseVisualizer(useVisualizer);

    const Set<Muscle>& muscles = osimModel.getMuscles();
    Array<Ligament*> ligaments;

//    double initLen = muscles[0].getLength(si);

    double muscleMaxIsometricForce[muscles.getSize()];

    for (int i = 0; i < muscles.getSize(); ++i) {
        Ligament* lig = new Ligament();
        lig->updGeometryPath() = muscles[i].getGeometryPath();
        muscleMaxIsometricForce[i] = muscles[i].getMaxIsometricForce();

        // typical non-linear ligament force-length curve
//        double x[17] = {-10.00000000, -0.00200000, -0.00100000,  0.00000000,  0.00131000,  0.00281000,  0.00431000,  0.00581000,  0.00731000,  0.00881000,  0.01030000,  0.01180000,  0.01230000,  9.20000000,  9.20100000,  9.20200000, 20.00000000};
//        double y[17] = {0.00000000,  0.00000000,  0.00000000,  0.00000000,  0.01080000,  0.02570000,  0.04350000,  0.06520000,  0.09150000,  0.12300000,  0.16100000,  0.20800000,  0.22700000, 345.00000000, 345.00000000, 345.00000000, 345.00000000};
//        SimmSpline spline(17,x,y,"");
//        lig.setForceLengthCurve(spline);
//        lig.setMaxIsometricForce(stiffness);
//        lig.setRestingLength(restLength);

        double initialLength = muscles[i].getLength(si);
//        cout << "initial length = " << initialLength << endl;

        lig->setLinearStiffness(maxIsoForceToStiffness*muscleMaxIsometricForce[i],
                initialLength*restLengthRatio);
        lig->setDamping(damping);

        ligaments.append(lig);
    }

    osimModel.invalidateSystem();

    // remove all forces (muscles)
    osimModel.updForceSet().clearAndDestroy();

    // add new ligaments
    for (int i = 0; i < ligaments.getSize(); ++i) {
        osimModel.addForce(ligaments[i]);
    }

    osimModel.disownAllComponents(); // because Ligaments are on stack

//    SimTK::State& si = osimModel.initSystem();

    osimModel.buildSystem();
    MultibodySystem& system = osimModel.updMultibodySystem();

    if (reportPathInfo) {
        for (int i = 0; i < osimModel.getForceSet().getSize(); ++i) {
            Ligament* lig = dynamic_cast<Ligament*>(&osimModel.getForceSet()[i]);
            if (lig != 0) {
                char filename[1024];
                sprintf(filename, "%s_ligament%d.txt", modelFile.substr(0, modelFile.find(".osim",0)).c_str(), i);
    //            cout << "writing stats to " << filename << endl;
                string filename_str(filename);
                system.addEventReporter(new PrintLigamentStats(system, *lig, filename, 0.1*0.1));
            }
        }
    }


    SimTK::State& state = osimModel.initializeState();

//    viz.report(state);

    simulate(osimModel, state, initialTime, finalTime, "ligaments");

}// end of simulateModelWithoutMuscles()




void simulateModelWithCables(const string &modelFile, double finalTime)
{
    // Create a new OpenSim model
    Model osimModel(modelFile);
    double initialTime = 0;

    SimTK::State& si = osimModel.initSystem();

    Array<GeometryPath*> paths;
    vector<Real> maxIsoForces;
    Array<String> pathNames;
    int numLigaments = 0, numMuscles = 0;
    for (int i = 0; i < osimModel.getForceSet().getSize(); ++i) {
        Ligament* lig = dynamic_cast<Ligament*>(&osimModel.getForceSet()[i]);
        if (lig != 0) {
            numLigaments++;
            paths.append(&lig->updGeometryPath());
            pathNames.append(lig->getName());
            maxIsoForces.push_back(lig->getMaxIsometricForce());
            continue;
        }

        Muscle* mus = dynamic_cast<Muscle*>(&osimModel.getForceSet()[i]);
        if (mus != 0) {
            numMuscles++;
            paths.append(&mus->updGeometryPath());
            pathNames.append(mus->getName());
//            cout << mus->getName() << ": " << mus->getGeometryPath().getWrapSet().getSize() << endl;
            maxIsoForces.push_back(mus->getMaxIsometricForce());
            continue;
        }
    }

//    cout << "num ligaments = " << numLigaments << endl;
//    cout << "num muscles = " << numMuscles << endl;

    Array<CableInfo> cableInfos;
    std::set<String> wrapObjectsInUse;
    for (int i = 0; i < paths.getSize(); ++i) {
        CableInfo cableInfo;

        GeometryPath* geomPath = paths[i];
        const PathWrapSet& wrapSet = geomPath->getWrapSet();
        const PathPointSet& viaSet = geomPath->getPathPointSet();
        Array<PathPoint*> activePathPoints = geomPath->getCurrentPath(si);

        PathPoint* orgPoint = &viaSet[0];
        PathPoint* insPoint = &viaSet[viaSet.getSize()-1];

        cableInfo.length = geomPath->getLength(si);
//        cout << "initial length = " << cableInfo.length << endl;
        cableInfo.orgBodyName = orgPoint->getBody().getName();
        cableInfo.orgLoc = orgPoint->getLocation();
        cableInfo.insBodyName =  insPoint->getBody().getName();
        cableInfo.insLoc = insPoint->getLocation();

        int numVias = 0, numSurfs = 0;

//        for (int j = 0; j < activePathPoints.getSize(); ++j) {
//            PathPoint* pp = activePathPoints[j];
//            cout << pp << " pp" << j << " = " << pp->getName() << " @ " << pp->getBodyName() << ", loc = " << pp->getLocation() << endl;
//        }
//
//        for (int j = 0; j < viaSet.getSize(); ++j) {
//            PathPoint* pp = &viaSet[j];
//            cout << "mp" << j << " = " << pp->getName() << " @ " << pp->getBodyName() << ", loc = " << pp->getLocation() << endl;
//        }

        // add vias to cableInfo
        for (int j = 1; j < viaSet.getSize()-1; ++j) { // skip first (origin) and last (insertion) points
            const PathPoint& pp = viaSet[j];
            ObstacleInfo obs;
            obs.bodyName = pp.getBodyName();
            obs.X_BS.setP(pp.getLocation());
            obs.wrapObjectPtr=NULL;
            obs.P_S.setToNaN(); // not used for viapoint
            obs.Q_S.setToNaN(); // not used for viapoint
            obs.isVia = true;
            obs.isActive = true;
            cableInfo.obstacles.append(obs);
            numVias++;
        }

        Array<ObstacleInfo*> wrapObs;
        // add surfaces to cableInfo
        for (int j = 0; j < wrapSet.getSize(); j++) {
            const PathWrap& wrap = wrapSet[j];
            WrapObject* wrapObj = wrap.getWrapObject();
            ObstacleInfo* obs = new ObstacleInfo();
            obs->wrapObjectPtr = wrapObj;
            obs->bodyName = wrapObj->getBody().getName();
            obs->X_BS = wrapObj->getTransform();
            obs->isVia = false;
            // initially assume inactive
            obs->isActive = false;
            obs->P_S.setToNaN();
            obs->Q_S.setToNaN();
            wrapObs.append(obs);
            wrapObjectsInUse.insert(wrapObj->getName());
            numSurfs++;
        }

        // find active wrap surfaces and insert to obstacles list in order
        int obsIdx = 0;
        int viaPtIdx = 1; // skip first (origin) point
        for (int j = 1; j < activePathPoints.getSize()-1; ++j) { // skip first (origin) and last (insertion)
            PathPoint* pp = activePathPoints[j];
            if (*pp == viaSet[viaPtIdx]) {
                viaPtIdx++;
                obsIdx++;
                continue;
            }
            else { // next two path points should be a wrap point
                for (int k = 0; k < wrapSet.getSize(); ++k) {
                    const Vec3& wrapStartPointLoc = wrapSet[k].getPreviousWrap().r1;
                    if (!wrapStartPointLoc.isInf() && pp->getLocation().isNumericallyEqual(wrapStartPointLoc)) {
                        ObstacleInfo* obs = wrapObs[k]; // wrapSet and wrapObs are same ordered lists
//                        cout << "found wrap obs " << k << " name = " << obs->wrapObjectPtr->getName() << endl;
                        obs->isActive = true;
                        // pp and next pp are wrap points
                        Transform X_SB = obs->X_BS.invert();
                        PathPoint* pp_next = activePathPoints[++j]; // increment to next pp
                        obs->P_S = X_SB*pp->getLocation();
                        obs->Q_S = X_SB*pp_next->getLocation();
                        cableInfo.obstacles.insert(obsIdx++, *obs);
                        break;
                    }
                }
            }
        }

        // append inactive wrap surfaces to obstacle list
        // TODO find order of inactive wrap surfaces relative to via points
        for (int j = 0; j < wrapObs.getSize(); ++j) {
            ObstacleInfo* obs = wrapObs[j];
            if (!obs->isActive) {
                cableInfo.obstacles.append(*obs);
            }
        }

        cableInfos.append(cableInfo);

//        debugging
//
//        cout << pathNames[i] << " path, num pts " << activePathPoints.getSize() << ", num vias = " << numVias << ", num surfs = " << numSurfs << endl;
//
//
//        cout << "org" << " = " << orgPoint->getName() << " @ " << orgPoint->getBody().getName() << ", loc = " << orgPoint->getLocation() << endl;
//        cout << "ins" << " = " << insPoint->getName() << " @ " << insPoint->getBody().getName() << ", loc = " << insPoint->getLocation() << endl;
//
//        for (int j = 0; j < cableInfo.obstacles.getSize(); ++j) {
//            ObstacleInfo oi = cableInfo.obstacles[j];
//            cout << "ObsInfo " << j << ": via=" << oi.isVia << ", bodyName=" << oi.bodyName << ", loc=" << oi.X_BS.p() << ", P=" << oi.P_S << ", Q=" << oi.Q_S << endl;
//        }
//
//        for (int j = 0; j < wrapSet.getSize(); ++j) {
//            cout << "wrap object " << j << " name = " << wrapSet[j].getName() << endl;
//            cout << "wrap point 0 = " << wrapSet[j].getWrapPoint(0).getLocation() << endl;
//            cout << "wrap point 1 = " << wrapSet[j].getWrapPoint(1).getLocation() << endl;
//            const WrapResult& wr = wrapSet[j].getPreviousWrap();
//            cout << "wrap result r1 = " << wr.r1 << endl;
//            cout << "wrap result r2 = " << wr.r2 << endl;
//            cout << "wrap result startpt = " << wr.startPoint << endl;
//            cout << "wrap result endpt = " << wr.endPoint << endl;
//
////            for (int j = 0; j < wr.wrap_pts.getSize(); j++) {
////                cout << "wrap result pt[" << j << "] = " << wr.wrap_pts[j] << endl;
////            }
//        }
    }


//    cout << "num forces before removal = " << osimModel.getForceSet().getSize() << endl;

    // remove all OpenSim forces
    osimModel.updForceSet().clearAndDestroy();

    // remove all unused wrap objects
//    cout << "wraps in use:" << endl;
//    set<String>::iterator it;
//    for ( it=wrapObjectsInUse.begin() ; it != wrapObjectsInUse.end(); it++ ) {
//        cout << "    " << *it << endl;
//    }

    for (int i = 0; i < osimModel.getBodySet().getSize(); ++i) {
        Array<WrapObject*> toRemove;
        const WrapObjectSet& wraps = osimModel.getBodySet()[i].getWrapObjectSet();
        // XXX hack to remove wrap objects, which is not supported in the API
        WrapObjectSet *mutableWraps = const_cast<WrapObjectSet *>(&wraps);
//        cout << "culling wrap objs from " << osimModel.getBodySet()[i].getName() << ", num wraps = " << wraps.getSize() << endl;

        mutableWraps->setMemoryOwner(true);
        for (int j = 0; j < wraps.getSize(); ++j) {
            if (wrapObjectsInUse.find(wraps[j].getName()) == wrapObjectsInUse.end()) { // does not contain
                toRemove.append(&wraps[j]);
            }
        }

//        cout << "wraps to remove:" << endl;
//        for (int j = 0; j < toRemove.getSize(); ++j) {
//            cout << "    " << toRemove[j]->getName() << endl;
//        }

        for (int j = 0; j < toRemove.getSize(); ++j) {
            mutableWraps->remove(toRemove[j]);
        }
//        cout << "finished cull from " << osimModel.getBodySet()[i].getName() << ", num wraps = " << wraps.getSize() << endl;

    }

//    cout << "counting wraps in testWrapping..." << endl;
//    for (int i = 0; i < osimModel.getBodySet().getSize(); i++) {
//           const OpenSim::Body& body = osimModel.getBodySet()[i];
//           const WrapObjectSet& wrapObjects = body.getWrapObjectSet();
//           cout << body.getName() << ": wrap count = " << wrapObjects.getSize() << endl;
//    }

    // Show model visualizer
    osimModel.setUseVisualizer(useVisualizer);

    // Initialize the system and get the state representing the state system
//    SimTK::State& s = osimModel.initSystem();
    osimModel.buildSystem();

//    cout << "num forces after removal = " << osimModel.getForceSet().getSize() << endl;


    MultibodySystem& system = osimModel.updMultibodySystem();
    GeneralForceSubsystem& forceSubsystem = osimModel.updForceSubsystem();
    const SimbodyMatterSubsystem& matter = osimModel.getMatterSubsystem();
    const SimbodyEngine& engine = osimModel.getSimbodyEngine();

    // add cable system
    CableTrackerSubsystem cables(system);
    MyCableSpring* cableToReport;

    for (int i = 0; i < cableInfos.getSize(); ++i) {
        const CableInfo& cableInfo = cableInfos[i];

        const OpenSim::Body& orgBody = osimModel.getBodySet().get(cableInfo.orgBodyName);
        const OpenSim::Body& insBody = osimModel.getBodySet().get(cableInfo.insBodyName);
        const MobilizedBody& orgMobBody = matter.getMobilizedBody(orgBody.getIndex());
        const MobilizedBody& insMobBody = matter.getMobilizedBody(insBody.getIndex());
//        cout << "origin mob idx = " << orgBody.getIndex() << endl;
//        cout << "insertion mob idx = " << insBody.getIndex() << endl;


        CablePath path(cables, orgMobBody, cableInfo.orgLoc,   // origin
                            insMobBody, cableInfo.insLoc);  // termination

        // Add obstacles
        for (int j = 0; j < cableInfo.obstacles.getSize(); ++j) {
            ObstacleInfo oi = cableInfo.obstacles[j];
            const OpenSim::Body& osBody = osimModel.getBodySet().get(oi.bodyName);
            const MobilizedBody& mobBody = matter.getMobilizedBody(osBody.getIndex());

            if (oi.isVia) {
                CableObstacle::ViaPoint via(path, mobBody, oi.X_BS.p());
            }
            else {
                WrapSphere* wrapSphere = dynamic_cast<WrapSphere*>(oi.wrapObjectPtr);
                if (wrapSphere != 0) {
                    CableObstacle::Surface surf(path, mobBody,
                        oi.X_BS, SimTK::ContactGeometry::Sphere(wrapSphere->getRadius())); // along y
                    if (oi.isActive)
                        surf.setContactPointHints(oi.P_S, oi.Q_S);
                    else
                        surf.setDisabledByDefault(true);
                    continue;
                }

                WrapCylinder* wrapCyl = dynamic_cast<WrapCylinder*>(oi.wrapObjectPtr);
                if (wrapCyl != 0) {
                    CableObstacle::Surface surf(path, mobBody,
                        oi.X_BS, SimTK::ContactGeometry::Cylinder(wrapCyl->getRadius())); // along y
                    if (oi.isActive)
                        surf.setContactPointHints(oi.P_S, oi.Q_S);
                    else
                        surf.setDisabledByDefault(true);
                    continue;
                }

                WrapEllipsoid* wrapEllip = dynamic_cast<WrapEllipsoid*>(oi.wrapObjectPtr);
                if (wrapEllip != 0) {
                    CableObstacle::Surface surf(path, mobBody,
                        oi.X_BS, SimTK::ContactGeometry::Ellipsoid(wrapEllip->getRadii())); // along y
                    if (oi.isActive)
                        surf.setContactPointHints(oi.P_S, oi.Q_S);
                    else
                        surf.setDisabledByDefault(true);
                    continue;
                }

                WrapTorus* wrapTorus = dynamic_cast<WrapTorus*>(oi.wrapObjectPtr);
                if (wrapTorus != 0) {
                    Real tubeRadius = (wrapTorus->getOuterRadius()-wrapTorus->getInnerRadius())/2.0;
                    Real torusRadius = wrapTorus->getOuterRadius()-tubeRadius;
                    CableObstacle::Surface surf(path, mobBody,
                        oi.X_BS, SimTK::ContactGeometry::Torus(torusRadius, tubeRadius)); // along y
                    if (oi.isActive)
                        surf.setContactPointHints(oi.P_S, oi.Q_S);
                    else
                        surf.setDisabledByDefault(true);
                    continue;
                }

                cout << "unknown wrap object [" << oi.wrapObjectPtr->getName() << "], adding via point at origin" << endl;
                CableObstacle::ViaPoint via(path, mobBody, oi.X_BS.p());

            }
        }

        MyCableSpring cable(forceSubsystem, path, maxIsoForceToStiffness*maxIsoForces[i],
                cableInfo.length*restLengthRatio, damping);


        if (reportPathInfo) {
            char filename[1024];
            sprintf(filename, "%s_cable%d.txt", modelFile.substr(0, modelFile.find(".osim",0)).c_str(), i);
    //        cout << "writing stats to " << filename << endl;
            string filename_str(filename);
            system.addEventReporter(new PrintCableStats(system, cable, filename, 0.1*0.1));
        }
    }

//    system.invalidateSystemTopologyCache();
//    system.realizeTopology();
//    SimTK::State state = system.getDefaultState();
//    osimModel.copyDefaultStateIntoWorkingState();
//    matter.setUseEulerAngles(state, true);
//    system.realizeModel(state);
//    osimModel.initStateFromProperties(state);
//    system.realize(state, Stage::Instance);
//    modelViz.collectFixedGeometry(state);
//    viz.report(state);
//    cout << "Hit ENTER ...";
//    cout.flush();
//    getchar();


    SimTK::State& state = osimModel.initializeState();

//    viz.report(state);

//    for (int i = 0; i < cables.getNumCablePaths(); ++i) {
//        const CablePath& path = cables.getCablePath((CablePathIndex)i);
//        cout << "path " << i << " length = " << path.getCableLength(state) << endl;
//    }


    // compute initial path
    system.realize(state, Stage::Position);
    cout << "Cable initialization finished" << endl;
//    cout << "Hit ENTER ...";
//    cout.flush();
//    getchar();

    // Simulate it.
    if (reportPathInfo) {
        saveStates.clear(); saveStates.reserve(2000);
    }

    simulate(osimModel, state, initialTime, finalTime, "cables");

//    while (true) {
//        cout << "Hit ENTER FOR REPLAY, Q to quit ...";
//        cout.flush();
//        const char ch = getchar();
//        if (ch=='q' || ch=='Q') break;
//        for (unsigned i=0; i < saveStates.size(); ++i) {
//            Transform X_GC;
//            X_GC.updR().setRotationToBodyFixedXYZ(Vec3(-2.55534,0.797992,2.4824));
//            X_GC.setP(Vec3(0.702832,0.140739,-0.399551));
//            viz.setCameraTransform(X_GC);
//            viz.report(saveStates[i]);
//        }
//    }

}// end of simulateModelWithCables()




void simulate(Model& osimModel, State& si, double initialTime, double finalTime, string simulationName) {

    // use visualizer
    if (useVisualizer) {
        const ModelVisualizer& modelViz = osimModel.getVisualizer();
        const Visualizer& viz = modelViz.getSimbodyVisualizer();
        viz.setBackgroundColor(Vec3(0.6,0.6,0.6));
    //    viz.zoomCameraToShowAllGeometry();
        viz.setCameraTransform(X_GC);
    }

    //  osimModel.printBasicInfo(cout);

    // Dump model back out; no automated test provided here though.
    // osimModel.print(osimModel.getName() + "_out.osim");

    // Create the integrator and manager for the simulation.
    const double accuracy = 1.0e-4;
    SimTK::RungeKuttaMersonIntegrator integrator(osimModel.getMultibodySystem());
    integrator.setAccuracy(accuracy);

    Manager manager(osimModel, integrator);

    // Integrate from initial time to final time
    manager.setInitialTime(initialTime);
    manager.setFinalTime(finalTime);
//    cout << "\nIntegrating from " << initialTime << " to " << finalTime << endl;

    const double start = SimTK::realTime();
    integrator.resetAllStatistics();
    manager.integrate(si);
    printf("sim=%s \t; model=%s \t; duration=%f s \t; simulationtime=%f s \t; numsteps=%d \n",
            simulationName.c_str(), osimModel.getDocumentFileName().c_str(),
            finalTime-initialTime, SimTK::realTime()-start, integrator.getNumStepsTaken());
//    cout << "simulation=" << simulationName
//         << "; model=" << osimModel.getDocumentFileName()
//         << "; duration = " << finalTime-initialTime
//         << " s; simulation_time = " << SimTK::realTime()-start << " s" << endl;
//
//    cout << "integrator iterations = " << integrator.getNumStepsTaken() << endl;

    // Save the simulation results
//    Storage states(manager.getStateStorage());
//    states.print(osimModel.getName()+"_states.sto");
//    osimModel.updSimbodyEngine().convertRadiansToDegrees(states);
//    states.setWriteSIMMHeader(true);
//    states.print(osimModel.getName()+"_states_degrees.mot");
} // end of simulate()

