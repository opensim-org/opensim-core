/* ------------------------------------------------------------------------- *
*                  OpenSim:  testWrappping_Deprecated.cpp                    *
* -------------------------------------------------------------------------- *
* The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
* See http://opensim.stanford.edu and the NOTICE file for more information.  *
* OpenSim is developed at Stanford University and supported by the US        *
* National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
* through the Warrior Web program.                                           *
*                                                                            *
* Copyright (c) 2005-2023 Stanford University and the Authors                *
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

// "Deprecated" wrapping tests (i.e., tests that are not currently run in 
// continuous integratino) to either be removed or reincorporated in to the
// test suites (after refactor and review). 

#include <OpenSim/OpenSim.h>
#include <OpenSim/Auxiliary/auxiliaryTestFunctions.h>
using namespace OpenSim;
using namespace SimTK;
using namespace std;

class ObstacleInfo {
public:
    String bodyName;
    Transform X_BS; // X_BS.p used for via point location
    const WrapObject* wrapObjectPtr;
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
    {   ASSERT(stiffness >= 0 && nominal >= 0 && damping >= 0); }
    
    const CablePath& getCablePath() const {return path;}

    // Must be at stage Velocity. Evaluates tension if necessary.
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
            override
    {   path.applyBodyForces(state, getTension(state), bodyForces); }

    // Return the potential energy currently stored by the stretch of the cable.
    Real calcPotentialEnergy(const State& state) const override {
        const Real stretch = calcStretch(state);
        if (stretch == 0) return 0;
        return k*square(stretch)/2;
    }

    // Allocate the state variable for tracking dissipated energy, and a
    // cache entry to hold the calculated tension.
    void realizeTopology(State& state) const override {
        Vector initWork(1, 0.);
        workx = forces.allocateZ(state, initWork);
        tensionx = forces.allocateLazyCacheEntry(state, Stage::Velocity,
                new Value<Real>(NaN));
    }

    // Report power dissipation as the derivative for the work variable.
    void realizeAcceleration(const State& state) const override {
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
class ShowStuff : public PeriodicEventReporter {
public:
    
    ShowStuff(const MultibodySystem& mbs,
            Real interval)
            :   PeriodicEventReporter(interval) {}

    static void showHeading(std::ostream& o) {
        printf("%8s %10s %10s %10s %10s %10s %10s %10s %10s %12s\n",
                "time", "length", "rate", "integ-rate", "unitpow", "tension", "disswork",
                "KE", "PE", "KE+PE-W");
    }

    /** This is the implementation of the EventReporter virtual. **/
    void handleEvent(const State& state) const override {
        //        const CablePath& path1 = cable1.getCablePath();
        ////        const CablePath& path2 = cable2.getCablePath();
        //        printf("%8g %10.4g %10.4g %10.4g %10.4g %10.4g %10.4g CPU=%g\n",
        //            state.getTime(),
        //            path1.getCableLength(state),
        //            path1.getCableLengthDot(state),
        //            path1.getIntegratedCableLengthDot(state),
        //            path1.calcCablePower(state, 1), // unit power
        //            cable1.getTension(state),
        //            cable1.getDissipatedEnergy(state),
        //            cpuTime());
        //        printf("%8s %10.4g %10.4g %10.4g %10.4g %10.4g %10.4g %10.4g %10.4g %12.6g\n",
        //            "",
        //            path2.getCableLength(state),
        //            path2.getCableLengthDot(state),
        //            path2.getIntegratedCableLengthDot(state),
        //            path2.calcCablePower(state, 1), // unit power
        //            cable2.getTension(state),
        //            cable2.getDissipatedEnergy(state),
        //            mbs.calcKineticEnergy(state),
        //            mbs.calcPotentialEnergy(state),
        //            mbs.calcEnergy(state)
        //                + cable1.getDissipatedEnergy(state)
        //                + cable2.getDissipatedEnergy(state));
        saveStates.push_back(state);
    }
private:
    //    MyCableSpring           cable1;
};

void simulateModelWithPassiveMuscles(const string &modelFile, double finalTime)
{
    // Create a new OpenSim model
    Model osimModel(modelFile);
    double initialTime = 0;
    
    // Show model visualizer
    osimModel.setUseVisualizer(true);
    
    // Initialize the system and get the state representing the state system
    SimTK::State& si = osimModel.initSystem();

    const Set<Muscle>& muscles = osimModel.getMuscles();
    for (int i=0; i<muscles.getSize(); ++i){
        muscles[i].setActivation(si, 0); //setDisabled(si, true);
    }
    osimModel.equilibrateMuscles(si); 

    const ModelVisualizer& modelViz = osimModel.getVisualizer();
    const Visualizer& viz = modelViz.getSimbodyVisualizer();
    viz.report(si);

    simulate(osimModel, si, initialTime, finalTime);


}

void simulateModelWithoutMuscles(const string &modelFile, double finalTime)
{
    // Create a new OpenSim model
    Model osimModel(modelFile);
    double initialTime = 0;

    // Show model visualizer
    osimModel.setUseVisualizer(true);

    // remove all forces
    auto& forces = osimModel.updForceSet();
    for(int i = 0; i<forces.getSize(); ++i){
        forces.remove(i);
    }

    SimTK::State& si = osimModel.initSystem();

    simulate(osimModel, si, initialTime, finalTime);

}

void simulateModelWithLigaments(const string &modelFile, double finalTime)
{
    // Create a new OpenSim model
    Model osimModel(modelFile);
    double initialTime = 0;

    // Show model visualizer
    osimModel.setUseVisualizer(true);

    // TODO replace muscles with ligaments
    //    Set<OpenSim::Force> &forces = osimModel.updForceSet();
    //    for(int i = 0; i<forces.getSize(); ++i){
    //        forces.remove(&forces[i]);
    //    }

    SimTK::State& si = osimModel.initSystem();

    simulate(osimModel, si, initialTime, finalTime);

}

void simulateModelWithCables(const string &modelFile, double finalTime)
{
    // Create a new OpenSim model
    Model osimModel(modelFile);
    double initialTime = 0;

    SimTK::State& si = osimModel.initSystem();

    Array<GeometryPath*> paths;
    Array<String> pathNames;
    int numLigaments = 0, numMuscles = 0;
    for (int i = 0; i < osimModel.getForceSet().getSize(); ++i) {
        Ligament* lig = dynamic_cast<Ligament*>(&osimModel.getForceSet()[i]);
        if (lig != 0) {
            numLigaments++;
            paths.append(&lig->updGeometryPath());
            pathNames.append(lig->getName());
            continue;
        }

        Muscle* mus = dynamic_cast<Muscle*>(&osimModel.getForceSet()[i]);
        if (mus != 0) {
            numMuscles++;
            paths.append(&mus->updGeometryPath());
            pathNames.append(mus->getName());
            cout << mus->getName() << ": " 
                 << mus->getGeometryPath().getWrapSet().getSize() << endl;
            continue;
        }
    }

    cout << "num ligaments = " << numLigaments << endl;
    cout << "num muscles = " << numMuscles << endl;

    Array<CableInfo> cableInfos;
    std::set<String> wrapObjectsInUse;
    for (int i = 0; i < paths.getSize(); ++i) {
        CableInfo cableInfo;

        GeometryPath* geomPath = paths[i];
        const PathWrapSet& wrapSet = geomPath->getWrapSet();
        const PathPointSet& viaSet = geomPath->getPathPointSet();
        Array<AbstractPathPoint*> activePathPoints = 
                geomPath->getCurrentPath(si);

        AbstractPathPoint* orgPoint = &viaSet[0];
        AbstractPathPoint* insPoint = &viaSet[viaSet.getSize()-1];

        cableInfo.orgBodyName = orgPoint->getParentFrame().getName();
        cableInfo.orgLoc = orgPoint->getLocation(si);
        cableInfo.insBodyName =  insPoint->getParentFrame().getName();
        cableInfo.insLoc = insPoint->getLocation(si);

        int numVias = 0, numSurfs = 0;
        for (int j = 0; j < activePathPoints.getSize(); ++j) {
            AbstractPathPoint* pp = activePathPoints[j];
            cout << "pp" << j << " = " << pp->getName() << " @ "
                 << pp->getParentFrame().getName() << ", loc = "
                 << pp->getLocation(si) << endl;
        }

        for (int j = 0; j < viaSet.getSize(); ++j) {
            AbstractPathPoint* pp = &viaSet[j];
            cout << "mp" << j << " = " << pp->getName() << " @ "
                 << pp->getParentFrame().getName() << ", loc = "
                 << pp->getLocation(si) << endl;
        }

        // add vias to cableInfo
        // skip first (origin) and last (insertion) points
        for (int j = 1; j < viaSet.getSize()-1; ++j) { 
            const AbstractPathPoint& pp = viaSet[j];
            ObstacleInfo obs;
            obs.bodyName = pp.getParentFrame().getName();
            obs.X_BS.setP(pp.getLocation(si));
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
            const WrapObject* wrapObj = wrap.getWrapObject();
            ObstacleInfo obs;
            obs.wrapObjectPtr = wrapObj;
            obs.bodyName = wrapObj->getFrame().getName();
            obs.X_BS = wrapObj->getTransform();
            obs.isVia = false;
            // initially assume inactive
            obs.isActive = false;
            obs.P_S.setToNaN();
            obs.Q_S.setToNaN();
            wrapObs.append(&obs);

            wrapObjectsInUse.insert(wrapObj->getName());
            numSurfs++;
        }

        // find active wrap surfaces and insert to obstacles list in order
        int obsIdx = 0;
        int viaPtIdx = 1; // skip first (origin) point
        // skip first (origin) and last (insertion)
        for (int j = 1; j < activePathPoints.getSize()-1; ++j) { 
            AbstractPathPoint* pp = activePathPoints[j];
            if (*pp == viaSet[viaPtIdx]) {
                viaPtIdx++;
                obsIdx++;
                continue;
            }
            else { // next two path points should be a wrap point
                for (int k = 0; k < wrapSet.getSize(); ++k) {
                    const Vec3& wrapStartPointLoc = wrapSet[k].getPreviousWrap().r1;
                    if (!wrapStartPointLoc.isInf() && 
                            pp->getLocation(si).isNumericallyEqual(wrapStartPointLoc)) {
                        ObstacleInfo* obs = wrapObs[k];
                        obs->isActive = true;
                        // pp and next pp are wrap points
                        Transform X_SB = obs->X_BS.invert();
                        // increment to next pp
                        AbstractPathPoint* pp_next = activePathPoints[++i]; 
                        obs->P_S = X_SB*pp->getLocation(si);
                        obs->Q_S = X_SB*pp_next->getLocation(si);
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

        cout << pathNames[i] << " path, num pts " << activePathPoints.getSize()
             << ", num vias = " << numVias << ", num surfs = " << numSurfs << endl;


        // debugging

        cout << "org" << " = " << orgPoint->getName() << " @ " << 
                orgPoint->getParentFrame().getName() << ", loc = " 
             << orgPoint->getLocation(si) << endl;
        cout << "ins" << " = " << insPoint->getName() << " @ " <<
                insPoint->getParentFrame().getName() << ", loc = " 
             << insPoint->getLocation(si) << endl;

        for (int j = 0; j < cableInfo.obstacles.getSize(); ++j) {
            ObstacleInfo oi = cableInfo.obstacles[j];
            cout << "ObsInfo " << j << ": via=" << oi.isVia << ", bodyName=" 
                 << oi.bodyName << ", loc=" << oi.X_BS.p() << ", P=" 
                 << oi.P_S << ", Q=" << oi.Q_S << endl;
        }


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


    cout << "num forces before removal = " 
         << osimModel.getForceSet().getSize() << endl;

    // remove all OpenSim forces
    osimModel.updForceSet().clearAndDestroy();

    // remove all unused wrap objects
    cout << "wraps in use:" << endl;
    set<String>::iterator it;
    for ( it=wrapObjectsInUse.begin() ; it != wrapObjectsInUse.end(); it++ ) {
        cout << "    " << *it << endl;
    }

    for (int i = 0; i < osimModel.getBodySet().getSize(); ++i) {
        Array<WrapObject*> toRemove;
        const WrapObjectSet& wraps = osimModel.getBodySet()[i].getWrapObjectSet();
        // XXX hack to remove wrap objects, which is not supported in the API
        WrapObjectSet *mutableWraps = const_cast<WrapObjectSet *>(&wraps);
        cout << "culling wrap objs from " << osimModel.getBodySet()[i].getName() 
             << ", num wraps = " << wraps.getSize() << endl;
        //        mutableWraps->clearAndDestroy();
        mutableWraps->setMemoryOwner(true);
        for (int j = 0; j < wraps.getSize(); ++j) {
            // does not contain
            if (wrapObjectsInUse.find(
                        wraps[j].getName()) == wrapObjectsInUse.end()) { 
                toRemove.append(&wraps[j]);
            }
        }

        cout << "wraps to remove:" << endl;
        for (int j = 0; j < toRemove.getSize(); ++j) {
            cout << "    " << toRemove[j]->getName() << endl;
        }

        //        cout << "indices to remove = " << toRemove << endl;
        for (int j = 0; j < toRemove.getSize(); ++j) {
            mutableWraps->remove(toRemove[j]);
        }
        cout << "finished cull from " << osimModel.getBodySet()[i].getName() 
             << ", num wraps = " << wraps.getSize() << endl;

    }

    //    cout << "counting wraps in testWrapping..." << endl;
    //    for (int i = 0; i < osimModel.getBodySet().getSize(); i++) {
    //           const OpenSim::Body& body = osimModel.getBodySet()[i];
    //           const WrapObjectSet& wrapObjects = body.getWrapObjectSet();
    //           cout << body.getName() << ": wrap count = " 
    //                << wrapObjects.getSize() << endl;
    //    }

    // Show model visualizer
    osimModel.setUseVisualizer(true);

    // Initialize the system and get the state representing the state system
    //    SimTK::State& s = osimModel.initSystem();
    osimModel.buildSystem();

    const ModelVisualizer& modelViz = osimModel.getVisualizer();
    const Visualizer& viz = modelViz.getSimbodyVisualizer();
    viz.setBackgroundColor(Vec3(1,1,1));

    cout << "num forces after removal = " 
         << osimModel.getForceSet().getSize() << endl;


    MultibodySystem& system = osimModel.updMultibodySystem();
    GeneralForceSubsystem& forceSubsystem = osimModel.updForceSubsystem();
    // const SimbodyMatterSubsystem& matter = osimModel.getMatterSubsystem();
    // const SimbodyEngine& engine = osimModel.getSimbodyEngine();

    // add cable system
    CableTrackerSubsystem cables(system);

    for (int i = 0; i < cableInfos.getSize(); ++i) {
        const CableInfo& cableInfo = cableInfos[i];

        const OpenSim::Body& orgBody = osimModel.getBodySet().get(
                cableInfo.orgBodyName);
        const OpenSim::Body& insBody = osimModel.getBodySet().get(
                cableInfo.insBodyName);
        const MobilizedBody& orgMobBody = orgBody.getMobilizedBody();
        const MobilizedBody& insMobBody = insBody.getMobilizedBody();
        //        cout << "origin mob idx = " << orgBody.getIndex() << endl;
        //        cout << "insertion mob idx = " << insBody.getIndex() << endl;


        CablePath path(cables, orgMobBody, cableInfo.orgLoc,   // origin
                insMobBody, cableInfo.insLoc);  // termination

        // Add obstacles
        for (int j = 0; j < cableInfo.obstacles.getSize(); ++j) {
            ObstacleInfo oi = cableInfo.obstacles[j];
            const OpenSim::Body& osBody = osimModel.getBodySet().get(
                    oi.bodyName);
            const MobilizedBody& mobBody = osBody.getMobilizedBody();

            if (oi.isVia) {
                CableObstacle::ViaPoint via(path, mobBody, oi.X_BS.p());
            }
            else {
                const WrapSphere* wrapSphere = 
                        dynamic_cast<const WrapSphere*>(oi.wrapObjectPtr);
                if (wrapSphere != 0) {
                    CableObstacle::Surface surf(path, mobBody,
                            oi.X_BS, SimTK::ContactGeometry::Sphere(
                                             wrapSphere->getRadius())); // along y
                    if (oi.isActive)
                        surf.setContactPointHints(oi.P_S, oi.Q_S);
                    else
                        surf.setDisabledByDefault(true);
                    continue;
                }

                const WrapCylinder* wrapCyl = 
                        dynamic_cast<const WrapCylinder*>(oi.wrapObjectPtr);
                if (wrapCyl != 0) {
                    CableObstacle::Surface surf(path, mobBody,
                            oi.X_BS, SimTK::ContactGeometry::Cylinder(
                                             wrapCyl->get_radius())); // along y
                    if (oi.isActive)
                        surf.setContactPointHints(oi.P_S, oi.Q_S);
                    else
                        surf.setDisabledByDefault(true);
                    continue;
                }

                const WrapEllipsoid* wrapEllip = 
                        dynamic_cast<const WrapEllipsoid*>(oi.wrapObjectPtr);
                if (wrapEllip != 0) {
                    CableObstacle::Surface surf(path, mobBody, oi.X_BS, 
                            SimTK::ContactGeometry::Ellipsoid(
                                    wrapEllip->getRadii())); // along y
                    if (oi.isActive)
                        surf.setContactPointHints(oi.P_S, oi.Q_S);
                    else
                        surf.setDisabledByDefault(true);
                    continue;
                }

                const WrapTorus* wrapTorus = 
                        dynamic_cast<const WrapTorus*>(oi.wrapObjectPtr);
                if (wrapTorus != 0) {
                    Real tubeRadius = 
                            (wrapTorus->getOuterRadius()-wrapTorus->getInnerRadius())/2.0;
                    Real torusRadius = wrapTorus->getOuterRadius()-tubeRadius;
                    CableObstacle::Surface surf(path, mobBody,
                            oi.X_BS, 
                            SimTK::ContactGeometry::Torus(torusRadius, tubeRadius)); // along y
                    if (oi.isActive)
                        surf.setContactPointHints(oi.P_S, oi.Q_S);
                    else
                        surf.setDisabledByDefault(true);
                    continue;
                }

                cout << "unknown wrap object [" 
                     << oi.wrapObjectPtr->getName() 
                     << "], adding via point at origin" << endl;
                CableObstacle::ViaPoint via(path, mobBody, oi.X_BS.p());

            }
        }

        MyCableSpring cable(forceSubsystem, path, 100., 3.5, 0*0.1);
        //    system.addEventReporter(new ShowStuff(system, cable, 0.1*0.1));

    }

    system.addEventReporter(new ShowStuff(system, 0.1*0.1));

    SimTK::State& state = osimModel.initializeState();
    viz.report(state);
    for (int i = 0; i < cables.getNumCablePaths(); ++i) {
        const CablePath& path = cables.getCablePath((CablePathIndex)i);
        cout << "path " << i << " length = " 
             << path.getCableLength(state) << endl;
    }

    //    cout << "Hit ENTER ...";
    //    cout.flush();
    //    getchar();

    // Simulate it.
    saveStates.clear(); saveStates.reserve(2000);

    ShowStuff::showHeading(cout);

    simulate(osimModel, state, initialTime, finalTime);

}