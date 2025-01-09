
#include <OpenSim/OpenSim.h>

using namespace OpenSim;
using namespace SimTK;

// This example only deals with "list outputs" in which each output is a
// single value of type T. The key notion here is that outputs can contain
// any number of channels.

// TODO Multiplexer, channels of type T into 1 vector. with the same aliases.

// TODO this could be a type of component that has a minimum required number
// of inputs: need 3 markers to define a body.
// TODO however, this component is a little unrealistic, since you need an
// entire trajectory of the markers to compute the trajectory of the joint
// center. What makes more sense is a component that just averages markers.
class JointCenter : public ModelComponent {
    OpenSim_DECLARE_CONCRETE_OBJECT(JointCenter, ModelComponent);
public:
    // TODO to a list input, vector and list outputs look the same.
    // TODO so make these into list inputs again.
    OpenSim_DECLARE_LIST_INPUT(seg1_markers, Vec3, SimTK::Stage::Time,
        "Markers on segment 1.");
    OpenSim_DECLARE_LIST_INPUT(seg2_markers, Vec3, SimTK::Stage::Time,
        "Markers on segment 2.");
    OpenSim_DECLARE_OUTPUT(joint_center, Vec3, getJointCenter,
                           SimTK::Stage::Time);
    Vec3 getJointCenter(const SimTK::State& s) const {
        const auto& seg1 = getInput<Vec3>("seg1_markers").getVector(s);
        const auto& seg2 = getInput<Vec3>("seg2_markers").getVector(s);
        
        // I'm just doing an arbitrary calculation with both vectors.
        return 0.5 * (seg1.sum() + seg2.sum());
    }
};

// TODO this should have an internal component for interpolating.
template <typename T>
class DataSource_ : public ModelComponent {
    OpenSim_DECLARE_CONCRETE_OBJECT_T(DataSource_, T, ModelComponent);
public:
    OpenSim_DECLARE_LIST_OUTPUT(col, T, getColumnAtTime,
                                SimTK::Stage::Time);
    OpenSim_DECLARE_OUTPUT(all, SimTK::Vector_<T>, getAllColumnsAtTime,
                                SimTK::Stage::Time);
    
    T getColumnAtTime(const SimTK::State& s, const std::string& label) const {
        // TODO I have not tested the interpolation.
        const auto& colIndex = _table.getColumnIndex(label);
        
        // Get help with the interpolating.
        int ibelow, iabove;
        double fraction;
        getSurroundingIndices(s.getTime(), ibelow, iabove, fraction);
        
        // Get the row at or below the requested time.
        const auto& rowBelow = _table.getRowAtIndex(ibelow);
        
        // This means that requested time is an exact match.
        if (iabove == -1) { return rowBelow[colIndex]; }
        
        // Get the row above the requested time.
        const auto& rowAbove = _table.getRowAtIndex(iabove);
        
        const auto& elementBelow = rowBelow[colIndex];
        const auto& elementAbove = rowAbove[colIndex];
        
        const T& delta = elementAbove - elementBelow;
        return elementBelow + fraction * delta;
    }
    SimTK::Vector_<T> getAllColumnsAtTime(const SimTK::State& s) const {
        // TODO I have not tested the interpolation.
    
        // Get help with the interpolating.
        int ibelow, iabove;
        double fraction;
        getSurroundingIndices(s.getTime(), ibelow, iabove, fraction);
        
        // Get the row at or below the requested time.
        const auto& rowBelow = _table.getRowAtIndex(ibelow);
        
        // This means that requested time is an exact match.
        if (iabove == -1) {
            // TODO this is all probably very inefficient, but I had trouble
            // converting a RowVector_<T> to a Vector_<T>.
            SimTK::Vector_<T> transposed(rowBelow.size());
            for (int i = 0; i < rowBelow.size(); ++i) transposed[i] = rowBelow[i];
            return transposed;
        }
        
        // Get the row above the requested time.
        const auto& rowAbove = _table.getRowAtIndex(iabove);
        
        const auto& delta = rowAbove - rowBelow;
        const auto& result = rowBelow + fraction * delta;
        
        // TODO this is all probably very inefficient, but I had trouble
        // converting a RowVector_<T> to a Vector_<T>.
        SimTK::Vector_<T> transposed(result.size());
        // TODO transposed.copyAssign(result.transpose());
        for (int i = 0; i < result.size(); ++i) transposed[i] = result[i];
        return transposed;
    }
    std::vector<std::string> getColumnLabels() const {
        return _table.getColumnLabels();
    }
    TimeSeriesTable_<T>& updTable() { return _table; }
    const TimeSeriesTable_<T> getTable() const { return _table; }
    
protected:
    void extendFinalizeFromProperties() override {
        Super::extendFinalizeFromProperties();
        const auto& keys = _table.getDependentsMetaData().getKeys();
        // TODO tables should be initialized with a labels metadata.
        updOutput("col").clearChannels();
        if (std::find(keys.begin(), keys.end(), "labels") != keys.end()) {
            for (const auto& label : _table.getColumnLabels()) {
                updOutput("col").addChannel(label);
            }
        }
    }
    void getSurroundingIndices(const double& time, int& ibelow, int& iabove,
                               double& fraction) const {
        // TODO I have not tested the interpolation.
        const auto& times = _table.getIndependentColumn();
        
        // Get the first time greater or equal to the requested time.
        const auto& lowerb = std::lower_bound(times.begin(), times.end(), time);
        const auto& timeLowerb = *lowerb;
        const auto& ilowerb = lowerb - times.begin();
        // If the time is an exact match to an existing column.
        if (timeLowerb == time) {
            ibelow = ilowerb;
            iabove = -1;
            return;
        }
        
        // Get the latest time that is less than the requested time.
        const auto& below = lowerb - 1;
        const auto& timeBelow = (*below);
        ibelow = below - times.begin();
        
        // If we got this far, then lowerb is the first time greater than
        // the requested time.
        iabove = ilowerb;
        const auto& timeAbove = timeLowerb;
        
        // Compute fraction within the interval.
        const double numer = time - timeBelow;
        const double denom = timeAbove - timeBelow;
        fraction = denom < SimTK::Eps ? 0.0 : numer / denom;
    }
private:
    TimeSeriesTable_<T> _table;
};

typedef DataSource_<double> DataSource;
typedef DataSource_<SimTK::Vec3> DataSourceVec3;

// TODO InverseKinematics shouldn't really produce a coordinates output;
// it's more of a states solver--it should be editing the coordinate values
// in the State. So, this example is really mostly for demonstration.
// TODO It actually would work really well as a task space controller.
class InverseKinematics : public ModelComponent {
    OpenSim_DECLARE_CONCRETE_OBJECT(InverseKinematics, ModelComponent);
public:
    // TODO connector to a model.
    
    // TODO convert these to vectors.
    OpenSim_DECLARE_LIST_OUTPUT(model_marker_pos, Vec3, getModelMarkerPosition,
        SimTK::Stage::Position);
    OpenSim_DECLARE_LIST_OUTPUT(coords, double, getSolution,
        SimTK::Stage::Position);
    OpenSim_DECLARE_LIST_INPUT(targets, Vec3, SimTK::Stage::Position,
        "The target (experimental) marker positions. Input aliases must "
        "be the name of the model marker to pair with each target.");
    // TODO OpenSim_DECLARE_LIST_INPUT(marker_weights, double, SimTK::Stage::Position,
    // TODO     "Weights for each marker specified in targets.");
    
    Vec3 getModelMarkerPosition(const SimTK::State& s, const std::string& name) const {
        solve(s); // TODO cache the result in some way.
        const auto& markerSet = getModel().getMarkerSet();
        const auto& ground = getModel().getGround();
        return markerSet.get(name).findLocationInFrame(s, ground);
    }
    
    double getSolution(const SimTK::State& s, const std::string& coord) const {
        solve(s); // TODO cache the result in some way.
        // Just pretend to do something.
        return getModel().getCoordinateSet().get(coord).getValue(s);
    }
    
    void solve(const SimTK::State& s) const {
        // These target markers could have come from mixed sources.
        const auto& targets = getInput<Vec3>("targets").getVector(s);
        // To pretend like we're doing something useful, we'll multiply by
        // the station Jacobian transpose.
        // TODO
        const auto& smss = getModel().getMatterSubsystem();
        SimTK::Vector f;
        smss.multiplyByStationJacobianTranspose(s, _onBodyB, _stationPinB,
                                                targets, f);
        
    }
    void extendRealizeInstance(const SimTK::State& s) const override {
        Super::extendRealizeInstance(s);
        // TODO I need to do some initialization *after* the connections are set,
        // because the channels connected to the input "targets" tell me what
        // I need to cache.
        // Logically, the connections are serialized, so I *should* also be able
        // to do this in finalizeFromProperties(); it'd be neat if we could make
        // that work.
        const_cast<Self*>(this)->_onBodyB.clear();
        const_cast<Self*>(this)->_stationPinB.clear();
        const auto& input = getInput<Vec3>("targets");
        for (int ichan = 0; ichan < input.getNumConnectees(); ++ichan) {
            const auto& markerName = input.getAlias(ichan);
            const auto& marker = getModel().getMarkerSet().get(markerName);
            const auto& mbi = marker.getParentFrame().getMobilizedBodyIndex();
            const_cast<Self*>(this)->_onBodyB.push_back(mbi);
            const_cast<Self*>(this)->_stationPinB.push_back(marker.get_location());
        }
    }
protected:
    void extendFinalizeFromProperties() override {
        // Create the output channels.
        for (const auto& marker : getOwner().getComponentList<Marker>()) {
            updOutput("model_marker_pos").addChannel(marker.getName());
        }
        for (const auto& coord : getOwner().getComponentList<Coordinate>()) {
            updOutput("coords").addChannel(coord.getName());
        }
    }
private:
    // These are arguments to multiplybyStationJacobianTranspose() and describe
    // the stations that the targets correspond to.
    // TODO we could also achieve this maybe using the Marker (Station) component?
    SimTK::Array_<SimTK::MobilizedBodyIndex> _onBodyB;
    SimTK::Array_<SimTK::Vec3> _stationPinB;
};

typedef ConsoleReporter_<Vector_<Vec3>> ConsoleReporterVectorVec3;

void integrate(const System& system, Integrator& integrator,
        const State& initialState,
        Real finalTime) {
    TimeStepper ts(system, integrator);
    ts.initialize(initialState);
    ts.setReportAllSignificantStates(true);
    integrator.setReturnEveryInternalStep(true); 
    while (ts.getState().getTime() < finalTime) {
        ts.stepTo(finalTime);
        system.realize(ts.getState(), SimTK::Stage::Report);
    }
}

void createModel(Model& model) {
    auto* pelvis = new OpenSim::Body("pelvis", 1.0, Vec3(0), Inertia(1));
    model.addBody(pelvis);
    auto* femur = new OpenSim::Body("femur", 1.0, Vec3(0), Inertia(1));
    model.addBody(femur);
    auto* hog = new PinJoint("hog",
                             model.getGround(), Vec3(0), Vec3(0),
                             *pelvis,           Vec3(0, 1, 0), Vec3(0));
    model.addJoint(hog);
    auto* hip = new PinJoint("hip",
                             *pelvis, Vec3(0), Vec3(0),
                             *femur,  Vec3(0, 1, 0), Vec3(0));
    model.addJoint(hip);
    hog->updCoordinate().setDefaultValue(0.5 * SimTK::Pi);
    
    auto* asis = new OpenSim::Marker();
    asis->setName("asis"); asis->setParentFrame(*pelvis);
    model.addMarker(asis);
    
    auto* psis = new OpenSim::Marker();
    psis->setName("psis"); psis->setParentFrame(*pelvis);
    model.addMarker(psis);
    
    auto* med_knee = new OpenSim::Marker();
    med_knee->setName("med_knee"); med_knee->setParentFrame(*femur);
    model.addMarker(med_knee);
    
    auto* lat_knee = new OpenSim::Marker();
    lat_knee->setName("lat_knee"); lat_knee->setParentFrame(*femur);
    model.addMarker(lat_knee);
    
    auto* thigh = new OpenSim::Marker();
    thigh->setName("thigh"); thigh->setParentFrame(*femur);
    model.addMarker(thigh);
    
    auto* hjc = new OpenSim::Marker();
    hjc->setName("hjc"); hjc->setParentFrame(*femur);
    model.addMarker(hjc);
}

void fabricateExperimentalMarkers(TimeSeriesTable_<Vec3>& table) {
    table.setColumnLabels({"asis", "psis", "med_knee", "lat_knee", "thigh"});
    
    TimeSeriesTable_<Vec3>::RowVector row(5, Vec3(0.0));
    row[1] = Vec3(0, 0, 0); row[2] = Vec3(1, 0, 0); row[3] = Vec3(2, 0, 0);
    row[4] = Vec3(3, 0, 0);
    table.appendRow(0.0, row); row += Vec3(0, 1, 0);
    table.appendRow(0.25, row); row += Vec3(1, 0, 0);
    table.appendRow(0.50, row); row += Vec3(0, 0, 1);
    table.appendRow(0.75, row); row += Vec3(2, 0, 0);
    table.appendRow(1.0, row); row += Vec3(0, 3, 0);
    table.appendRow(1.1, row);
}

void testFutureIKListOutputs() {

    // Create a model.
    // ---------------
    Model model;
    model.setName("ironman");
    createModel(model);
    // model.setUseVisualizer(true);

    SimTK::State& s0 = model.initSystem();
    
    // DataSource
    // ----------
    auto* exp = new DataSourceVec3();
    exp->setName("exp_markers");
    model.addModelComponent(exp);
    
    // Fill up the DataSource.
    auto& table = exp->updTable();
    fabricateExperimentalMarkers(table);
    
    // Inverse kinematics
    // ------------------
    auto* ik = new InverseKinematics();
    ik->setName("ik");
    model.addModelComponent(ik);
    
    // Reporters
    // ---------
    auto* expRep = new ConsoleReporterVec3();
    expRep->setName("exp_computed");
    //expRep->set_enabled(false);
    model.addComponent(expRep);
    
    auto* solution = new ConsoleReporter();
    solution->setName("coords");
    //solution->set_enabled(false);
    model.addComponent(solution);
    
    auto* modelMarkers = new ConsoleReporterVec3();
    modelMarkers->setName("model_markers");
    //modelMarkers->set_enabled(false);
    model.addComponent(modelMarkers);
    
    auto* vecRep = new ConsoleReporterVectorVec3();
    vecRep->setName("exp");
    //expRep->set_enabled(false);
    model.addComponent(vecRep);
    
    // A component with a non-list output
    // -----------------------------------
    auto* hjc0 = new JointCenter();
    auto* hjc = hjc0->clone();
    hjc->setName("hip_joint_center");
    model.addModelComponent(hjc);
    
    // Set up input-output connections.
    // --------------------------------
    // Must connect *after* initSystem(), since it first clears all
    // existing connections.
    
    // Connect up the study.
    
    // Compute joint centers.
    // TODO this would more naturally be a vector input.
    hjc->updInput("seg1_markers").connect(exp->getOutput("col").getChannel("asis"));
    hjc->updInput("seg1_markers").connect(exp->getOutput("col").getChannel("psis"));
    hjc->updInput("seg2_markers").connect(exp->getOutput("col").getChannel("med_knee"));
    hjc->updInput("seg2_markers").connect(exp->getOutput("col").getChannel("lat_knee"));
    hjc->updInput("seg2_markers").connect(exp->getOutput("col").getChannel("thigh"));
    
    // Set the marker targets for IK. Connect to all channels in the "col"
    // list output.
    ik->updInput("targets").connect(exp->getOutput("col"));
    // Must provide an alias for this output since we need a way to figure out
    // which marker it corresponds to. The InverseKinematics component uses
    // aliases to pair target Vec3's with model markers.
    ik->updInput("targets").connect(hjc->getOutput("joint_center"), "hjc");
    
    // Connect up the reporters.
    expRep->updInput("inputs").connect(exp->getOutput("col").getChannel("asis"));
    expRep->updInput("inputs").connect(exp->getOutput("col").getChannel("psis"));
    expRep->updInput("inputs").connect(hjc->getOutput("joint_center"));
    // This outputs a Vector_<Vec3>.
    vecRep->updInput("inputs").connect(exp->getOutput("all"));
    
    modelMarkers->updInput("inputs").connect(ik->getOutput("model_marker_pos"));
    // Connect to all channels in the "coords" list output.
    solution->updInput("inputs").connect(ik->getOutput("coords"));
    hjc->printSocketInfo();
    ik->printSocketInfo();

    model.printSubcomponentInfo();

    SimTK::State& s = model.initSystem();
    
    // TODO replace with a driver / time step advancer.
    RungeKuttaMersonIntegrator integrator(model.getSystem());
    integrator.setFixedStepSize(0.1);
    integrate(model.getSystem(), integrator, s, 1.0);
}

int main() {
    // TODO SimTK_START_TEST("futureIKListOutputs");
        SimTK_SUBTEST(testFutureIKListOutputs);
    //SimTK_END_TEST();
}



