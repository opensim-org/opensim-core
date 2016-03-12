
#include <OpenSim/OpenSim.h>

using namespace OpenSim;
using namespace SimTK;

// TODO Multiplexer, channels of type T into 1 vector. with the same annotations.

// TODO this could be a type of component that has a minimum required number
// of inputs: need 3 markers to define a body.
// TODO however, this component is a little unrealistic, since you need an
// entire trajectory of the markers to compute the trajectory of the joint
// center. What makes more sense is a component that just averages markers.
class JointCenter : public ModelComponent {
    OpenSim_DECLARE_CONCRETE_OBJECT(JointCenter, ModelComponent);
public:
    OpenSim_DECLARE_LIST_INPUT(seg1_markers, Vec3, SimTK::Stage::Time,
        "Markers on segment 1.");
    OpenSim_DECLARE_LIST_INPUT(seg2_markers, Vec3, SimTK::Stage::Time,
        "Markers on segment 2.");
    OpenSim_DECLARE_OUTPUT(joint_center, Vec3, getJointCenter,
                           SimTK::Stage::Time);
    Vec3 getJointCenter(const SimTK::State& s) const {
        const auto& seg1Channels = getInput<Vec3>("seg1_markers").getChannels();
        const auto seg1(getVectorOfMarkers(s, seg1Channels));
        
        const auto& seg2Channels = getInput<Vec3>("seg2_markers").getChannels();
        const auto seg2(getVectorOfMarkers(s, seg2Channels));
        
        // I'm just doing an arbitrary calculation with both vectors.
        return 0.5 * (seg1.sum() + seg2.sum());
    }
private:
    Vector_<Vec3> getVectorOfMarkers(const SimTK::State& s,
            const Input<Vec3>::ChannelList& channels) const {
        // TODO vector outputs would get rid of this.
        // TODO or some way to get all inputs as a vector.
        Vector_<Vec3> vec(channels.size());
        for (int ichan = 0; ichan < channels.size(); ++ichan) {
            vec[ichan] = channels[ichan]->getValue(s);
        }
        return vec;
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
            for (int i = 0; i < rowBelow.size(); ++i) {
                transposed[i] = rowBelow[i];
            }
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
        for (int i = 0; i < result.size(); ++i) {
            transposed[i] = result[i];
        }
        return transposed;
    }
    void getSurroundingIndices(const double& time, int& ibelow, int& iabove,
                               double& fraction) const {
        // TODO I have not tested the interpolation.
        const auto& times = _table.getIndependentColumn();
        
        // Get the first time greater or equal to the requested time.
        const auto& lowerb = std::lower_bound(times.begin(), times.end(), time);
        const auto& timeLowerb = *lowerb;
        const auto& ilowerb = lowerb - times.begin();
        // If the the time is an exact match to an existing column.
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
        "The target (experimental) marker positions.");
    // TODO OpenSim_DECLARE_LIST_INPUT(marker_weights, double, SimTK::Stage::Position,
    // TODO     "Weights for each marker specified in targets.");
    
    Vec3 getModelMarkerPosition(const SimTK::State& s,
                                const std::string& marker) const {
        solve(s); // TODO cache the result in some way.
        return getModel().getMarkerSet().get(marker).findLocationInFrame(s,
                getModel().getGround());
    }
    
    double getSolution(const SimTK::State& s, const std::string& coord) const {
        solve(s); // TODO cache the result in some way.
        // TODO just pretend to do something.
        return getModel().getCoordinateSet().get(coord).getValue(s);
    }
    void solve(const SimTK::State& s) const {
    
    }
protected:
    void extendFinalizeFromProperties() override {
        Super::extendFinalizeFromProperties();
        // TODO or use a model connected to us via a connectee?
        for (const auto& coord : getParent().getComponentList<Coordinate>()) {
            updOutput("coords").addChannel(coord.getName());
        }
        // TODO available channels could depend on what target markers are
        // given to us. Consider a callback "updateChannels()" that is called
        // whenever the inputs are updated, so that we can create new output
        // channels.
        for (const auto& marker : getParent().getComponentList<Marker>()) {
            updOutput("model_marker_pos").addChannel(marker.getName());
        }
    }
private:
};

template <typename T>
class ConsoleReporter_ : public ModelComponent {
    OpenSim_DECLARE_CONCRETE_OBJECT_T(ConsoleReporter_, T, Component);
public:
    ConsoleReporter_() { constructProperty_enabled(true); }
    OpenSim_DECLARE_PROPERTY(enabled, bool, "Should this report results?");
    OpenSim_DECLARE_LIST_INPUT(input, T, SimTK::Stage::Acceleration, "");
private:
    void extendRealizeReport(const State& state) const override {
        if (!get_enabled()) return;
        const auto& input = getInput<T>("input");
        
        // Don't report anything if nothing is connected to us.
        if (!input.isConnected()) return;
        
        if (_printCount % 20 == 0) {
            std::cout << "[" << getName() << "] "
                      << std::setw(_width) << "time" << "| ";
            for (const auto& chan : input.getChannels()) {
                const auto& name = chan->getPathName();
                const auto& truncName = name.size() <= _width ?
                    name : name.substr(name.size() - _width);
                std::cout << std::setw(_width) << truncName << "|";
            }
            std::cout << "\n";
        }
        std::cout << "[" << getName() << "] "
                  << std::setw(_width) << state.getTime() << "| ";
        for (const auto& chan : input.getChannels()) {
            const auto& value = chan->getValue(state);
            const auto& nSigFigs = chan->getOutput().getNumberOfSignificantDigits();
            std::cout << std::setw(_width)
                      << std::setprecision(nSigFigs) << value << "|";
        }
        std::cout << std::endl;
        
        const_cast<ConsoleReporter_<T>*>(this)->_printCount++;
    }
    unsigned int _printCount = 0;
    int _width = 12;
};

typedef ConsoleReporter_<double> ConsoleReporter;
typedef ConsoleReporter_<Vec3> ConsoleReporterVec3;
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
    hog->getCoordinateSet().get(0).setDefaultValue(0.5 * SimTK::Pi);
    
    auto* asis = new OpenSim::Marker();
    asis->setName("asis"); asis->setReferenceFrame(*pelvis);
    model.addMarker(asis);
    
    auto* psis = new OpenSim::Marker();
    psis->setName("psis"); psis->setReferenceFrame(*pelvis);
    model.addMarker(psis);
    
    auto* med_knee = new OpenSim::Marker();
    med_knee->setName("med_knee"); med_knee->setReferenceFrame(*femur);
    model.addMarker(med_knee);
    
    auto* lat_knee = new OpenSim::Marker();
    lat_knee->setName("lat_knee"); lat_knee->setReferenceFrame(*femur);
    model.addMarker(lat_knee);
    
    auto* thigh = new OpenSim::Marker();
    thigh->setName("thigh"); thigh->setReferenceFrame(*femur);
    model.addMarker(thigh);
    
    auto* hjc = new OpenSim::Marker();
    hjc->setName("hjc"); hjc->setReferenceFrame(*femur);
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
    expRep->set_enabled(false);
    model.addModelComponent(expRep);
    
    auto* solution = new ConsoleReporter();
    solution->setName("coords");
    solution->set_enabled(false);
    model.addModelComponent(solution);
    
    auto* modelMarkers = new ConsoleReporterVec3();
    modelMarkers->setName("model_markers");
    modelMarkers->set_enabled(false);
    model.addModelComponent(modelMarkers);
    
    auto* vecRep = new ConsoleReporterVectorVec3();
    vecRep->setName("exp");
    //expRep->set_enabled(false);
    model.addModelComponent(vecRep);
    
    // A component with a non-list output
    // -----------------------------------
    auto* hjc = new JointCenter();
    hjc->setName("hip_joint_center");
    model.addModelComponent(hjc);
    
    SimTK::State& s = model.initSystem();
    
    // Set up input-output connections.
    // --------------------------------
    // Must connect *after* initSystem(), since it first clears all
    // existing connections.
    
    // Connect up the study.
    
    // Compute joint centers.
    hjc->updInput("seg1_markers").connect(exp->getOutput("col").getChannel("asis"));
    hjc->updInput("seg1_markers").connect(exp->getOutput("col").getChannel("psis"));
    hjc->updInput("seg2_markers").connect(exp->getOutput("col").getChannel("med_knee"));
    hjc->updInput("seg2_markers").connect(exp->getOutput("col").getChannel("lat_knee"));
    hjc->updInput("seg2_markers").connect(exp->getOutput("col").getChannel("thigh"));
    
    // Set the marker targets for IK.
    // TODO how to ignore a single element of the vector?
    // TODO need some kind of "reducer" component or the
    // DataSource can list the columns to export.
    ik->updInput("targets").connect(exp->getOutput("col"));
    ik->updInput("targets").connect(hjc->getOutput("joint_center"));
    
    // Connect up the reporters.
    expRep->updInput("input").connect(exp->getOutput("col").getChannel("asis"));
    expRep->updInput("input").connect(exp->getOutput("col").getChannel("psis"));
    expRep->updInput("input").connect(hjc->getOutput("joint_center"));
    vecRep->updInput("input").connect(exp->getOutput("all"));
    
    modelMarkers->updInput("input").connect(ik->getOutput("model_marker_pos"));
    solution->updInput("input").connect(ik->getOutput("coords"));
    
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



