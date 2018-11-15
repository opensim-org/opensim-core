#include <OpenSim/OpenSim.h>

using namespace OpenSim;
using namespace SimTK;

class Sugar : public ModelComponent {
    OpenSim_DECLARE_CONCRETE_OBJECT(Sugar, ModelComponent);
public:
    OpenSim_DECLARE_OUTPUT(fructose, double, getFructose,
                           SimTK::Stage::Time);
    double getFructose(const SimTK::State& s) const { return 5.3; }
};

template <typename T>
class DataSource_ : public ModelComponent {
    OpenSim_DECLARE_CONCRETE_OBJECT_T(DataSource_, T, ModelComponent);
public:
    OpenSim_DECLARE_LIST_OUTPUT(col, T, getColumnAtTime,
                                SimTK::Stage::Time);
// OpenSim_DECLARE_OUTPUT(all, Vector<T>, getRow, SimTK::Stage::Instance);
    
    T getColumnAtTime(const SimTK::State& s, const std::string& label) const {
        return interpolate(s.getTime(), label);
    }
    
    T interpolate(const double& time, const std::string& label) const {
        const auto& colIndex = _table.getColumnIndex(label);
        const auto& times(_table.getIndependentColumn());
        
        // Get the first time greater or equal to the requested time.
        const auto& lowerb = std::lower_bound(times.begin(), times.end(), time);
        const auto& timeLowerb = *lowerb;
        const auto& ilowerb = lowerb - times.begin();
        // If the time is an exact match to an existing column.
        if (timeLowerb == time) {
            const auto& row = _table.getRowAtIndex(ilowerb);
            return row[colIndex];
        }
        
        // Get the latest time that is less than the requested time.
        const auto& below = lowerb - 1;
        const auto& timeBelow = (*below);
        const auto& ibelow = below - times.begin();
        
        // If we got this far, then lowerb is the first time greater than
        // the requested time.
        const auto& iabove = ilowerb;
        const auto& timeAbove = timeLowerb;
        
        // Compute fraction within the interval.
        const double numer = time - timeBelow;
        const double denom = timeAbove - timeBelow;
        const double fraction = denom < SimTK::Eps ? 0.0 : numer / denom;
        
        // Get the rows at the below and above times.
        const auto& rowBelow = _table.getRowAtIndex(ibelow);
        const auto& rowAbove = _table.getRowAtIndex(iabove);
        
        const T& delta = rowAbove[colIndex] - rowBelow[colIndex];
        return rowBelow[colIndex] + fraction * delta;
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

void testOutputVectorsAndChannels() {
    Model model;
    model.setName("ironman");
    
    // DataSource
    // ----------
    auto* src = new DataSource();
    src->setName("exp_data");
    model.addModelComponent(src);
    
    // Fill up the DataSource.
    auto& table = src->updTable();
    
    // Column labels.
    std::vector<std::string> columnLabels;
    for (unsigned i = 0; i < 9; ++i) {
        columnLabels.push_back("col" + std::to_string(i));
    }
    table.setColumnLabels(columnLabels);
    
    TimeSeriesTable::RowVector row(9, 0.0);
    row[1] = 1; row[2] = 2; row[3] = 3; row[4] = 4;
    table.appendRow(0.0, row); row += 1;
    table.appendRow(0.25, row); row += 1;
    table.appendRow(0.50, row); row += 1;
    table.appendRow(0.75, row); row += 1;
    table.appendRow(1.0, row); row += 1;
    table.appendRow(1.1, row);
    
    // Reporter
    // --------
    auto* rep = new ConsoleReporter();
    rep->setName("interped");
    model.addComponent(rep);
    
    // A component with a non-list output
    // -----------------------------------
    auto* sugar = new Sugar();
    sugar->setName("calories");
    model.addModelComponent(sugar);
    
    model.finalizeFromProperties();
    
    SimTK::State& s = model.initSystem();
    
    // Must connect *after* initSystem(), since it first clears all
    // existing connections.
    rep->updInput("inputs").connect(src->getOutput("col").getChannel("col0"));
    //rep->updInput("input").connect(src->getOutput("col").getChannel("col1"));
    rep->updInput("inputs").connect(src->getOutput("col"));
    rep->updInput("inputs").connect(sugar->getOutput("fructose"));
    
    RungeKuttaMersonIntegrator integrator(model.getSystem());
    integrator.setFixedStepSize(0.1);
    integrate(model.getSystem(), integrator, s, 1.0);
    
}

int main() {
    // TODO SimTK_START_TEST("futureOutputVectorsAndChannels");
        SimTK_SUBTEST(testOutputVectorsAndChannels);
    //SimTK_END_TEST();
}




