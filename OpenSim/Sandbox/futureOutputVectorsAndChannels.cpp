#include <OpenSim/OpenSim.h>

using namespace OpenSim;
using namespace SimTK;

template <typename T>
class DataSource_ : public ModelComponent {
    OpenSim_DECLARE_CONCRETE_OBJECT_T(DataSource_, T, ModelComponent);
    OpenSim_DECLARE_LIST_OUTPUT(columns, T, getColumnAtTime,
                                SimTK::Stage::Instance);
// OpenSim_DECLARE_OUTPUT(all, Vector<T>, getRow, SimTK::Stage::Instance);
public:
    
    T getColumnAtTime(const SimTK::State& s, const std::string& label) const {
        // TODO change "col1" to label.
        return interpolate(s.getTime(), "col1");
    }
    
    T interpolate(const double& time, const std::string& label) const {
        const auto& colIndex = _table.getColumnIndex(label);
        const auto& times(_table.getIndependentColumn());
        
        // Get the first time greater or equal to the requested time.
        const auto& lowerb = std::lower_bound(times.begin(), times.end(), time);
        const auto& timeLowerb = *lowerb;
        const auto& ilowerb = lowerb - times.begin();
        // If the the time is an exact match to an existing column.
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
        if (std::find(keys.begin(), keys.end(), "labels") != keys.end()) {
            for (const auto& label : _table.getColumnLabels()) {
                // TODO getOutput("columns").addChannel(label);
            }
        }
    }
    
private:
    
    TimeSeriesTable_<T> _table;
};

typedef DataSource_<double> DataSource;

template <typename T>
class ConsoleReporter_ : public ModelComponent {
    OpenSim_DECLARE_CONCRETE_OBJECT_T(ConsoleReporter_, T, Component);
public:
    OpenSim_DECLARE_LIST_INPUT(input, T, SimTK::Stage::Acceleration, "");
private:
    void extendRealizeReport(const State& state) const override {
        const auto& input = getInput<T>("input");
        
        if (_printCount % 20 == 0) {
            std::cout << "[" << getName() << "] "
                      << std::setw(_width) << "time" << "| ";
            for (const auto& output : input.getOutputs()) {
                const auto& outName = output->getName();
                const auto& truncName = outName.size() <= _width ?
                    outName : outName.substr(outName.size() - _width);
                std::cout << std::setw(_width) << truncName << "|";
            }
            std::cout << "\n";
        }
        std::cout << "[" << getName() << "] "
                  << std::setw(_width) << state.getTime() << "| ";
        for (const auto& output : input.getOutputs()) {
            const auto& value = output->getValue(state);
            const auto& nSigFigs = output->getNumberOfSignificantDigits();
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
    
    // DataSource
    // ----------
    auto* src = new DataSource();
    model.addModelComponent(src);
    
    // Fill up the DataSource.
    auto& table = src->updTable();
    
    // Column labels.
    std::vector<std::string> columnLabels;
    for (unsigned i = 0; i < 5; ++i) {
        columnLabels.push_back("col" + std::to_string(i));
    }
    table.setColumnLabels(columnLabels);
    
    TimeSeriesTable::RowVector row(5, 0.0);
    row[1] = 1; row[2] = 2; row[3] = 3; row[4] = 4;
    table.appendRow(0.0, row);
    row += 1;
    table.appendRow(0.25, row);
    row += 1;
    table.appendRow(0.50, row);
    row += 1;
    table.appendRow(0.75, row);
    row += 1;
    table.appendRow(1.0, row);
    row += 1;
    table.appendRow(1.1, row);
    
    // Reporter
    // --------
    auto* rep = new ConsoleReporter();
    rep->setName("interped");
    model.addModelComponent(rep);
    
    //rep->updInput("input").connect(src->getOutput("columns").getChannel("col1")));
    rep->updInput("input").connect(src->getOutput("columns"));
    
    SimTK::State& s = model.initSystem();
    RungeKuttaMersonIntegrator integrator(model.getSystem());
    integrator.setFixedStepSize(0.1);
    integrate(model.getSystem(), integrator, s, 1.0);
    
}

int main() {
    // TODO SimTK_START_TEST("futureOutputVectorsAndChannels");
        SimTK_SUBTEST(testOutputVectorsAndChannels);
    //SimTK_END_TEST();
}




