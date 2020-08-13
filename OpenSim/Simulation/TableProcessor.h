#ifndef OPENSIM_TABLEPROCESSOR_H
#define OPENSIM_TABLEPROCESSOR_H
/* -------------------------------------------------------------------------- *
 * OpenSim: TableProcessor.h                                                  *
 * -------------------------------------------------------------------------- *
 * Copyright (c) 2019 Stanford University and the Authors                     *
 *                                                                            *
 * Author(s): Christopher Dembia, Nicholas Bianco, Prasanna Sritharan         *
 *                                                                            *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may    *
 * not use this file except in compliance with the License. You may obtain a  *
 * copy of the License at http://www.apache.org/licenses/LICENSE-2.0          *
 *                                                                            *
 * Unless required by applicable law or agreed to in writing, software        *
 * distributed under the License is distributed on an "AS IS" BASIS,          *
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   *
 * See the License for the specific language governing permissions and        *
 * limitations under the License.                                             *
 * -------------------------------------------------------------------------- */

#include "SimulationUtilities.h"
#include <algorithm>

#include <OpenSim/Common/TableUtilities.h>
#include <OpenSim/Common/TimeSeriesTable.h>
#include <OpenSim/Simulation/Model/Model.h>

namespace OpenSim {

/** This abstract class describes *any* operation that consumes and modifies a
TimeSeriesTable as part of a TableProcessor. */
class OSIMSIMULATION_API TableOperator : public Object {
    OpenSim_DECLARE_ABSTRACT_OBJECT(TableOperator, Object);

public:
    /** This function may or may not be provided with a model. If the operation
    requires a model and model == nullptr, an exception is thrown. */
    virtual void operate(TimeSeriesTable& table, const Model* model) const = 0;
};

/** This class describes a workflow for processing a table using
TableOperator%s. The user must provide a source table either as a filepath
to a table or an in-memory TimeSeriesTable. In C++, one can easily chain
together the operators in a processor using the C++ pipe operator:
@code
TableProcessor proc = TableProcessor("file.sto") | TabOpLowPassFilter(6);
@endcode */
class OSIMSIMULATION_API TableProcessor : public Object {
    OpenSim_DECLARE_CONCRETE_OBJECT(TableProcessor, Object);

public:
    OpenSim_DECLARE_PROPERTY(
            filepath, std::string, "File path to a TimeSeriesTable.");
    OpenSim_DECLARE_LIST_PROPERTY(operators, TableOperator,
            "Operators to apply to the source table of this processor.");
    /** This constructor is only for use when reading (deserializing) from an
    XML file. */
    TableProcessor() {
        constructProperty_filepath("");
        constructProperty_operators();
    }
    /** Use an in-memory TimeSeriesTable as the source table.
    Since this constructor is not explicit, you can provide a
    TimeSeriesTable to any function that takes a TableProcessor (in C++). */
    TableProcessor(TimeSeriesTable table) : TableProcessor() {
        m_tableProvided = true;
        m_table = std::move(table);
    }
    /** Use a filepath as the source table.
    Since this constructor is not explicit, you can provide a string
    filepath to any function that takes a TableProcessor. */
    TableProcessor(std::string filepath) : TableProcessor() {
        set_filepath(std::move(filepath));
    }
    /** Process and obtain the table. If a filepath is provided, it will be
    evaluated relative to `relativeToDirectory`.
    Certain TableOperator%s require a Model (e.g.,
    TabOpConvertDegreesToRadians, TabOpUseAbsoluteStateNames). If this processor
    contains such an operator, then the operator will throw an exception
    if you do not provide a model when invoking this function. */
    TimeSeriesTable process(std::string relativeToDirectory,
            const Model* model = nullptr) const {
        OPENSIM_THROW_IF_FRMOBJ(get_filepath().empty() && !m_tableProvided,
                Exception, "No source table.");
        OPENSIM_THROW_IF_FRMOBJ(!get_filepath().empty() && m_tableProvided,
                Exception,
                "Expected either an in-memory table or a filepath, but "
                "both were provided.");
        TimeSeriesTable table;
        if (m_tableProvided) {
            table = m_table;
        } else {
            std::string path = get_filepath();
            if (!relativeToDirectory.empty()) {
                using SimTK::Pathname;
                path = Pathname::
                        getAbsolutePathnameUsingSpecifiedWorkingDirectory(
                                relativeToDirectory, path);
            }
            table = TimeSeriesTable(path);
        }

        for (int i = 0; i < getProperty_operators().size(); ++i) {
            get_operators(i).operate(table, model);
        }
        return table;
    }
    /** Same as above, but paths are evaluated with respect to the current
    working directory. */
    TimeSeriesTable process(const Model* model = nullptr) const {
        return process({}, model);
    }
    /** Same as process(), but the columns of processed table are converted from
    degrees to radians, if applicable. This conversion requires a model. */
    TimeSeriesTable processAndConvertToRadians(std::string relativeToDirectory,
            const Model& model) const {
        TimeSeriesTable table = process(relativeToDirectory, &model);
        if (TableUtilities::isInDegrees(table)) {
            OPENSIM_THROW_IF(
                    !model.hasSystem(), ModelHasNoSystem, model.getName());
            model.getSimbodyEngine().convertDegreesToRadians(table);
        }
        return table;
    }
    /** Same as above, but paths are evaluated with respect to the current
    working directory. */
    TimeSeriesTable processAndConvertToRadians(const Model& model) const {
        return processAndConvertToRadians({}, model);
    }
    /** Returns true if neither a filepath nor an in-memory table have been
    provided. */
    bool empty() const {
        return !m_tableProvided && get_filepath().empty();
    }
    /** Append an operation to the end of the operations in this processor. */
    TableProcessor& append(const TableOperator& op) {
        append_operators(op);
        return *this;
    }
    /** Append all operations in another processor to this processor.
    The source table of the provided trajectory is ignored. */
    TableProcessor& append(const TableProcessor& traj) {
        for (int i = 0; i < traj.getProperty_operators().size(); ++i) {
            append_operators(traj.get_operators(i));
        }
        return *this;
    }
    /** This operator allows one to write the following code in C++:
    @code
    TableProcessor proc = TableProcessor("file.sto") |
            TabOpLowPassFilter(6);
    @endcode */
    TableProcessor& operator|(const TableOperator& right) {
        return append(right);
    }

private:
    bool m_tableProvided = false;
    TimeSeriesTable m_table;
};

class OSIMSIMULATION_API TabOpConvertDegreesToRadians : public TableOperator {
    OpenSim_DECLARE_CONCRETE_OBJECT(TabOpConvertDegreesToRadians,
            TableOperator);
    void operate(TimeSeriesTable& table,
            const Model* model = nullptr) const override {
        OPENSIM_THROW_IF(!model, Exception,
                "Expected a model, but no model was provided.");
        if (TableUtilities::isInDegrees(table)) {
            OPENSIM_THROW_IF(
                    !model->hasSystem(), ModelHasNoSystem, model->getName());
            model->getSimbodyEngine().convertDegreesToRadians(table);
        }
    }
};

/// Apply a low-pass filter to the trajectory.
class OSIMSIMULATION_API TabOpLowPassFilter : public TableOperator {
    OpenSim_DECLARE_CONCRETE_OBJECT(TabOpLowPassFilter, TableOperator);

public:
    OpenSim_DECLARE_PROPERTY(cutoff_frequency, double,
            "Low-pass cutoff frequency (Hz) (default is -1, which means no "
            "filtering).");
    TabOpLowPassFilter() { constructProperty_cutoff_frequency(-1); }
    TabOpLowPassFilter(double cutoffFrequency) : TabOpLowPassFilter() {
        set_cutoff_frequency(cutoffFrequency);
    }
    void operate(TimeSeriesTable& table, const Model* model = nullptr)
            const override {
        if (get_cutoff_frequency() != -1) {
            OPENSIM_THROW_IF(get_cutoff_frequency() <= 0, Exception,
                    "Expected cutoff frequency to be positive, but got {}.",
                    get_cutoff_frequency());

            TableUtilities::filterLowpass(
                    table, get_cutoff_frequency(), true);
        }
    }
};

/** Update table column labels to use post-4.0 state paths instead of pre-4.0
state names. For example, this converts column labels as follows:
  - `pelvis_tilt` -> `/jointset/ground_pelvis/pelvis_tilt/value`
  - `pelvis_tilt_u` -> `/jointset/ground_pelvis/pelvis_tilt/speed`
  - `soleus.activation` -> `/forceset/soleus/activation`
  - `soleus.fiber_length` -> `/forceset/soleus/fiber_length`
This can also be used to convert an Inverse Kinematics Tool solution MOT
file to be used as a states file (with only coordinate values).
If a column label does not identify a state in the model,
the column label is not changed. Column labels must be unique.
This operator is implemented using updateStateLabels40(). */
class OSIMSIMULATION_API TabOpUseAbsoluteStateNames : public TableOperator {
    OpenSim_DECLARE_CONCRETE_OBJECT(TabOpUseAbsoluteStateNames, TableOperator);

public:
    TabOpUseAbsoluteStateNames() {}

    void operate(TimeSeriesTable& table, const Model* model) const override {

        OPENSIM_THROW_IF(!model, Exception,
                "Expected a model, but no model was provided.");

        auto labels = table.getColumnLabels();
        updateStateLabels40(*model, labels);
        table.setColumnLabels(labels);
    }
};

} // namespace OpenSim

#endif // OPENSIM_TABLEPROCESSOR_H
