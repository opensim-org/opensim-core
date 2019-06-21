#ifndef MOCO_TABLEPROCESSOR_H
#define MOCO_TABLEPROCESSOR_H
/* -------------------------------------------------------------------------- *
 * OpenSim Moco: TableProcessor.h                                             *
 * -------------------------------------------------------------------------- *
 * Copyright (c) 2019 Stanford University and the Authors                     *
 *                                                                            *
 * Author(s): Christopher Dembia, Nicholas Bianco                             *
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

#include "../MocoUtilities.h"

#include <OpenSim/Common/TimeSeriesTable.h>
#include <OpenSim/Simulation/Model/Model.h>

namespace OpenSim {

/// This abstract class describes *any* operation that consumes a
/// modifies a TimeSeriesTable as part of a TableProcessor.
class OSIMMOCO_API TableOperator : public Object {
    OpenSim_DECLARE_ABSTRACT_OBJECT(TableOperator, Object);

public:
    virtual void operate(TimeSeriesTable& table) const = 0;
};

/// This class describes a workflow for processing a table using
/// TableOperator%s. The user must provide a source table either as a filepath
/// to a table or an in-memory TimeSeriesTable. In C++, one can easily chain
/// together the operators in a processor using the C++ pipe operator:
/// @code
/// TableProcessor proc = TableProcessor("file.sto") | TabOpLowPassFilter(6);
/// @endcode
class OSIMMOCO_API TableProcessor : public Object {
    OpenSim_DECLARE_CONCRETE_OBJECT(TableProcessor, Object);

public:
    OpenSim_DECLARE_PROPERTY(
            filepath, std::string, "File path to a TimeSeriesTable.");
    OpenSim_DECLARE_LIST_PROPERTY(operators, TableOperator,
            "Operators to apply to the source table of this processor.");
    /// This constructor is only for use when reading (deserializing) from an
    /// XML file.
    TableProcessor() {
        constructProperty_filepath("");
        constructProperty_operators();
    }
    /// Use an in-memory TimeSeriesTable as the source table.
    /// Since this constructor is not explicit, you can provide a
    /// TimeSeriesTable to any function that takes a TableProcessor (in C++).
    TableProcessor(TimeSeriesTable table) : TableProcessor() {
        m_tableProvided = true;
        m_table = std::move(table);
    }
    /// Use a filepath as the source table.
    /// Since this constructor is not explicit, you can provide a string
    /// filepath to any function that takes a TableProcessor.
    TableProcessor(std::string filepath) : TableProcessor() {
        set_filepath(std::move(filepath));
    }
    /// Process and obtain the table. If a filepath is provided, it will be
    /// evaluated relative `relativeToDirectory`, if provided.
    /// If a model is provided, it is used to convert columns from degrees to
    /// radians (if the table has a header with inDegrees=yes) before any
    /// operations are performed.
    TimeSeriesTable process(std::string relativeToDirectory = {},
            const Model* modelToConvertDegreesToRadians = nullptr) const {
        TimeSeriesTable table;
        if (get_filepath().empty()) {
            if (m_tableProvided) {
                table = m_table;
            } else {
                OPENSIM_THROW_FRMOBJ(Exception, "No source table.");
            }
        } else {
            OPENSIM_THROW_IF_FRMOBJ(m_tableProvided, Exception,
                    "Expected either an in-memory table or a filepath, but "
                    "both were provided.");
            std::string path = get_filepath();
            if (!relativeToDirectory.empty()) {
                using SimTK::Pathname;
                path = Pathname::
                        getAbsolutePathnameUsingSpecifiedWorkingDirectory(
                                relativeToDirectory, path);
            }
            table = readTableFromFile(path);
        }

        if (table.hasTableMetaDataKey("inDegrees") &&
                table.getTableMetaDataAsString("inDegrees") == "yes") {
            modelToConvertDegreesToRadians->getSimbodyEngine()
                    .convertDegreesToRadians(table);
        }

        for (int i = 0; i < getProperty_operators().size(); ++i) {
            get_operators(i).operate(table);
        }
        return table;
    }
    /// Returns true if neither a filepath nor an in-memory table have been
    /// provided.
    bool empty() const {
        return !m_tableProvided && get_filepath().empty();
    }
    /// Append an operation to the end of the operations in this processor.
    TableProcessor& append(const TableOperator& op) {
        append_operators(op);
        return *this;
    }
    /// Append all operations in another processor to this processor.
    /// The source table of the provided trajectory is ignored.
    TableProcessor& append(const TableProcessor& traj) {
        for (int i = 0; i < traj.getProperty_operators().size(); ++i) {
            append_operators(traj.get_operators(i));
        }
        return *this;
    }
    /// This operator allows one to write the following code in C++:
    /// @code
    /// TableProcessor proc = TableProcessor("file.sto") |
    ///         TabOpLowPassFilter(6);
    /// @endcode
    TableProcessor& operator|(const TableOperator& right) {
        return append(right);
    }

private:
    bool m_tableProvided = false;
    TimeSeriesTable m_table;
};

/// Apply a low-pass filter to the trajectory.
class OSIMMOCO_API TabOpLowPassFilter : public TableOperator {
    OpenSim_DECLARE_CONCRETE_OBJECT(TabOpLowPassFilter, TableOperator);

public:
    OpenSim_DECLARE_PROPERTY(cutoff_frequency, double,
            "Low-pass cutoff frequency (Hz) (default is -1, which means no "
            "filtering).");
    TabOpLowPassFilter() { constructProperty_cutoff_frequency(-1); }
    TabOpLowPassFilter(double cutoffFrequency) : TabOpLowPassFilter() {
        set_cutoff_frequency(cutoffFrequency);
    }
    void operate(TimeSeriesTable& table) const override {
        if (get_cutoff_frequency() != -1) {
            OPENSIM_THROW_IF(get_cutoff_frequency() <= 0, Exception,
                    format("Expected cutoff frequency to be positive, "
                           "but got %f.",
                            get_cutoff_frequency()));

            table = filterLowpass(table, get_cutoff_frequency(), true);
        }
    }
};

} // namespace OpenSim

#endif // MOCO_TABLEPROCESSOR_H
