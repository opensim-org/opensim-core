#ifndef MOCO_TABLEPROCESSOR_H
#define MOCO_TABLEPROCESSOR_H
/* -------------------------------------------------------------------------- *
 * OpenSim Moco: TableProcessor.h                                             *
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

#include "../MocoUtilities.h"
#include <algorithm>

#include <OpenSim/Common/TimeSeriesTable.h>
#include <OpenSim/Simulation/Model/Model.h>

namespace OpenSim {

/// This abstract class describes *any* operation that consumes a
/// modifies a TimeSeriesTable as part of a TableProcessor.
class OSIMMOCO_API TableOperator : public Object {
    OpenSim_DECLARE_ABSTRACT_OBJECT(TableOperator, Object);

public:
    virtual void operate(TimeSeriesTable& table, const Model* model) const = 0;
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
    /// operations are performed. This model is accessible by any 
    /// TableOperator that require it.
    TimeSeriesTable process(std::string relativeToDirectory = {},
            const Model* model = nullptr) const {
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
            model->getSimbodyEngine().convertDegreesToRadians(table);
        }

        for (int i = 0; i < getProperty_operators().size(); ++i) {
            get_operators(i).operate(table, model);
        }
        return table;
    }
    TimeSeriesTable process(const Model* model = nullptr) {
        return process({}, model);
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
    void operate(TimeSeriesTable& table, const Model* model) const override {
        if (get_cutoff_frequency() != -1) {
            OPENSIM_THROW_IF(get_cutoff_frequency() <= 0, Exception,
                    format("Expected cutoff frequency to be positive, "
                           "but got %f.",
                            get_cutoff_frequency()));

            table = filterLowpass(table, get_cutoff_frequency(), true);
        }
    }
};

/// Update table column headers to show full path to components in a model 
/// tree. If a column header matches a component name, the column header is 
/// replaced by the full path to the component. This operation applies the 
/// findComponent() method on the column header string, therefore the column 
/// header string may already contain a full or partial path to the component. 
///
/// The user may append a "/value" or "/speed" suffix to the end of the full 
/// path to the component.
///
/// However, for column headers with the suffix "_u" to represent speeds, 
/// a flag may be optionally set to automatically replace the "_u" with 
/// "/speeds" for these columns only (overriding the above).
///
/// If a component given by the column header is not found in the model, the 
/// column header is left unchanged.
///
/// Assumption: all column headers names are unique.
class OSIMMOCO_API TabOpAbsolutePathColumnLabels : public TableOperator {
    OpenSim_DECLARE_CONCRETE_OBJECT(TabOpAbsolutePathColumnLabels, TableOperator);
    OpenSim_DECLARE_PROPERTY(suffix, std::string,
            "Append optional \"value\" or \"speed\" suffix to full path. "
            "Default: no suffix.");
    OpenSim_DECLARE_PROPERTY(auto_change_u_to_speed, bool,
            "If the column name contains a \"_u\" suffix, set this flag to "
            "automatically change \"_u\" to \"/speed\". This will override any "
            "previously-set suffix. Default: false.");

public:
    TabOpAbsolutePathColumnLabels() {
        constructProperty_suffix("");
        constructProperty_auto_change_u_to_speed(false);
    };
    TabOpAbsolutePathColumnLabels(std::string suffval, 
            bool auto_change_u_flag = false)
                : TabOpAbsolutePathColumnLabels() {
        std::transform(
                suffval.begin(), suffval.end(), suffval.begin(), ::tolower);
        set_suffix(suffval);
        set_auto_change_u_to_speed(auto_change_u_flag);
    };

    void operate(TimeSeriesTable& table, const Model* model) const override {

        for (int i = 0; i < table.getNumColumns(); i++) {

            // check model nullptr
            OPENSIM_THROW_IF(!model, Exception,
                    format("Expected a model, but no model was provided."));

            // check valid suffix
            OPENSIM_THROW_IF(get_suffix().compare("") &&
                                     get_suffix().compare("value") &&
                                     get_suffix().compare("speed"),
                    Exception,
                    format("Expected suffix \"value\", \"speed\" or no "
                           "suffix. The provided suffix \"%s\" did not match "
                           "these options.",
                            get_suffix()));

            // get the column header
            std::string colheader = table.getColumnLabel(i);

            // trim the "_u" if column header has suffix "_u" and auto changed flagged
            bool autospeedflag = get_auto_change_u_to_speed() &&
                                 (colheader.rfind("_u") == (colheader.length() - 2));
            if (autospeedflag)
                colheader.erase(colheader.end() - 2, colheader.end());
            
            // find the component with the same name as the current column header
            if (const Component* found =
                model->findComponent(ComponentPath(colheader))) {

                // get the full path
                std::string pathstring = found->getAbsolutePathString();

                // automatically append "/speed" if flag is set
                if (autospeedflag) {
                    pathstring.append("/speed");
                }
                // otherwise just append the suffix provided (if any)
                else {
                    pathstring.append(
                            get_suffix().compare("") ? "/" + get_suffix() : "");
                }

                // update the column label
                table.setColumnLabel(i, pathstring);
            }
        }
    }
};

} // namespace OpenSim

#endif // MOCO_TABLEPROCESSOR_H
