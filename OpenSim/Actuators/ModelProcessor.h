#ifndef OPENSIM_MODELPROCESSOR_H
#define OPENSIM_MODELPROCESSOR_H
/* -------------------------------------------------------------------------- *
 * OpenSim: ModelProcessor.h                                                  *
 * -------------------------------------------------------------------------- *
 * Copyright (c) 2019 Stanford University and the Authors                     *
 *                                                                            *
 * Author(s): Christopher Dembia                                              *
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

#include "osimActuatorsDLL.h"

#include <OpenSim/Simulation/Model/Model.h>

namespace OpenSim {

/** This abstract base class describes *any* operation that modifies a Model
as part of a ModelProcessor. */
class OSIMACTUATORS_API ModelOperator : public Object {
    OpenSim_DECLARE_ABSTRACT_OBJECT(ModelOperator, Object);

public:
    /** Perform an operation on the model, using `relativeToDirectory` to locate
    any files that this operator reads. */
    virtual void operate(
            Model& model, const std::string& relativeToDirectory) const = 0;
};

/** This class describes a workflow for processing a Model using
ModelOperator%s. The user must provide a source model via either the model
property or the filepath property. In C++, one can easily chain together
the operators in a processor using the C++ pipe operator:
@code
ModelProcessor proc = ModelProcessor("model.osim") | ModOpAddReserves();
@endcode */
class OSIMACTUATORS_API ModelProcessor : public Object {
    OpenSim_DECLARE_CONCRETE_OBJECT(ModelProcessor, Object);

public:
    OpenSim_DECLARE_PROPERTY(
            filepath, std::string, "File path to a Model (.osim).");
    OpenSim_DECLARE_LIST_PROPERTY(operators, ModelOperator,
            "Operators to apply to the source Model of this processor.");
    /** This constructor is only for use when reading (deserializing) from an
    XML file. */
    ModelProcessor() {
        constructProperty_filepath("");
        constructProperty_operators();
        constructProperty_model();
    }

    /** Use a Model object as the source model.
    Since this constructor is not explicit, you can provide a Model to
    any function that takes a ModelProcessor (in C++). */
    ModelProcessor(Model model) : ModelProcessor() {
        model.finalizeConnections();
        set_model(std::move(model));
    }

    /** Use the filepath of a .osim file to obtain the source model.
    Since this constructor is not explicit, you can provide a string
    filepath any function that takes a ModelProcessor (in C++). */
    ModelProcessor(std::string filepath) : ModelProcessor() {
        set_filepath(std::move(filepath));
    }

    /** Set the base model. This returns a raw pointer equal to the pointer
    provided. */
    Model* setModel(std::unique_ptr<Model> model) {
        // Write the connectee paths to properties.
        model->finalizeConnections();
        updProperty_model().clear();
        updProperty_model().adoptAndAppendValue(model.release());
        return &upd_model();
    }

    /** Obtain the base model, if one was provided via the model property or
    setModel(). This ignores base models specified via the filepath
    property. */
    const Model& getModel() const {
        OPENSIM_THROW_IF_FRMOBJ(getProperty_model().empty(), Exception,
                "No base model available.");
        return get_model();
    }

    /** Obtain a mutable reference to the base model, if one was provided via
    the model property or setModel(). This ignores base models specified
    via the filepath property. */
    Model& updModel() {
        OPENSIM_THROW_IF_FRMOBJ(getProperty_model().empty(), Exception,
                "No base model available.");
        return upd_model();
    }

    /** Process and obtain the model. If the base model is specified via the
    filepath property, the filepath will be evaluated relative to
    `relativeToDirectory`, if provided. */
    Model process(const std::string& relativeToDirectory = {}) const {
        Model model;
        if (get_filepath().empty()) {
            if (!getProperty_model().empty()) {
                model = get_model();
            } else {
                OPENSIM_THROW_FRMOBJ(Exception, "No source model.");
            }
        } else {
            OPENSIM_THROW_IF_FRMOBJ(!getProperty_model().empty(), Exception,
                    "Expected either a Model object or a filepath, but "
                    "both were provided.");
            std::string path = get_filepath();
            if (!relativeToDirectory.empty()) {
                using SimTK::Pathname;
                path = Pathname::
                        getAbsolutePathnameUsingSpecifiedWorkingDirectory(
                                relativeToDirectory, path);
            }
            Model modelFromFile(path);
            model = std::move(modelFromFile);
            model.finalizeFromProperties();
            model.finalizeConnections();
        }

        for (int i = 0; i < getProperty_operators().size(); ++i) {
            get_operators(i).operate(model, relativeToDirectory);
        }
        return model;
    }

    /** Append an operation to the end of the operations in this processor. */
    ModelProcessor& append(const ModelOperator& op) {
        append_operators(op);
        return *this;
    }
    /** This operator allows one to write the following code in C++:
    @code
    ModelProcessor proc = ModelProcessor("model.osim") | ModOpAddReserves();
    @endcode */
    ModelProcessor& operator|(const ModelOperator& right) {
        return append(right);
    }

private:
    OpenSim_DECLARE_OPTIONAL_PROPERTY(model, Model, "Base model to process.");
};

} // namespace OpenSim

#endif // OPENSIM_MODELPROCESSOR_H
