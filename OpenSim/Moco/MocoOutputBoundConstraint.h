//
// Created by Allison John on 7/24/24.
// Make an output bound constraint that can bound one or two outputs between two
// functions
//

#ifndef MOCOOUTPUTBOUNDCONSTRAINT_H
#define MOCOOUTPUTBOUNDCONSTRAINT_H

#include "MocoConstraint.h"

namespace OpenSim {

class MocoOutputBoundConstraint : MocoPathConstraint {
    OpenSim_DECLARE_CONCRETE_OBJECT(MocoOutputBoundConstraint, MocoPathConstraint);

public:
    MocoOutputBoundConstraint() { constructProperties(); }

    /** Set the absolute path to the output in the model to be used in this path
    constraint. The format is "/path/to/component|output_name". */
    void addOutputPath(std::string path) {
        append_control_paths(std::move(path));
    }
    void setControlPaths(const std::vector<std::string>& controlPaths) {
        updProperty_control_paths().clear();
        for (const auto& path : controlPaths) { append_control_paths(path); }
    }
    void clearControlPaths() { updProperty_control_paths().clear(); }
    std::vector<std::string> getControlPaths() const {
        std::vector<std::string> paths;
        for (int i = 0; i < getProperty_control_paths().size(); ++i) {
            paths.push_back(get_control_paths(i));
        }
        return paths;
    }

    /** Set the exponent applied to the output value in the constraint. This
    exponent is applied when minimizing the norm of a vector type output. */
    void setExponent(int exponent) { set_exponent(exponent); }
    int getExponent() const { return get_exponent(); }

    /** Set the index to the value to be constrained when a vector type Output is
    specified. For SpatialVec Outputs, indices 0, 1, and 2 refer to the
    rotational components and indices 3, 4, and 5 refer to the translational
    components. A value of -1 indicates to constrain the vector norm (which is
    the default setting). If an index for a type double Output is provided, an
    exception is thrown. */
    void setOutputIndex(int index) { set_output_index(index); }
    int getOutputIndex() const { return get_output_index(); }

};

} // namespace OpenSim

#endif //MOCOOUTPUTBOUNDCONSTRAINT_H


two outputs?
    index for each output?