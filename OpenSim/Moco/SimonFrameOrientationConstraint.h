#ifndef OPENSIM_MOCOFRAMEORIENTATIONCONSTRAINT_H 
#define OPENSIM_MOCOFRAMEORIENTATIONCONSTRAINT_H 
/* -------------------------------------------------------------------------- *
 * OpenSim Moco: MocoFrameOrientationConstraint.h                             *
 * -------------------------------------------------------------------------- *
 *                                                                            *
 * Author(s): Size Zheng                                              		  *
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

#include "MocoConstraint.h"
#include <OpenSim/Moco/osimMoco.h>
#include "osimMocoDLL.h"

namespace OpenSim {

class OSIMMOCO_API MocoFrameOrientationConstraintPair : public Object {
    OpenSim_DECLARE_CONCRETE_OBJECT(MocoFrameOrientationConstraintPair, Object);

public:
    OpenSim_DECLARE_PROPERTY(frame1_path, std::string,
        "The first model frame path of the pair.");
    OpenSim_DECLARE_PROPERTY(frame2_path, std::string,
        "The second model frame path of the pair.");
    OpenSim_DECLARE_PROPERTY(minimum_angle, double,
        "The minimum distance apart that the two frame origins can be "
        "(meters).");
    OpenSim_DECLARE_PROPERTY(maximum_angle, double,
        "The maximum distance apart that the two frame origins can be "
        "(meters).")

    MocoFrameOrientationConstraintPair();
    MocoFrameOrientationConstraintPair(std::string firstFramePath,
        std::string secondFramePath, double minimum_angle,
        double maximum_angle);

private:
    void constructProperties();
};

class OSIMMOCO_API MocoFrameOrientationConstraint : public MocoPathConstraint {
    OpenSim_DECLARE_CONCRETE_OBJECT(MocoFrameOrientationConstraint, MocoPathConstraint);

public:
    MocoFrameOrientationConstraint();
    
    void addFramePair(MocoFrameOrientationConstraintPair pair) {
        append_frame_pairs(std::move(pair));
    }
    void addFramePair(const std::string& frame1_path,const std::string& frame2_path, double minimum_angle,
        double maximum_angle) {
        append_frame_pairs(MocoFrameOrientationConstraintPair(frame1_path,frame2_path, minimum_angle,maximum_angle));
    }
protected:
    void initializeOnModelImpl(const Model& model, const MocoProblemInfo&) const override;
    void calcPathConstraintErrorsImpl(const SimTK::State& state, SimTK::Vector& errors) const override;
private:
    OpenSim_DECLARE_LIST_PROPERTY(frame_pairs,
        MocoFrameOrientationConstraintPair,
        "Pairs of frames whose origins are constrained to be within "
        "minimum and maximum bounds.");
       
    void constructProperties();
    mutable std::vector<std::pair<SimTK::ReferencePtr<const Frame>,
        SimTK::ReferencePtr<const Frame>>> m_frame_pairs;

};

} // namespace OpenSim

#endif // OPENSIM_MOCOFRAMEORIENTATIONCONSTRAINT_H 
