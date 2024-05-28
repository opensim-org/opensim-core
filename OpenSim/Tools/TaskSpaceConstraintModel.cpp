/* -------------------------------------------------------------------------- *
 *                OpenSim: TaskSpaceConstraintModel.cpp                       *
 * -------------------------------------------------------------------------- *
 * Developed by CFD Research Corporation for a project sponsored by the US    *
 * Army Medical Research and Materiel Command under Contract No.              *
 * W81XWH-22-C-0020. Any views, opinions and/or findings expressed in this    *
 * material are those of the author(s) and should not be construed as an      *
 * official Department of the Army position, policy or decision unless so     *
 * designated by other documentation.                                         *
 *                                                                            *
 * Please refer to the following publication for mathematical details, and    *
 * cite this paper if you use this code in your own work:                 *
 *                                                                            *
 * Pickle and Sundararajan. "Predictive simulation of human movement in       *
 * OpenSim using floating-base task space control".                           *
 *                                                                            *
 * Copyright (c) 2023 CFD Research Corporation and the Authors                *
 *                                                                            *
 * Author(s): Aravind Sundararajan, Nathan Pickle, Garrett Tuer, and Dimitar  *
 *            Stanev                                                          *
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

#include "TaskSpaceConstraintModel.h"

#include "TaskSpaceUtilities.h"

#include "OpenSim/Simulation/Model/SmoothSphereHalfSpaceForce.h"

using namespace OpenSim;
using namespace SimTK;

namespace {
    //TODO: convert to a test instead of running every time
    static void validateConstraintData(const ConstraintData& data) {
        if(data.M.nelt()==0) { 
            throw OpenSim::Exception("Error: ConstraintData field M is empty."); }
        if(data.MInv.nelt()==0) { 
            throw OpenSim::Exception("Error: ConstraintData field MInv is empty."); }
        if(data.McInv.nelt()==0) { 
            throw OpenSim::Exception("Error: ConstraintData field McInv is empty."); }
        if(data.NcT.nelt()==0) { 
            throw OpenSim::Exception("Error: ConstraintData field NcT is empty."); }
        if(data.bc.nelt()==0) { 
            throw OpenSim::Exception("Error: ConstraintData field bc is empty."); }
        if(data.Jc.nelt()==0) { 
            throw OpenSim::Exception("Error: ConstraintData field Jc is empty."); }
    }

    /**
     * @brief wraps SimbodyMatterSubsystem's calcG (constraint matrix)
     *
     * @param s
     * @param model
     * @return Matrix
     */
    Matrix calcConstraintJacobian(const State& s, const Model& model) {
        Matrix Jc;
        model.getMatterSubsystem().calcG(s, Jc);
        return Jc;
    }

    /**
     * @brief wraps SimbodyMatterSubsystem's calcBiasForAccelerationConstraints()
     *
     * @param s
     * @param model
     * @return Vector
     */
    Vector calcConstraintBias(const State& s, const Model& model) {
        // calcBiasForAccelerationConstraints assumes P *udot - bias = 0 we must
        // invert the results so that P * udot = bias
        Vector b;
        model.getMatterSubsystem().calcBiasForAccelerationConstraints(s, b);
        return -1.0 * b;
    }

    /**
     * @brief calculates the support data for the constraint model, iterates over
     * all the forces in the model and computes the center of pressure and the bias
     * acceleration for each force.
     *
     * @param s
     * @param model
     * @return supportData
     */
    supportData calcSupportData(const State& s, const Model& model) {
        supportData out;
        out.bias_agg = Vector(s.getNU(), 0.0);

        auto G = calcConstraintJacobian(s, model);
        if (G.nelt() > 0) { // check constraints and break out early
            log_debug("Constraints found in model. Contact forces will "
                "NOT be used for support calculation.");
            out.Js_agg = G;
            out.M = calcM(s, model);
            out.McInv = calcMInv(s, model);
            Matrix LambdaInv = out.Js_agg * out.McInv * out.Js_agg.transpose();
            Matrix Lambdac = FactorSVDPseudoinverse(LambdaInv);
            auto PhiBarT = Lambdac * out.Js_agg * out.McInv;
            out.bias_agg = calcConstraintBias(s, model);
            out.NsT = 1 - out.Js_agg.transpose() * PhiBarT;
            return out;
        }

        for (const auto& f : model.getComponentList<OpenSim::Force>()) {
            if (f.getConcreteClassName() == "SmoothSphereHalfSpaceForce") {
                auto& fc =
                        dynamic_cast<const OpenSim::SmoothSphereHalfSpaceForce&>(f);
                auto b = fc.getConnectee<ContactSphere>("sphere")
                                .getFrame()
                                .getMobilizedBodyIndex();
                OpenSim::Array<double> f_ext = f.getRecordValues(s);
                SimTK::Vec3 position =
                        fc.getConnectee<ContactSphere>("sphere").get_location();
                SimTK::Vec3 location = fc.getConnectee<ContactSphere>("sphere")
                                            .getFrame()
                                            .getPositionInGround(s);
                auto position_in_ground =
                        fc.getConnectee<ContactSphere>("sphere")
                                .getFrame()
                                .findStationLocationInGround(s, position);
                if (out.cop.find(b) ==
                        out.cop.end()) { // init struct values to 0 at mat and
                                        // compute frame jacobian
                    out.force[b] = SpatialVec(2);
                    out.force[b][0] = 0;
                    out.force[b][1] = 0;
                    out.cop[b] = Vec3(0.0);
                    out.Fs[b] = Matrix(6, 1, 0.0); // spatial
                    out.bias[b] =
                            Vector(out.Js[b].nrow(), 0.0); // task bias acceleration
                    out.bodies[b] = fc.getConnectee<ContactSphere>("sphere")
                                            .getFrame()
                                            .getMobilizedBody();
                }

                // Forces
                out.force[b][1][0] += f_ext[6];
                out.force[b][1][1] += f_ext[7];
                out.force[b][1][2] += f_ext[8];

                // Moments
                out.force[b][0][0] += out.force[b][1][1] * position_in_ground[0]; // supposed to ignore the Y since the cop is on the floor
                out.force[b][0][1] += 0;
                out.force[b][0][2] +=
                        out.force[b][1][1] *
                        position_in_ground[2]; // supposed to ignore the Y since the
                                            // cop is on the floor
            }
        }

        double total_mass = 0.0;
        for (auto it = out.cop.begin(); it != out.cop.end(); ++it) 
        { // loop over keys i map and update center of pressure
            auto k = it->first; // keys are MobilizedBodyIndex
            auto bod = out.bodies[k];
            if (FlattenSpatialVec(out.force[k]).norm() > std::numeric_limits<float>::epsilon()) {
                out.cop[k][0] = out.force[k][0][0] / out.force[k][1][1]; // Mx / Fy
                out.cop[k][1] = 0.0;
                out.cop[k][2] = out.force[k][0][2] / out.force[k][1][1]; // Mz / Fy
                out.cop[k] = out.cop[k] - bod.findStationLocationInGround(s, Vec3(0, 0, 0));
                auto temp = FlattenSpatialVec(out.force[k]);
                out.Fs[k] = temp;

                out.Js[k] = calcFrameJacobian(s, model, k, out.cop[k]); // bod.getBodyOriginLocation(s)
                out.Js[k] = out.Js[k].block(3, 0, 3, s.getNU());

                auto bias = model.getMatterSubsystem().calcBiasForFrameJacobian(s, bod, out.cop[k]); // bod.getBodyOriginLocation(s)

                out.bias[k] = -1.0 * FlattenSpatialVec(bias).getAsVector();

                total_mass += bod.getBodyMass(s);

                if (out.Js_agg.nelt() ==0) {
                    out.Js_agg = out.Js[k];    
                    out.bias_agg = out.bias[k];
                    out.Fs_agg = out.Fs[k];
                } else {
                    out.Js_agg = out.Js_agg + (bod.getBodyMass(s)) * out.Js[k];

                    out.Js_agg = concatenate(out.Js_agg, out.Js[k], 0);                     
                    out.bias_agg = concatenate(out.bias_agg, out.bias[k], 0).getAsVector(); 
                    out.Fs_agg = concatenate(out.Fs_agg, out.Fs[k], 0);
                }
            }
        }

        if (out.Js_agg.nelt() == 0) {
            auto M = calcM(s, model);
            out.M = M;
            out.MInv = M.invert();
            out.NsT = Matrix(s.getNU(), s.getNU());
            out.NsT = 1.0;
            out.Js_agg = Matrix(s.getNU(), s.getNU(), 0.0);
            out.Js_agg = 0.0;
            Matrix Mc = (out.NsT * M) + (1 - out.NsT);
            out.Mc = Mc;
            out.McInv = Mc.invert();
            out.bias_agg = Vector(out.Js_agg.nrow(), 0.0);
            out.Fs_agg = Vector(out.Js_agg.nrow(), 0.0);
            return out;
        }

        out.MInv = calcMInv(s, model);
        Matrix LambdaInv = out.Js_agg * out.MInv * out.Js_agg.transpose();
        Matrix Lambdac = FactorSVDPseudoinverse(LambdaInv);
        auto PhiBarT = Lambdac * out.Js_agg * out.MInv;
        out.NsT = 1.0 - out.Js_agg.transpose() * PhiBarT;
        auto M = calcM(s, model);
        out.M = M;
        Matrix Mc = (out.NsT * M) + 1.0 - out.NsT;
        out.McInv = DampedSVDPseudoinverse(Mc);

        return out;
    }
}

/******************************************************************************/

Matrix ConstraintModel::calcLambda(
        const Matrix& Phi, const Matrix& MInv) const {
    auto LambdaInv = Phi * MInv * ~Phi;
    Matrix Lambda;
    FactorSVD svd(LambdaInv);
    svd.inverse(Lambda);
    return Lambda;
}

/******************************************************************************/

ConstraintData NoConstraintModel::calcConstraintData(const Model& model, const SimTK::State& s) const {
    
    ConstraintData data;
    data.Jc = Matrix(s.getNU(), s.getNU(), 0.0);
    data.Jc = 1;
    data.M = calcM(s, model);
    data.MInv = calcMInv(s, model);
    data.Mc = data.M;
    data.McInv = data.MInv;
    data.bc = Vector(s.getNU(), 0.0);
    data.NcT = Matrix(s.getNU(), s.getNU());
    data.NcT = 1;
    validateConstraintData(data);
    return data;
}
/******************************************************************************/

ConstraintData DeSapioModel::calcConstraintData(const Model& model, const SimTK::State& s) const {
    
    ConstraintData data;
    data.M = calcM(s, model);
    data.MInv = calcMInv(s, model);
    // McInv
    data.McInv = data.MInv;
    // bc
    data.Jc = calcConstraintJacobian(s, model);
    auto JcT = ~data.Jc;
    auto Lambdac = this->calcLambda(data.Jc, data.McInv);
    auto b = calcConstraintBias(s, model);
    // Stanev and Moustakas 2018, Eq 16
    data.bc = -1.0 * JcT * Lambdac * b;
    // NcT
    auto JcBarT = Lambdac * data.Jc * data.McInv;
    data.NcT = 1 - JcT * JcBarT;
    validateConstraintData(data);
    return data;
}
/******************************************************************************/

ConstraintData AghiliModel::calcConstraintData(const Model& model, const SimTK::State& s) const {
    
    ConstraintData data;
    // NcT
    data.Jc = calcConstraintJacobian(s, model);
    Matrix JcInv;
    FactorSVD JcSVD(data.Jc);
    JcSVD.inverse(JcInv);
    data.NcT = 1 - JcInv * data.Jc; // Nc^T = Nc due to MPP properties
    // McInv
    data.M = calcM(s, model);
    data.MInv = calcMInv(s, model);
    auto Ms = data.M + data.NcT * data.M - ~(data.NcT * data.M);
    FactorSVD MsSVD(Ms);
    MsSVD.inverse(data.McInv);
    // bc
    auto b = calcConstraintBias(s, model);
    data.bc = -1.0 * data.M * JcInv * b;
    validateConstraintData(data);
    return data;
}

/******************************************************************************/

ConstraintData MistryModel::calcConstraintData(const Model& model, const SimTK::State& s) const {

    ConstraintData data;
    Matrix Jc = calcConstraintJacobian(s, model);
    Matrix Jcstar = FactorSVDPseudoinverse(Jc);
    data.NcT = 1 - (Jcstar * Jc);
    data.Jc = Jc;
    data.M = calcM(s, model);
    data.MInv = calcMInv(s, model);
    Matrix Mc = (data.NcT * data.M) + (1 - data.NcT);
    data.McInv = FactorSVDPseudoinverse(Mc);
    data.bc = calcConstraintBias(s, model);
    validateConstraintData(data);
    return data;
}

/******************************************************************************/

ConstraintData SupportModel::calcConstraintData(const Model& model, const SimTK::State& s) const {
    
    ConstraintData data;
    supportData out = calcSupportData(s, model);
    data.Jc = out.Js_agg;
    data.NcT = out.NsT;
    data.M = out.M;
    data.Mc = out.Mc;
    data.MInv = out.MInv;
    //TODO: This should be out.McInv (constraint consistent), but doing so conflicts with 
    //      gait simulations since ground contact is treated as a constraint and given
    //      priority -1. This causes foot position tasks to be ineffective. Need to 
    //      develop a workaround.
    data.McInv = out.MInv;
    Matrix JcPinv = FactorSVDPseudoinverse(data.Jc);
    data.bc = JcPinv * out.bias_agg;
    data.Fext = out.Fs_agg;
    validateConstraintData(data);
    return data;
}