#ifndef OPENSIM_TASK_SPACE_CONSTRAINT_MODEL_H_
#define OPENSIM_TASK_SPACE_CONSTRAINT_MODEL_H_
/* -------------------------------------------------------------------------- *
 *               OpenSim: TaskSpaceConstraintModel.h                          *
 * -------------------------------------------------------------------------- *
 * Developed by CFD Research Corporation for a project sponsored by the US    *
 * Army Medical Research and Materiel Command under Contract No.              *
 * W81XWH-22-C-0020. Any views, opinions and/or findings expressed in this    *
 * material are those of the author(s) and should not be construed as an      *
 * official Department of the Army position, policy or decision unless so     *
 * designated by other documentation.                                         *
 *                                                                            *
 * Please refer to the following publication for mathematical details, and    *
 * cite this paper if you use this code in your own research:                 *
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

#include "osimToolsDLL.h"
#include "OpenSim/Common/Object.h"
#include "OpenSim/Simulation/Model/Model.h"

namespace OpenSim {

//================================================================================
//                              DATA STRUCTURES
//================================================================================

/** \brief Calculated by calcConstraintData(). */
struct ConstraintData {
    /** System mass matrix.*/
    SimTK::Matrix M;
    /** Inverse of system mass matrix.*/
    SimTK::Matrix MInv;
    /** Constraint-consistent inertia mass matrix.*/
    SimTK::Matrix Mc;
    /** Inverse of constraint-consistent inertia mass matrix.*/
    SimTK::Matrix McInv;
    /** Transpose of the constraint nullspace matrix. */
    SimTK::Matrix NcT;
    /** Constraint bias term. */
    SimTK::Vector bc;
    /** Constraint Jacobian. */
    SimTK::Matrix Jc;
    /** External forces. */
    SimTK::Matrix Fext;
};

struct supportData {
    SimTK::Matrix Js_agg; //!<@brief aggregate support jacobian
    SimTK::Matrix Vb_agg; //!<@brief aggregate transformation Matrix
    SimTK::Matrix M;      //!<@brief mass matrix
    SimTK::Matrix MInv;   //!<@brief inverse mass matrix
    SimTK::Matrix Mc;     //!<@brief support consistent mass matrix
    SimTK::Matrix McInv;  //!<@brief support consistent inverse mass matrix
    SimTK::Matrix
            NsT; //!<@brief  support consistent Nullspace for support tasks
    SimTK::Matrix Fs_agg;   //!<@brief  Aggregate support force (applied to
                            //!< centers of pressure)
    SimTK::Vector bias_agg; //!<@brief  aggregate bias force (mapping external
                            //!< loads task to bias accel)
    std::map<SimTK::MobilizedBodyIndex, SimTK::Matrix>
            Js; //!<@brief  support jacobian (6xnt)
    std::map<SimTK::MobilizedBodyIndex, SimTK::Matrix>
            Vb; //!<@brief  Support Transformation Matrix
    std::map<SimTK::MobilizedBodyIndex, SimTK::Matrix>
            Fs; //!<@brief  support force (spatial (6x1))
    std::map<SimTK::MobilizedBodyIndex, SimTK::Vec3>
            cop; //!<@brief  center of pressure (3x1)
    std::map<SimTK::MobilizedBodyIndex, SimTK::SpatialVec>
            force; //!<@brief  linear summed cop force and resultant moment
                   //!<(3x2)
    std::map<SimTK::MobilizedBodyIndex, SimTK::Vector>
            bias; //!<@brief  task bias accel (6x1)
    std::map<SimTK::MobilizedBodyIndex, SimTK::MobilizedBody>
            bodies; //!<@brief  task bias accel (6x1)
};

//================================================================================
//                              CONSTRAINT MODELS
//================================================================================
//_____________________________________________________________________________
/**
 * \brief An abstract class for defining constraint models.
 */
class ConstraintModel : public ModelComponent {
    OpenSim_DECLARE_ABSTRACT_OBJECT(ConstraintModel, ModelComponent);

public:
    /**
     * Calculates the constraint data. Note that in the future additional
     * ConstraintData may be provided.
     */
    virtual ConstraintData calcConstraintData(const Model& model, const SimTK::State& s) const = 0;
protected:
    SimTK::Matrix calcLambda(
            const SimTK::Matrix& Phi, const SimTK::Matrix& MInv) const;
};

//_____________________________________________________________________________
/**
 * \brief This model assumes that there are no constraints.
 *
 * \f$ M \ddot{q} + f = \tau \f$
 *
 * \f$ M_c^{-1} = M^{-1}, \; b_c = 0, \; N_c^T = 1 \f$
 */
class OSIMTOOLS_API NoConstraintModel : public ConstraintModel {
    OpenSim_DECLARE_CONCRETE_OBJECT(NoConstraintModel, ConstraintModel);

public:
    ConstraintData calcConstraintData(const Model& model, const SimTK::State& s) const override;
};

//_____________________________________________________________________________
/**
 * \brief This model uses the inertia weighted generalized inverse of the
 * constraint Jacobian as adopted by De Sapio et al. [1] to derive the
 * constrained representation of the equations of motion.
 *
 * \f$ M \ddot{q} + f + \Phi^T \lambda = \tau \f$                       (1)
 *
 * \f$ \Phi \ddot{q} = b \f$                                            (2)
 *
 * @see Utilities.h
 *
 * The goal is to decouple constraint and applied forces through coordinate
 * projection. Multiply Eq. (1) from the left by \f$ \Phi M^{-1} \f$,
 * making use of Eq. (2) and solve for \f$ \lambda \f$
 *
 * \f$ b + \Phi M^{-1} f + \Phi M^{-1} \Phi^T \lambda = \Phi M^{-1} \tau
 * \f$
 *
 * \f$ \lambda = \bar{\Phi}^T (\tau - f) - \Lambda_c b, ;\ \Lambda_c = (\Phi
 * M^{-1} \Phi^T)^{-1}, \; \bar{\Phi}^T = \Lambda_c \Phi M^{-1} \f$      (3)
 *
 * Then combine Eqs. (1) and (3)
 *
 * \f$ M \ddot{q} + f^{\perp} + bc = \tau^{\perp} \f$
 *
 * \f$ b_c = - \Phi^T \Lambda_c b \f$
 *
 * \f$ f^{\perp} = N_c^T f, \; \tau^{\perp} = N_c^T \tau, \; N_c^T = 1 -
 * \Phi^T \bar{\Phi}^T \f$
 *
 * [1] De Sapio, V., & Park, J. (2010). Multitask Constrained Motion
 * Control Using a Mass-Weighted Orthogonal Decomposition. Journal of
 * Applied Mechanics, 77(4), 1–9. https://doi.org/10.1115/1.4000907
 */
class OSIMTOOLS_API DeSapioModel : public ConstraintModel {
    OpenSim_DECLARE_CONCRETE_OBJECT(DeSapioModel, ConstraintModel);

public:
    ConstraintData calcConstraintData(const Model& model, const SimTK::State& s) const override;
};

//_____________________________________________________________________________
/**
 * \brief This model make uses of the Moore - Penrose pseudoinverse (MPP)
 * and the theory of linear projection operators to decouple the constraint
 * and applied forces based on Aghili [1].
 *
 * \f$ M \ddot{q} + f + \Phi^T \lambda = \tau \f$                       (1)
 *
 * \f$ \Phi \ddot{q} = b \f$                                            (2)
 *
 * \f$ N_c = N_c^T = I - \Phi^+ \Phi \f$                                (3)
 *
 * Reexpress Eqs. (1) and (2) using (3)
 *
 * \f$ N_c M \ddot{q} + N_c f = N_c \tau, \; N_c \Phi^T = 0 \f$
 *                                                                      (4)
 * \f$ \ddot{q}_{\parallel} = M (I - N_c) \ddot{q} = \Phi^+ b \f$
 *
 * Combining Eqs. (4) together we can derive the model
 *
 * \f$ M^{'} \ddot{q} + f_{perp} + b_c = \tau_{\perp} \f$
 *
 * \f$ M^{'} = M + N_c M - (N_c M)^T \f$
 *
 * \f$ f^{\perp} = N_c f, \; \tau^{\perp} = N_c \tau \f$
 *
 * \f$ b_c = -M \Phi^+ b \f$
 *
 * [1] Aghili, F. (2005). A unified approach for inverse and direct
 * dynamics of constrained multibody systems based on linear projection
 * operator: Applications to control and simulation. IEEE Transactions on
 * Robotics, 21(5), 834–849. https://doi.org/10.1109/TRO.2005.851380
 */
class OSIMTOOLS_API AghiliModel : public ConstraintModel {
    OpenSim_DECLARE_CONCRETE_OBJECT(AghiliModel, ConstraintModel);

public:
    ConstraintData calcConstraintData(const Model& model, const SimTK::State& s) const override;
};

//_____________________________________________________________________________
/**
 * \brief This model make use of the augmented Jacobian methods for task
 * prioritization based on Mistry [4]. Requires external contact with the
 * environment to be modeled using kinematic constraints.
 *
 * [4] Mistry, Michael & Nakanishi, Jun & Schaal, Stefan. (2007). Task space
 * control with prioritization for balance and locomotion.
 * 331-338. 10.1109/IROS.2007.4399595.
 */
class OSIMTOOLS_API MistryModel : public ConstraintModel {
    OpenSim_DECLARE_CONCRETE_OBJECT(MistryModel, ConstraintModel);

public:
    ConstraintData calcConstraintData(const Model& model, const SimTK::State& s) const override;
};

//_____________________________________________________________________________
/**
 * \brief Computes support-consistent constraint data for models which use
 * contact objects to model interaction with the environment. The center of
 * pressure is calculated for each body in contact with the environment, then
 * constraint data is calculated as if there were a weld constraint at the
 * location of the center of pressure. This model utilizes this the support
 * consistent formulation from Sentis 2007 [1]. 
 *
 * [1] Sentis, L. (2007). Synthesis and Control of Whole-Body
 * Behaviors in Humanoid Systems. Stanford University. Retrieved from
 * http://dl.acm.org/citation.cfm?id=1354211
 *
 *   * @see Utilities.h
 *
 *   * @see ConstraintModel.h
 *
 * */
class OSIMTOOLS_API SupportModel : public ConstraintModel {
    OpenSim_DECLARE_CONCRETE_OBJECT(SupportModel, ConstraintModel);

public:
    ConstraintData calcConstraintData(const Model& model, const SimTK::State& s) const override;
};
} // END namespace OpenSim

#endif // OPENSIM_TASK_SPACE_CONSTRAINT_MODEL_H_
