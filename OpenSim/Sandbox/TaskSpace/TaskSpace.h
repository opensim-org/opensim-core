/* -------------------------------------------------------------------------- *
*                               Simbody(tm)                                  *
* -------------------------------------------------------------------------- *
* This is part of the SimTK biosimulation toolkit originating from           *
* Simbios, the NIH National Center for Physics-Based Simulation of           *
* Biological Structures at Stanford, funded under the NIH Roadmap for        *
* Medical Research, grant U54 GM072970. See https://simtk.org/home/simbody.  *
*                                                                            *
* Portions Copyright (c) 2005-2017 Stanford University and the Authors.      *
* Authors: Chris Dembia                                                      *
* Contributors: Michael Sherman, Dimitar Stanev                              *
*                                                                            *
* Licensed under the Apache License, Version 2.0 (the "License"); you may    *
* not use this file except in compliance with the License. You may obtain a  *
* copy of the License at http://www.apache.org/licenses/LICENSE-2.0.         *
*                                                                            *
* Unless required by applicable law or agreed to in writing, software        *
* distributed under the License is distributed on an "AS IS" BASIS,          *
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   *
* See the License for the specific language governing permissions and        *
* limitations under the License.                                             *
* -------------------------------------------------------------------------- */

#ifndef TASK_SPACE_H
#define TASK_SPACE_H

#include <SimTKmath.h>
#include <simbody/internal/common.h>
#include <simbody/internal/SimbodyMatterSubsystem.h>
#include <simbody/internal/Force_Gravity.h>

#include <OpenSim/Simulation/Model/ModelComponent.h>
#include <OpenSim/Simulation/Model/Model.h>

using namespace SimTK;

namespace OpenSim
{
    /**
    * Macro for getters
    */
#define TASKSPACEQUANTITY_MEMBERS(CLASS, MEMVAR, SHORT) \
private: \
    CLASS m_ ## MEMVAR; \
public: \
    const CLASS & get ## CLASS () const { \
        return m_ ## MEMVAR; \
    } \
    const CLASS & SHORT () const { \
        return m_ ## MEMVAR; \
    }

    /** (Experimental – API will change – use at your own risk) Efficient and
    * convenient computation of quantities necessary for task-space
    * model-based controllers.
    *
    * This class provides convenient access to the quantities used in task-space
    * or operational-space controllers, such as the Jacobian and the task-space
    * mass matrix. Each such quantity is encapsulated in its own class, and objects
    * of these types can be used as if they were the quantities (e.g. matrix)
    * themselves. This encapsulation allows us to perform
    * the necessary calculations more efficiently under the covers.
    *
    * Computing quantities such as the Jacobian \a explicitly is often very
    * inefficient, and is usually unnecessary. The Jacobian usually appears in a
    * matrix-vector product, which is more efficient to compute. This class and
    * the classes within use operator overloading to allow a user to write code
    * as if they had the matrices explicitly; internally, we compute matrix-vector
    * products instead. Also, this class caches the quantities once they've
    * been computed.
    *
    * <h3>List of task-space quantities</h3>
    *
    * In this class, we use Khatib's notation for operational-space controllers [1].
    * This notation conflicts with the notation used elsewhere in Simbody.
    *  - nu: number of degrees of freedom.
    *  - nt: number of tasks (e.g., number of stations).
    *  - nst: number of scalar tasks; 3 * nt.
    *  - \f$ A \f$ (nu x nu): joint-space mass matrix (M elsewhere in Simbody).
    *  - \f$ b \f$ (nu x 1): joint-space inertial forces (Coriolis, etc.).
    *  - \f$ g \f$ (nu x 1): joint-space gravity forces.
    *  - \f$ \Lambda \f$ (nst x nst): task-space mass matrix.
    *  - \f$ p \f$ (nst x 1): task-space gravity forces.
    *  - \f$ \mu \f$ (nst x 1): task-space inertial forces.
    *  - \f$ J \f$ (nst x nu): task Jacobian.
    *  - \f$ \bar{J} \f$ (nu x nst): dynamically consistent generalized inverse of the Jacobian.
    *  - \f$ N \f$ (nu x nu): nullspace projection matrix.
    *
    *  See the individual TaskSpaceQuantity's for more information.
    *
    * <h3>Usage</h3>
    *
    * We expect you to use this class within a Force::Custom::Implementation, but
    * this is not the only option. However, this class can only be used within a
    * class that has a realizeTopology method. Here are the necessary steps for
    * making use of this class:
    *
    *  -# Specify your tasks with the TaskSpace::addTask method. If your tasks
    *     don't change throughout your simulation, you can do this in a constructor.
    *  -# Call TaskSpace::realizeTopology within the owning class' realizeTopology.
    *     This is used to allocate cache entries.
    *  -# Access the quantities (perhaps in a calcForce method).
    *
    * Look at the TaskSpaceControl-{UR10,Atlas} examples to see how to use this
    * class.
    *
    * [1] Khatib, Oussama, et al. "Robotics-based synthesis of human motion."
    * Journal of physiology-Paris 103.3 (2009): 211-219.
    */
    class TaskSpace : public ModelComponent
    {
        OpenSim_DECLARE_CONCRETE_OBJECT(TaskSpace, ModelComponent);
    public:

        /** Constructor creates a new container for tasks at the same priority,
        initially containing no tasks.
        */
        TaskSpace()
        {
        }

        ~TaskSpace()
        {
        }

        /** The SimbodyMatterSubsystem that this TaskSpace object uses
        * to compute its quantities.
        */
        const SimbodyMatterSubsystem& getMatterSubsystem() const
        {
            return _model->getMatterSubsystem();
        }

        /** The GravityForce that this TaskSpace object uses
        * to compute its quantities.
        */
        const SimTK::Force::Gravity& getGravityForce() const
        {
            return _model->getGravityForce();
        }

        /** Use this to identify Station tasks that have been added to this
        * TaskSpace.
        */
        SimTK_DEFINE_UNIQUE_LOCAL_INDEX_TYPE(TaskSpace, StationTaskIndex);

        /// @name Station tasks.
        /// @{
        /** Add a task for a station (point fixed on a rigid body).
        * @param[in] body The index of the body on which the point is fixed.
        * @param[in] station The point of the body, expressed in the body frame.
        */
        void addStationTask(MobilizedBodyIndex body, Vec3 station)
        {
            m_indices.push_back(body);
            m_stations.push_back(station);
        }

        /** The number of calls to add*Task that have been made (nt).
        */
        unsigned int getNumTasks() const
        {
            return m_indices.size();
        }

        /** The dimensionality of the task space (nst).
        */
        unsigned int getNumScalarTasks() const
        {
            return 3 * m_indices.size();
        }

        /// @}

        /// @name Convenience calculations.
        /// @{
        /** Obtain the location and velocity, in the ground frame and expressed
        * in the ground frame, of the station for a given station task.
        */
        void findStationLocationAndVelocityInGround(const State& s,
            StationTaskIndex index,
            const Vec3& stationLocationInBody,
            Vec3& locationInGround, Vec3& velocityInGround) const
        {
            getMatterSubsystem().getMobilizedBody(m_indices[index])
                .findStationLocationAndVelocityInGround(s,
                    stationLocationInBody,
                    locationInGround, velocityInGround);
        }

        /** Given accelerations, computes inverse dynamics in the task-space
        * \f$ F = \Lambda F^{*} \mu + p \f$ (nst x 1).
        *
        * This is used to perform feedforward control: given a control law that
        * is specified as task-space accelerations, this provides the task-space
        * forces that achieve those accelerations.
        */
        Vector calcInverseDynamics(const State& s,
            const Vector& taskAccelerations) const;

        Vector calcGravityCompensation(const State& s) const;

        /** The joint-space gravity forces (nu x 1).
        */
        Vector g(const State& s) const
        {
            return getGravity().systemGravity(s);
        }

        /// @}

    protected:

        void extendConnectToModel(Model& model) override
        {
            Super::extendConnectToModel(model);

            addComponent(&m_jacobian);
            m_jacobian.setTaskSpace(this);
            addComponent(&m_jacobianTranspose);
            m_jacobianTranspose.setTaskSpace(this);
            addComponent(&m_inertia);
            m_inertia.setTaskSpace(this);
            addComponent(&m_inertiaInverse);
            m_inertiaInverse.setTaskSpace(this);
            addComponent(&m_jacobianInverse);
            m_jacobianInverse.setTaskSpace(this);
            addComponent(&m_jacobianInverseTranspose);
            m_jacobianInverseTranspose.setTaskSpace(this);
            addComponent(&m_inertialForces);
            m_inertialForces.setTaskSpace(this);
            addComponent(&m_gravity);
            m_gravity.setTaskSpace(this);
            addComponent(&m_nullspace);
            m_nullspace.setTaskSpace(this);
            addComponent(&m_nullspaceTranspose);
            m_nullspaceTranspose.setTaskSpace(this);
        }


    private:

        const Array_<MobilizedBodyIndex>& getMobilizedBodyIndices() const
        {
            return m_indices;
        }

        const Array_<Vec3>& getStations() const
        {
            return m_stations;
        }

        //==========================================================================
        // Member variables.
        //==========================================================================

        // For station tasks
        Array_<MobilizedBodyIndex> m_indices;
        Array_<Vec3> m_stations;

    public:

        //==========================================================================
        // nested classes
        //==========================================================================

        /** An abstract class for common task-space Matrix or Vector quantities.
        *
        * All task-space quantities must be capable of providing their explicit
        * value. After this value is computed, it is cached in the State for
        * efficiency. These classes may optionally provide operators that allow
        * for more efficient computations.
        *
        * The template parameter T is the type of the task-space quantity
        * (e.g., Matrix). The parameter S is used when allocating the cache entry;
        * it is the earliest stage at which the cache entry can be provided.
        */
        template <typename T, Stage::Level S = Stage::Position>
        class TaskSpaceQuantity : public Component
        {
        public:

            TaskSpaceQuantity(std::string cacheName)
                : m_cacheName(cacheName)
            {
            }

            /** Obtain this quantity explicitly. If possible, use operators
            * instead of this method.
            */
            const T& getValue(const State& s) const
            {
                if (!isCacheVariableValid(s, m_cacheName))
                {
                    T& value = updCacheVariableValue<T>(s, m_cacheName);
                    updateCache(s, value);
                    markCacheVariableValid(s, m_cacheName);

                    return value;
                }
                return getCacheVariableValue<T>(s, m_cacheName);
            }

            void setTaskSpace(TaskSpace* ref)
            {
                m_tspace = ref;
            }

        protected:

            ReferencePtr<TaskSpace> m_tspace;

            void extendAddToSystem(SimTK::MultibodySystem& system) const override
            {
                T value;
                addCacheVariable(m_cacheName, value, getEarliestStage());
            }

        private:

            std::string m_cacheName;

            virtual void updateCache(const State& s, T& cache) const = 0;

            // The earliest stage at which this quantity can be calculated. This
            // is used for creating lazy cache entries.
            static Stage getEarliestStage()
            {
                return S;
            }
        };

        // Forward declarations.
        class JacobianTranspose;
        class Inertia;
        class DynamicallyConsistentJacobianInverseTranspose;
        class InertiaInverse;
        class NullspaceProjectionTranspose;

        /** Relates task-space velocities to generalized speeds; (nst x nu).
        */
        class Jacobian : public TaskSpaceQuantity<Matrix>
        {
            OpenSim_DECLARE_CONCRETE_OBJECT(Jacobian, TaskSpaceQuantity<Matrix>);
        public:

            Jacobian()
                : TaskSpaceQuantity<Matrix>("jacobian")
            {
            }

            const JacobianTranspose& transpose() const;

            /// Using this operator is likely more efficient than obtaining this
            /// matrix explicitly and performing the multiplication on your own.
            Vector multiplyByJ(const State& s, const Vector& u) const;

            // TODO Matrix_<Vec3> operator*(const Matrix& u) const;
            // TODO Matrix operator*(const Matrix& u) const;
            // TODO Matrix operator*(const NullspaceProjection& N) const;
        private:

            void updateCache(const State& s, Matrix& cache) const override;
        };

        /** Used to compute task-space forces; (nu x nst).
        */
        class JacobianTranspose : public TaskSpaceQuantity<Matrix>
        {
            OpenSim_DECLARE_CONCRETE_OBJECT(JacobianTranspose,
                TaskSpaceQuantity<Matrix>);
        public:

            JacobianTranspose()
                : TaskSpaceQuantity<Matrix>("jacobianTranspose")
            {
            }

            const Jacobian& transpose() const;

            const Jacobian& operator~()
            {
                return transpose();
            }

            Vector multiplyByJT(const State& s, const Vector_<Vec3>& f_GP) const;

            Vector multiplyByJT(const State& s, const Vector& f_GP) const;

            Vector multiplyByJT(const State& s, const Vec3& f_GP) const;

            // TODO Matrix operator*(const Matrix_<Vec3>& f_GP) const;

            Matrix multiplyByJT(const State& s, const Matrix& f_GP) const;

            Matrix multiplyByJT(const State& s, const TaskSpace::Inertia& Lambda) const;

            Matrix multiplyByJT(const State& s,
                const TaskSpace::DynamicallyConsistentJacobianInverseTranspose&
                JBarT) const;

        private:

            void updateCache(const State& s, Matrix& cache) const override;
        };

        /** Task-space inertia matrix; \f$ \Lambda = (J A^{-1} J^T)^{-1} \f$
        * (nst x nst).
        */
        class Inertia : public TaskSpaceQuantity<Matrix>
        {
            OpenSim_DECLARE_CONCRETE_OBJECT(Inertia, TaskSpaceQuantity<Matrix>);
        public:

            Inertia()
                : TaskSpaceQuantity<Matrix>("inertia")
            {
            }

            const InertiaInverse& inverse() const;

            Vector multiplyByLambda(const State& s, const Vector& a) const;

            Vector multiplyByLambda(const State& s, const Vec3& a) const;

        private:

            void updateCache(const State& s, Matrix& cache) const override;
        };

        /** Inverse of task-space inertia matrix;
        * \f$ \Lambda^{-1} = J M^{-1} J^T \f$ (nst x nst).
        *
        * This is only needed for computing the Inertia matrix.
        */
        class InertiaInverse : public TaskSpaceQuantity<Matrix>
        {
            OpenSim_DECLARE_CONCRETE_OBJECT(InertiaInverse, TaskSpaceQuantity<Matrix>);
        public:

            InertiaInverse()
                : TaskSpaceQuantity<Matrix>("inertiaInverse")
            {
            }

            const Inertia& inverse() const;

        private:

            void updateCache(const State& s, Matrix& cache) const override;
        };

        /** Mass-matrix weighted generalized inverse;
        * \f$ \bar{J} = A^{-1} J^T \Lambda \f$ (nu x nst).
        */
        class DynamicallyConsistentJacobianInverse : public TaskSpaceQuantity<Matrix>
        {
            OpenSim_DECLARE_CONCRETE_OBJECT(DynamicallyConsistentJacobianInverse,
                TaskSpaceQuantity<Matrix>);
        public:

            DynamicallyConsistentJacobianInverse()
                : TaskSpaceQuantity<Matrix>("jacobianInverse")
            {
            }

            const DynamicallyConsistentJacobianInverseTranspose& transpose() const;

            const DynamicallyConsistentJacobianInverseTranspose& operator~() const
            {
                return transpose();
            }

            Vector multiplyByJBar(const State& s, const Vector& vec) const;

            Matrix multiplyByJBar(const State& s, const Matrix& mat) const;

        private:

            void updateCache(const State& s, Matrix& cache) const override;
        };

        /** (nst x nu).
        */
        class DynamicallyConsistentJacobianInverseTranspose :
            public TaskSpaceQuantity<Matrix>
        {
            OpenSim_DECLARE_CONCRETE_OBJECT(DynamicallyConsistentJacobianInverseTranspose,
                TaskSpaceQuantity<Matrix>);
        public:

            DynamicallyConsistentJacobianInverseTranspose()
                : TaskSpaceQuantity<Matrix>("jacobianInverseTranspoase")
            {
            }

            const DynamicallyConsistentJacobianInverse& transpose() const;

            const DynamicallyConsistentJacobianInverse& operator~() const

            {
                return transpose();
            }

            Vector multiplyByJBarT(const State& s, const Vector& g) const;

        private:

            void updateCache(const State& s, Matrix& cache) const override;
        };

        /** Includes Coriolis forces and the like;
        * \f$ \mu = \bar{J}^T b - \Lambda \dot{J} u \f$ (nst x 1).
        */
        class InertialForces : public TaskSpaceQuantity<Vector, Stage::Velocity>
        {
            OpenSim_DECLARE_CONCRETE_OBJECT(InertialForces,
                TaskSpaceQuantity<Vector>);//problem with stage
        public:

            InertialForces()
                : TaskSpaceQuantity<Vector, Stage::Velocity>("inertialForce")
            {
            }

        private:

            void updateCache(const State& s, Vector& cache) const override;
        };

        /** The task-space forces arising from gravity;
        * \f$ p = \bar{J}^T g \f$ (nst x 1).
        */
        class Gravity : public TaskSpaceQuantity<Vector>
        {
            OpenSim_DECLARE_CONCRETE_OBJECT(Gravity, TaskSpaceQuantity<Vector>);
        public:

            Gravity()
                : TaskSpaceQuantity<Vector>("gravityForce")
            {
            }

            /** The joint-space gravity forces (nu x 1).
            */
            Vector systemGravity(const State& s) const;

            /** A shorthand for the joint-space gravity forces (nu x 1).
            */
            Vector g(const State& s) const
            {
                return systemGravity(s);
            }

        private:

            void updateCache(const State& s, Vector& cache) const override;
        };

        /** Used to prioritize tasks; \f$ N = I - \bar{J} J \f$ (nu x nu).
        */
        class NullspaceProjection : public TaskSpaceQuantity<Matrix>
        {
            OpenSim_DECLARE_CONCRETE_OBJECT(NullspaceProjection,
                TaskSpaceQuantity<Matrix>);
        public:

            NullspaceProjection()
                : TaskSpaceQuantity<Matrix>("nullspace")
            {
            }

            const NullspaceProjectionTranspose& transpose() const;

            const NullspaceProjectionTranspose& operator~() const
            {
                return transpose();
            }

            Vector multiplyByN(const State& s, const Vector& vec) const;

        private:

            void updateCache(const State& s, Matrix& cache) const override;
        };

        /** (nu x nu).
        */
        class NullspaceProjectionTranspose : public TaskSpaceQuantity<Matrix>
        {
            OpenSim_DECLARE_CONCRETE_OBJECT(NullspaceProjectionTranspose,
                TaskSpaceQuantity<Matrix>);
        public:

            NullspaceProjectionTranspose()
                : TaskSpaceQuantity<Matrix>("nullspaceTranspose")
            {
            }

            const NullspaceProjection& transpose() const;


            const NullspaceProjection& operator~() const
            {
                return transpose();
            }
            Vector multiplyByNT(const State& s, const Vector& vec) const;

        private:

            void updateCache(const State& s, Matrix& cache) const override;
        };

        /// @name Access to TaskSpaceQuantity's.
        /// @{
        TASKSPACEQUANTITY_MEMBERS(Jacobian, jacobian, J);
        TASKSPACEQUANTITY_MEMBERS(JacobianTranspose, jacobianTranspose, JT);
        TASKSPACEQUANTITY_MEMBERS(Inertia, inertia, Lambda);
        TASKSPACEQUANTITY_MEMBERS(InertiaInverse, inertiaInverse, LambdaInv);
        TASKSPACEQUANTITY_MEMBERS(DynamicallyConsistentJacobianInverse, jacobianInverse, JBar);
        TASKSPACEQUANTITY_MEMBERS(DynamicallyConsistentJacobianInverseTranspose, jacobianInverseTranspose, JBarT);
        TASKSPACEQUANTITY_MEMBERS(InertialForces, inertialForces, mu);
        TASKSPACEQUANTITY_MEMBERS(Gravity, gravity, p);
        TASKSPACEQUANTITY_MEMBERS(NullspaceProjection, nullspace, N);
        TASKSPACEQUANTITY_MEMBERS(NullspaceProjectionTranspose, nullspaceTranspose, NT);
        /// @}
    };
} // end namespace

#endif // TASK_SPACE_H
