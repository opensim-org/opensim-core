"""
Kinematics method classes.

The dynamic model should inherit from this class in order to get the proper kinematics.
"""

import opensim as osim


class KinematicModel:
    def __init__(self, model, mot, filter_param=None):

        # load model
        self.model = osim.Model(model)
        self.state = self.model.initSystem()

        # prepare data
        self.actual_q, self.actual_qdot, self.actual_qddot = [], [], []

        # get some reference that will be modified during the optimization (for speed sake)
        self.actuators = self.model.getForceSet()
        self.muscle_actuators = self.model.getMuscles()
        self.n_dof = self.state.getQ().size()
        self.n_actuators = self.actuators.getSize()
        self.n_muscles = self.muscle_actuators.getSize()

        self._data_storage = osim.Storage(mot)
        self.filter_param = filter_param

        self.__dispatch_kinematics()

    def get_time(self, frame):
        return self._data_storage.getStateVector(frame).getTime()

    def __dispatch_kinematics(self):
        self.model.getSimbodyEngine().convertDegreesToRadians(self._data_storage)

        if self.filter_param:
            # TODO: read value from xml
            self._data_storage.lowpassFIR(self.filter_param[0], self.filter_param[1])

        # TODO: read value from xml
        self.gen_coord_function = osim.GCVSplineSet(5, self._data_storage)
        self.n_frame = self._data_storage.getSize()

        self.all_q, self.all_qdot, self.all_qddot = [], [], []
        for iframe in range(self.n_frame):
            q, qdot, qddot = [], [], []
            for iq in range(self._data_storage.getStateVector(iframe).getSize()):
                q.append(
                    self.gen_coord_function.evaluate(
                        iq, 0, self._data_storage.getStateVector(iframe).getTime()
                    )
                )
                qdot.append(
                    self.gen_coord_function.evaluate(
                        iq, 1, self._data_storage.getStateVector(iframe).getTime()
                    )
                )
                qddot.append(
                    self.gen_coord_function.evaluate(
                        iq, 2, self._data_storage.getStateVector(iframe).getTime()
                    )
                )

            self.all_q.append(q)
            self.all_qdot.append(qdot)
            self.all_qddot.append(qddot)

            self.upd_model_kinematics(frame=0)

    def upd_model_kinematics(self, frame):
        # get a fresh state
        self.model.initStateWithoutRecreatingSystem(self.state)
        self.state.setTime(self.get_time(frame))

        # update kinematic states
        self.actual_q = self.all_q[frame]
        self.actual_qdot = self.all_qdot[frame]
        self.actual_qddot = self.all_qddot[frame]

        self.state.setQ(osim.Vector(self.actual_q))
        self.state.setU(osim.Vector(self.actual_qdot))
        self.model.realizeVelocity(self.state)
