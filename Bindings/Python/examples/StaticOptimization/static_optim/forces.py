"""
Forces method classes.

The dynamic model should inherit from these classes in order to get the proper forces.
"""
import opensim as osim


class ResidualForces:
    def __init__(self, residual_actuator_xml=None):

        if residual_actuator_xml:
            force_set = osim.ArrayStr()
            force_set.append(residual_actuator_xml)

            analyze_tool = osim.AnalyzeTool(self.model)
            analyze_tool.setModel(self.model)
            analyze_tool.setForceSetFiles(force_set)
            analyze_tool.updateModelForces(self.model, residual_actuator_xml)
            self.state = self.model.initSystem()

            fs = self.model.getForceSet()
            for i in range(fs.getSize()):
                act = osim.CoordinateActuator.safeDownCast(fs.get(i))
                if act:
                    act.overrideActuation(self.state, True)


class ExternalForces:
    def __init__(self, external_load_xml=None):
        if external_load_xml:
            analyze_tool = osim.AnalyzeTool(self.model)
            analyze_tool.setModel(self.model)
            analyze_tool.setExternalLoadsFileName(external_load_xml)
