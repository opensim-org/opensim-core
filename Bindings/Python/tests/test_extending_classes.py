"""Subclassing OpenSim classes in python Using SWIG director classes.

"""

import os
import unittest

import opensim as osim

test_dir = os.path.join(os.path.dirname(os.path.abspath(osim.__file__)),
                        'tests')

@osim.declare_concrete_object
class MyAnalysis(osim.Analysis):
    def __init__(self):
        super(MyAnalysis, self).__init__()
    def begin(self, state):
        self.test_begin = 61
        return 1
    def step(self, state, stepNumber):
        self.test_step = 51
    def end(self, state):
        self.test_end = 37
        return 1
        
class TestExtendingClasses(unittest.TestCase):
    def test_analysis_forward_adopt(self):
        # Adopt the analysis and run the ForwardTool.
        model = osim.Model(os.path.join(test_dir, "arm26.osim"))
        state = model.initSystem()
        myanalysis = MyAnalysis()

        # Add our python subclass to the model.
        model.getAnalysisSet().adoptAndAppend(myanalysis)

        # Simple tests.
        analysis = model.getAnalysisSet().get(0)
        assert analysis.getConcreteClassName() == 'MyAnalysis'
        analysis.begin(state)

        # Run tool.
        forward = osim.ForwardTool()
        forward.setModel(model)
        forward.run()

        # Make sure that MyAnalysis was evaluated.
        assert analysis.test_begin == 61
        assert analysis.test_step == 51
        #assert analysis.test_end == 37 # TODO fails.

    def test_analysis_forward_adopt(self):
        # Adopt the analysis and run the ForwardTool.
        model = osim.Model(os.path.join(test_dir, "arm26.osim"))
        state = model.initSystem()
        myanalysis = MyAnalysis()

        # Add our python subclass to the model.
        model.getAnalysisSet().cloneAndAppend(myanalysis)

        # Simple tests.
        analysis = model.getAnalysisSet().get(0)
        assert analysis.getConcreteClassName() == 'MyAnalysis'
        analysis.begin(state)

        # Run tool.
        forward = osim.ForwardTool()
        forward.setModel(model)
        forward.run()

        # Make sure that MyAnalysis was evaluated.
        assert analysis.test_begin == 61
        assert analysis.test_step == 51
        #assert analysis.test_end == 37 # TODO fails.
