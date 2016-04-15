"""Subclassing OpenSim classes in python Using SWIG director classes.

"""

import os
import unittest

import opensim as osim
import copy

test_dir = os.path.join(os.path.dirname(os.path.abspath(osim.__file__)),
                        'tests')

class MyAnalysis(osim.Analysis):
    def begin(self, state):
        print("BEGIN!!!!")
        self.test_begin = 61
        return 1
    def step(self, state, stepNumber):
        print("STEP!!!!")
        self.test_step = 51
        return 1
    def end(self, state):
        print("END!!!!")
        self.test_end = 37
        return 1
    def clone(self):
        # this's wrong sinc no cloning is done but seems unrelated to SWIG wiring
        print("CLONING!")
        return self
    def getConcreteClassName(self):
        print("in getConcreteClassName")
        return "MyAnalysis"


@osim.declare_concrete_object
class MyModelComponent(osim.ModelComponent):
    def __init__(self):
        super(MyModelComponent, self).__init__()
    def extendAddToSystem(self, *args, **kwargs):
        print("DEBUG!!!!")
        self.addStateVariable("funstate")
        print("DEBUG????")
        
class TestExtendingClasses(unittest.TestCase):
    def test_analysis_forward_adopt(self):
        # Adopt the analysis and run the ForwardTool.
        model = osim.Model(os.path.join(test_dir, "arm26.osim"))
        state = model.initSystem()
        myanalysis = MyAnalysis()
        myanalysis.setModel(model)
        myanalysis.setName('my_analysis')

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
        print("TOOL RAN SUCCESSFULLY!!!!")
        # Make sure that MyAnalysis was evaluated.
        #assert analysis.test_begin == 61
        #assert analysis.test_step == 51
        #assert analysis.test_end == 37 # TODO fails.

    def test_analysis_clone(self):
        # Adopt the analysis and run the ForwardTool.
        model = osim.Model(os.path.join(test_dir, "arm26.osim"))
        myanalysis = MyAnalysis()
        myanalysis.setName('my_analysis')

        #import copy
        #copy.deepcopy(myanalysis)

        # Add our python subclass to the model.
        model.getAnalysisSet().cloneAndAppend(myanalysis)

        # Simple tests.
        #analysis = model.getAnalysisSet().get(0)

    def test_analysis_concreteclassname(self):
        # Adopt the analysis and run the ForwardTool.
        model = osim.Model(os.path.join(test_dir, "arm26.osim"))
        state = model.initSystem()
        myanalysis = MyAnalysis()
        myanalysis.setName('my_analysis')

        # Add our python subclass to the model.
        model.getAnalysisSet().adoptAndAppend(myanalysis)

        # Simple tests.
        analysis = model.getAnalysisSet().get(0)
        print analysis.getConcreteClassName()
        assert analysis.getConcreteClassName() == 'MyAnalysis'

    def test_analysis_forward_clone(self):
        # Adopt the analysis and run the ForwardTool.
        model = osim.Model(os.path.join(test_dir, "arm26.osim"))
        state = model.initSystem()
        myanalysis = MyAnalysis()
        myanalysis.setName('my_analysis')
        myanalysis.setModel(model);
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

    def test_registerType(self):
        # TODO causes segfault.
        ma = MyAnalysis()
        ma.thisown = False
        osim.OpenSimObject.registerType(ma) #.__disown__())
        a = osim.OpenSimObject.getDefaultInstanceOfType('MyAnalysis')
        assert a.getConcreteClassName() == 'MyAnalysis'

    def test_printToXML(self):
        ma = MyAnalysis()
        ma.printToXML('test_MyAnalysis.xml')

    def test_ModelComponent_addToSystem(self):
        # Adopt the analysis and run the ForwardTool.
        model = osim.Model(os.path.join(test_dir, "arm26.osim"))
        mc = MyModelComponent()
        model.addModelComponent(mc)
        mc.extendAddToSystem()
        state = model.initSystem()

    def test_ModelComponent(self):
        # Adopt the analysis and run the ForwardTool.
        model = osim.Model(os.path.join(test_dir, "arm26.osim"))
        mc = MyModelComponent()
        model.addModelComponent(mc)
        state = model.initSystem()

        # Run tool.
        forward = osim.ForwardTool()
        forward.setModel(model)
        forward.run()




