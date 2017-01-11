"""The tests here ensure that the Python and Java/Jython scripting are
consistent with each other. The initial tests here come from
pre-existing Jython scripting code.

"""

import os
import unittest

import opensim as osim

test_dir = os.path.join(os.path.dirname(os.path.abspath(osim.__file__)),
                        'tests')

# Silence warning messages if mesh (.vtp) files cannot be found.
osim.Model.setDebugLevel(0)

def assert_almost_equal(valA, valB, tol=1e-5):
    assert abs(valA - valB) < tol

class TestConsistentWithJython(unittest.TestCase):
    def test_makeUlnaHeavy(self):
    
        # Get handle to the Arm26 model
        oldModel = osim.Model(os.path.join(test_dir, "arm26.osim"))
        # Create a fresh copy
        myModel = oldModel.clone()
    
        # Initialize the copy, if values need to be set in the model's state
        # pass along the variable myState  returned by initSystem
        myState = myModel.initSystem()
    
        # Name the copy for later display in the GUI
        oldName = oldModel.getName()
        myModel.setName(oldName + "_heavier")
    
        # A scale factor for mass of forarm
        massScale = 3.0
    
        # Change mass of forarm in the model
        forearm = myModel.getBodySet().get("r_ulna_radius_hand")
        newMass = forearm.getMass() * massScale
        forearm.setMass(newMass)
    
        # Get absolute path name of original.old model
        absPathName = oldModel.getInputFileName()
    
        # Change the name of the modified model
        newName = os.path.join(test_dir, 'Arm26_makeUlnaHeavy.osim')
        myModel.printToXML(newName)
    
        deserMyModel = osim.Model(newName)
        assert_almost_equal(
                deserMyModel.getBodySet().get("r_ulna_radius_hand").getMass(),
                newMass)
    
        os.remove(newName)
    
    
    def test_alterTendonSlackLength(self):
    
        oldModel = osim.Model(os.path.join(test_dir, "arm26.osim"))
    
        # Create a fresh copy
        myModel = oldModel.clone()
    
        # Initialize the copy, if values needed to be set in state
        # pass along the variable myState returned by initSystem
        myState = myModel.initSystem()
    
        # Name the copy for later display in the GUI
        oldName = oldModel.getName()
        myModel.setName(oldName + "_longerTSL")
    
        # A scale factor for tendonSlackLength of muscles
        tendonSlackLengthScale = 1.5
    
        # Change TendonSlackLength for all muscles in the model
        for i in range(myModel.getMuscles().getSize()):
            currentMuscle = myModel.getMuscles().get(i)
            oldSL = currentMuscle.getTendonSlackLength()
            currentMuscle.setTendonSlackLength(oldSL * tendonSlackLengthScale)
    
        #get absolute path name of original model
        absPathName = oldModel.getInputFileName()
    
        #Change pathname to output file name
        newName = os.path.join(test_dir, 'Arm26_alterTendonSlackLength.osim')
        myModel.printToXML(newName)
    
        deserMyModel = osim.Model(newName)
        for i in range(deserMyModel.getMuscles().getSize()):
            oldMuscle = oldModel.getMuscles().get(i)
            oldSL = oldMuscle.getTendonSlackLength()
            newMuscle = deserMyModel.getMuscles().get(i)
            assert_almost_equal(newMuscle.getTendonSlackLength(),
                    oldSL * tendonSlackLengthScale)
    
        os.remove(newName)
    
    
    def test_strengthenModel(self):
    
        oldModel = osim.Model(os.path.join(test_dir, "arm26.osim"))
    
        # Create a fresh copy
        myModel = oldModel.clone()
    
        # Initialize the copy
        myModel.initSystem()
    
        # Name the copy for later showing in GUI
        oldName = oldModel.getName()
        myModel.setName(oldName+"_Stronger")
    
        # Define a scale factor for MaxIsometricForce of muscles
        scaleFactor = 1.2
    
        # Apply scale factor to MaxIsometricForce
        for i in range(myModel.getMuscles().getSize()):
            currentMuscle = myModel.getMuscles().get(i)
            currentMuscle.setMaxIsometricForce(currentMuscle.getMaxIsometricForce()
                    * scaleFactor)
    
        # Save resulting model
        newName = os.path.join(test_dir, 'Arm26_strengthenModel.osim')
        myModel.printToXML(newName)
    
        deserMyModel = osim.Model(newName)
        for i in range(deserMyModel.getMuscles().getSize()):
            oldMuscle = oldModel.getMuscles().get(i)
            newMuscle = deserMyModel.getMuscles().get(i)
            assert_almost_equal(newMuscle.getMaxIsometricForce(),
                    oldMuscle.getMaxIsometricForce() * scaleFactor)
    
        os.remove(newName)
    
    def test_StorageToPieceWiseLinearFunction(self):
    
        sto = osim.Storage(os.path.join(test_dir, 'storage.sto'))
        column_name = 'column1'
        scale_factor = 2.5
    
        time = osim.ArrayDouble()
        sto.getTimeColumn(time)
    
        state_index = sto.getStateIndex(column_name)
    
        if type(scale_factor) == float:
            sto.multiplyColumn(state_index, scale_factor)
        elif scale_factor == None:
            pass
        else:
            raise Exception('scale_factor, if specified, must be a float.')
    
        ordinate = osim.ArrayDouble()
        sto.getDataColumn(state_index, ordinate)
    
        fcn = osim.PiecewiseLinearFunction()
        for idx in range(time.getSize()):
            fcn.addPoint(time.get(idx), ordinate.get(idx))
    
        fcnName = os.path.join(test_dir, 'piecewiseLinearFunction.xml')
        fcn.printToXML(fcnName)
    
        assert fcn.getX(0) == 0
        assert_almost_equal(fcn.getX(1), 0.01)
        assert_almost_equal(fcn.getX(2), 0.02)
        assert_almost_equal(fcn.getX(3), 0.03)
        assert_almost_equal(fcn.getX(4), 0.04)
        assert_almost_equal(fcn.getY(0), 3.75)
        assert_almost_equal(fcn.getY(1), 23.025)
        assert_almost_equal(fcn.getY(2), 13.5575)
    
        os.remove(fcnName)
    
    def test_addMetabolicProbes(self):
    
        model = osim.Model(os.path.join(test_dir,
            "gait10dof18musc_subject01.osim"))
    
        # Twitch ratios for gait1018.
        twitchRatios = { 'hamstrings': 0.49, 'bifemsh': 0.53, 'glut_max': 0.55,
                'iliopsoas': 0.50, 'rect_fem': 0.39, 'vasti': 0.50, 'gastroc':
                0.54, 'soleus': 0.80, 'tib_ant': 0.70}
        
        # Parameters used for all probes
        # ------------------------------
        # The following booleans are constructor arguments for the Umberger probe.
        # These settings are used for all probes.
        activationMaintenanceRateOn = True
        shorteningRateOn = True
        basalRateOn = False
        mechanicalWorkRateOn = True
        
        # The mass of each muscle will be calculated using data from the model:
        #   muscleMass = (maxIsometricForce / sigma) * rho * optimalFiberLength
        # where sigma = 0.25e6 is the specific tension of mammalian muscle (in
        # Pascals) and rho = 1059.7 is the density of mammalian muscle (in kg/m^3).
        
        # The slow-twitch ratio used for muscles that either do not appear in the
        # file, or appear but whose proportion of slow-twitch fibers is unknown.
        defaultTwitchRatio = 0.5
        
        # Whole-body probe
        # ----------------
        # Define a whole-body probe that will report the total metabolic energy
        # consumption over the simulation.
        wholeBodyProbe = osim.Umberger2010MuscleMetabolicsProbe(
            activationMaintenanceRateOn,
            shorteningRateOn,
            basalRateOn,
            mechanicalWorkRateOn)
        wholeBodyProbe.setOperation("value")
        wholeBodyProbe.set_report_total_metabolics_only(False);
        
        # Add the probe to the model and provide a name.
        model.addProbe(wholeBodyProbe)
        wholeBodyProbe.setName("metabolic_power")
        
        # Loop through all muscles, adding parameters for each into the whole-body
        # probe.
        for iMuscle in range(model.getMuscles().getSize()):
            thisMuscle = model.getMuscles().get(iMuscle)
            
            # Get the slow-twitch ratio from the data we read earlier. Start with
            # the default value.
            slowTwitchRatio = defaultTwitchRatio
            
            # Set the slow-twitch ratio to the physiological value, if it is known.
            for key, val in twitchRatios.items():
                if thisMuscle.getName().startswith(key) and val != -1:
                    slowTwitchRatio = val
            
            # Add this muscle to the whole-body probe. The arguments are muscle
            # name, slow-twitch ratio, and muscle mass. Note that the muscle mass
            # is ignored unless we set useProvidedMass to True.
            wholeBodyProbe.addMuscle(thisMuscle.getName(),
                                     slowTwitchRatio)
    
        name = os.path.join(test_dir, 'gait10dof18musc_probed.osim')
        model.printToXML(name)
    
        os.remove(name)
