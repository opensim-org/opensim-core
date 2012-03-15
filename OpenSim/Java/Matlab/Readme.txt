README.TXT

Setting up Matlab to access the Java wrapper to Opensim -

1. Edit the classpath.txt file (in command window type 'edit classpath.txt')
   Add the following path to the end of the file ensuring the correct path
   to the opensim jar file -
   C:\OpenSim3.0\opensim\modules\org-opensim-modeling.jar
   Save the new file to your Matlab startup directory

2. Edit the librarypath.txt file (in command window type 'edit librarypath.txt')
   Add the following path to the end of the file ensuring the correct path
   to the DLL files in the opensim bin - 
   C:\OpenSim3.0\bin
   Save the new file to your Matlab startup directory (i.e. the folder that Matlab opens up in)


3. Also (to be safe) add the bin e.g. "C:\OpenSim3.0\bin" directory to the path 
   ('editpath' in command window and add the folder and save)

4. Close and restart Matlab

5. Test that everything is correct - 
   At command window type "model = org.opensim.modeling.Model();"
   If model object is created then you are ready to go.

6. Tips - 
   To prevent typing org.opensim.modeling.'Class' everytime, you can 
   import all classes using 'import org.opensim.modeling.*'
   To examine the methods of a class type 'methodsview('Model')' or 
   'methodsview(osimModel)' where osimModel is a model object already created
   See the Doxygen documentation for description of many classes available 
   in the Opensim API.

7. Examples in this directory -

	1. OpenSimCreateTugOfWarModel.m - Script to perform the same model building and
		simulation tasks as the MainExample from the SDK examples
		
		Type - ' OpenSimCreateTugOfWarModel' at the Matlab command line (generates OSIM file)
	2. TugOfWar_CompleteRunVisualize - Script to perform forward dynamic simulation using
		the TugOfWar_Complete.osim model.
		Type - 'TugOfWar_CompleteRunVisualize' at the Matlab command line (generates states file_
	3. strengthScalar - Test function to load muscles and change strength of muscles 
		and re-save model with new muscle strengths.
	4. setupAndRunIKBatchExample - Batch processing 2 ik trials, model and data files in testData folder
