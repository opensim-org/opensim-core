# ----------------------------------------------------------------------- #
# The OpenSim API is a toolkit for musculoskeletal modeling and           #
# simulation. See http://opensim.stanford.edu and the NOTICE file         #
# for more information. OpenSim is developed at Stanford University       #
# and supported by the US National Institutes of Health (U54 GM072970,    #
# R24 HD065690) and by DARPA through the Warrior Web program.             #
#                                                                         #
# Copyright (c) 2005-2017 Stanford University and the Authors             #
# Author(s): Christopher Dembia                                           #
#                                                                         #
# Licensed under the Apache License, Version 2.0 (the "License");         #
# you may not use this file except in compliance with the License.        #
# You may obtain a copy of the License at                                 #
# http://www.apache.org/licenses/LICENSE-2.0.                             #
#                                                                         #
# Unless required by applicable law or agreed to in writing, software     #
# distributed under the License is distributed on an "AS IS" BASIS,       #
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or         #
# implied. See the License for the specific language governing            #
# permissions and limitations under the License.                          #
# ----------------------------------------------------------------------- #

# This example shows how to wire inputs and outputs by reporting the position
# of the system's center of mass. We also illustrate that input-output
# connections are stored in model (.osim) files.
# The model contains just one body, a free joint, and the table reporter.

import opensim as osim


model_filename = 'wiring_inputs_and_outputs_with_TableReporter.osim'

# This function creates and prints the model to a .osim file. We invoke
# this function below.
def print_model():
    model = osim.Model()
    model.setName('model')
    
    # Create a body with name 'body', mass of 1 kg, center of mass at the
    # origin of the body, and unit inertia (Ixx = Iyy = Izz = 1 kg-m^2).
    body = osim.Body('body', 1.0, osim.Vec3(0), osim.Inertia(1))
    
    # Create a free joint (all 6 degrees of freedom) with Ground as the parent
    # body and 'body' as the child body.
    joint = osim.FreeJoint('joint', model.getGround(), body)
    
    # Add the body and joint to the model.
    model.addComponent(body)
    model.addComponent(joint)
    
    # Create a TableReporter to save quantities to a file after simulating.
    reporter = osim.TableReporterVec3()
    reporter.setName('reporter')
    reporter.set_report_time_interval(0.1)
    # Report the position of the origin of the body.
    reporter.addToReport(body.getOutput('position'))
    # For comparison, we will also get the center of mass position from the
    # Model, and we can check that the two outputs are the same for our
    # one-body system. The (optional) second argument is an alias for the name
    # of the output; it is used as the column label in the table.
    reporter.addToReport(model.getOutput('com_position'), 'com_pos')

    model.addComponent(reporter)
    model.finalizeConnections()

    # Display what input-output connections look like in XML (in .osim files).
    print("Reporter input-output connections in XML:\n" + reporter.dump())

    model.printToXML(model_filename)


# Create and print the model file.
print_model()
# Load the model file.
deserialized_model = osim.Model(model_filename)
state = deserialized_model.initSystem()

# We can fetch the TableReporter from within the deserialized model.
reporter = osim.TableReporterVec3.safeDownCast(
        deserialized_model.getComponent('reporter'))
# We can access the names of the outputs that the reporter is connected to.
print('Outputs connected to the reporter:')
for i in range(reporter.getInput('inputs').getNumConnectees()):
    print(reporter.getInput('inputs').getConnecteePath(i))

# Simulate the model.
manager = osim.Manager(deserialized_model)
state.setTime(0)
manager.initialize(state)
state = manager.integrate(1.0)

# Now that the simulation is done, get the table from the TableReporter and
# write it to a file.
# This returns the TimeSeriesTableVec3 that holds the history of positions.
table = reporter.getTable()
# Create a FileAdapter, which handles writing to (and reading from) .sto files.
sto = osim.STOFileAdapterVec3()
sto.write(table, 'wiring_inputs_and_outputs_with_TableReporter.sto')
# You can open the .sto file in a text editor and see that both outputs
# (position of body's origin, and position of system mass center) are the same.








