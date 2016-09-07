# ----------------------------------------------------------------------- #
# The OpenSim API is a toolkit for musculoskeletal modeling and           #
# simulation. See http://opensim.stanford.edu and the NOTICE file         #
# for more information. OpenSim is developed at Stanford University       #
# and supported by the US National Institutes of Health (U54 GM072970,    #
# R24 HD065690) and by DARPA through the Warrior Web program.             #
#                                                                         #
# Copyright (c) 2005-2016 Stanford University and the Authors             #
# Author(s): Neil Dhir                                                    #
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

# simple-arm.py
# Author: Neil Dhir
# ------------------------------------------------------------------------#
# ABSTRACT: This short piece of OpenSim python API example demonstrates a #
# simple arm which consists of two bodies, two joints, a muscle and a     #
# controller. All model elements are labeled with their appropriate       #
# biomechanical namesakes for easy identification and clarity of          #
# demonstration. Further, the model does not include forward simulation,  #
# but instead saves the model to an .osim file which can be used in the   #
# OpenSim plotter or the graphics window.                                 #
# ------------------------------------------------------------------------#

import opensim as osim

# Define global model where the arm lives.
arm = osim.Model()

# ---------------------------------------------------------------------------
# Create two links, each with a mass of 1 kg, centre of mass at the body's
# origin, and moments and products of inertia of zero.
# ---------------------------------------------------------------------------

humerus = osim.Body('humerus',
                    1.0,
                    osim.Vec3(0, 0, 0),
                    osim.Inertia(0, 0, 0))
radius = osim.Body('radius',
                   1.0,
                   osim.Vec3(0, 0, 0),
                   osim.Inertia(0, 0, 0))

# ---------------------------------------------------------------------------
# Connect the bodies with pin joints. Assume each body is 1m long.
# ---------------------------------------------------------------------------

shoulder = osim.PinJoint("shoulder",
                         arm.getGround(), # PhysicalFrame
                         osim.Vec3(0, 0, 0),
                         osim.Vec3(0, 0, 0),
                         humerus, # PhysicalFrame
                         osim.Vec3(0, 0, 0),
                         osim.Vec3(0, 1, 0))

elbow = osim.PinJoint("elbow",
                      humerus, # PhysicalFrame
                      osim.Vec3(0, 0, 0),
                      osim.Vec3(0, 0, 0),
                      radius, # PhysicalFrame
                      osim.Vec3(0, 0, 0),
                      osim.Vec3(0, 1, 0))

# ---------------------------------------------------------------------------
# Add a muscle that flexes the elbow (actuator for robotics people).
# ---------------------------------------------------------------------------

biceps = osim.Millard2012AccelerationMuscle("biceps",  # Muscle name
                                            200.0,  # Max isometric force
                                            0.6,  # Optimal fibre length
                                            0.55,  # Tendon slack length
                                            0.0)  # Pennation angle
biceps.addNewPathPoint("origin",
                       humerus,
                       osim.Vec3(0, 0.8, 0))

biceps.addNewPathPoint("insertion",
                       radius,
                       osim.Vec3(0, 0.7, 0))

# ---------------------------------------------------------------------------
# Add a controller that specifies the excitation of the muscle.
# ---------------------------------------------------------------------------

brain = osim.PrescribedController()
brain.addActuator(biceps)
brain.prescribeControlForActuator('biceps',  # Actuator's index in controller set
                                  osim.StepFunction(0.5, 3.0, 0.3, 1.0))

# ---------------------------------------------------------------------------
# Build model with components created above.
# ---------------------------------------------------------------------------

arm.addBody(humerus)
arm.addBody(radius)
arm.addJoint(shoulder) # Now required in OpenSim4.0
arm.addJoint(elbow)
arm.addForce(biceps)
arm.addController(brain)

# ---------------------------------------------------------------------------
# Add a console reporter to print the muscle fibre force and elbow angle.
# ---------------------------------------------------------------------------

# We want to write our simulation to a file in the end.
reporter = osim.TableReporter()
reporter.set_report_time_interval(1.0)
reporter.updInput("inputs").connect(biceps.getOutput("fiber_force"))
elbow_cord = elbow.get_coordinates(0).getOutput("value")
reporter.updInput("inputs").connect(elbow_cord, "elbow_angle")
arm.addComponent(reporter)

# ---------------------------------------------------------------------------
# Configure the model.
# ---------------------------------------------------------------------------

state = arm.initSystem()
# Fix the shoulder at its default angle and begin with the elbow flexed.
arm.upd_coordinates(0).setLocked(state, True)
arm.upd_coordinates(1).setValue(state, 0.5 * osim.SimTK_PI)
arm.equilibrateMuscles(state)

# ---------------------------------------------------------------------------
# Print/save model file
# ---------------------------------------------------------------------------

arm.printToXML("SimpleArm.osim")
