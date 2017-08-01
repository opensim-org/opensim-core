API Guide
=========
[TOC]

# Intended Audience {#intendedaudience}
This guide is recommended for those who desire in-depth knowledge of musculoskeletal modeling and simulation with %OpenSim and wish to use the Application Programming Interface (API) to create or contribute novel models or algorithms not currently supported by the OpenSim application. The guide also expects that you are familiar with multibody dynamics and biomechanics.

The %OpenSim software has a variety of interfaces: GUI, command line, XML, Python scripting, MATLAB scripting, and C++. This guide is designed for developers using the last 3 of these interfaces. Programming experience in C++ will be helpful in understanding the API architecture, but we have tried to make this guide accessible to those with only Matlab or Python experience, as well.

# Overview of the OpenSim API for Developers {#overviewoftheapi}

## OpenSim’s Purpose: {#opensimpurpose}
The purpose of OpenSim is to provide anyone interested in human or animal movement with high-performance software to create computer models of the neuromusculoskeletal system, analyze experimental movement data, and simulate movement dynamics to gain insights about the anatomy, physiology, mechanics and control of movement. The software is used in a variety of applications, including neuroscience research, biomechanical analysis, surgical design, and ergonomic analysis, and bioengineering education, just to name a few.  Our goal is to provide physically accurate models and tools that provide a solid foundation for scientific inquiry.

The OpenSim Application Programmer’s Interface (OpenSim API) serves both the OpenSim application, with its graphical user interface (GUI), and enables advanced users and developers to write their own programs to extend the capabilities of OpenSim, including customized analyses and workflows, Components, and OpenSim plug-ins.

## Design Philosophy: {#designphilosophy}
The ethos of the OpenSim API is that the code is modular, reusable, and easily extensible. To enable the assembly of neuromusculoskeletal systems from constituent biomechanical elements, OpenSim has adopted the [composite design pattern]( /en.wikipedia.org/wiki/Composite_pattern) to systematically define, organize, and manage the elements (components) of a neuromusculoskeletal system.

### A Composite Model Framework for Building a Computational System
In OpenSim, a Component is a computational model element that describes some physical phenomenon. As a neuromusculoskeletal simulator, OpenSim components represent bodies, joints, muscles, and other physical structures, as well as sensors, controllers, and feedback circuits that describe human and animal dynamics. The purpose of a Component is to both capture the physical phenomenon (degrees-of-freedom, constraints, actuation, control) of interest and to enable the systematic composition of complex behaviors (dynamical models) from simpler components.

To illustrate this point, think of two biomechanists, Gary Gait and Norma Knee. Gary is interested in the actions of individual muscles during gait and other forms of locomotion. Norma wants to understand how loads are distributed within the knee and their effect on cartilage wear and health. For Gary, a model represents a patient’s musculoskeletal dynamics and is capable of muscle-driven gait that reproduces human-like performance. In Norma’s case, a model consists of a femur, a tibia and a patella, contact between articulating surfaces, tensile ligaments and muscles, so that she can compute contact forces. Gary and Norma’s “models” may have similar complexity but their concepts of a model are fundamentally different (i.e., whole body vs. a knee). Gary may model knees as pin joints and attain reasonable answers about the action of leg muscles during different types of gait. Norma cannot justify the same simplification. A more accurate knee model may result in more realistic muscle forces for Gary, and simulated muscle forces (that reproduce human gait) may provide better boundary conditions for Norma’s estimates of knee loads. However, for these researchers to benefit directly from each other’s work, their models must be interoperable. In order for their models to talk to one another and be systematically combined, both models must have constructs(s) in common. This modularity and interoperability is the main purpose of the Component API.

![Figure 1](figures/OverviewFigure1.png "Figure 1")
*Figure 1. Overview of the OpenSim Modeling Framework. Aspects of the musculoskeletal system (Physical System) such as bones, joints, and muscles, and its neurocontrol composed of spinal circuits, muscle spindle and Golgi tendon organs are represented as Components in the Modeling Framework. The specification of the types and arrangement of Components in a Model reflect the physical system being modeled and it is the main task of the Modeler. From the assembly of Components (the Model) the corresponding computational System (system of equations readily solved by a computer) is automatically created and ready for numerical simulation and analysis. All System unknowns (variables) and their values are in the State.*


# New Section {#newsection}
This is how we can refer to a class without having to use the OpenSim
namespace: [Joint](@ref OpenSim::Joint)

This page is under construction.


# List of subpages {#listofsubpages}

[Frames](Frames.md)
 - @subpage frames
