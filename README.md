# OpenSim for Powerlifting Analysis

This is a fork of the [opensim-core](https://github.com/opensim-org/opensim-core) repository, tailored to serve as a backend for a React application that analyzes the biomechanics of the squat, bench press, and deadlift.

## Purpose

The goal of this project is to create a lightweight, special-purpose version of OpenSim that can be deployed as a backend service. This service will take user-provided data (e.g., motion capture, video analysis) and provide feedback on their lifting technique.

## Getting Started

This repository is under active development. The immediate next steps are to:

1.  Explore the Python API to understand how to programmatically control OpenSim.
2.  Identify the essential C++ components required for analyzing the three powerlifts.
3.  Develop a proof-of-concept Python script that can load a model and perform a basic analysis.

This `README.md` will be updated as the project progresses. For information about the original OpenSim project, see [`README.opensim.md`](README.opensim.md).

## Python API Workflow

Based on the examples and tutorials, the primary workflow for analyzing a movement using the Python API is as follows:

1.  **Model Scaling**: A generic musculoskeletal model (`.osim` file) is scaled to match the anthropometry of a specific subject. This step uses the `osim.ScaleTool` and requires a static motion capture file (`.trc`) of the subject.

2.  **Inverse Kinematics (IK)**: Using the scaled model and a dynamic motion capture file (`.trc`) of the movement (e.g., a squat), the `osim.InverseKinematicsTool` calculates the joint angles for each frame of the motion. The results are typically saved to a `.mot` file.

3.  **Inverse Dynamics (ID)**: The `osim.InverseDynamicsTool` takes the joint angles from the IK step and any external force data (e.g., from force plates) to compute the net joint forces and torques throughout the movement. The results are saved to a `.sto` file.

This pipeline provides the core data needed to analyze a lift, which can then be processed and sent to the frontend.