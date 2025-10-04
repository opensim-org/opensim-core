import opensim as osim

# This script serves as a proof-of-concept to verify that the core OpenSim
# tools can be instantiated from Python. It loads a model and creates instances
# of the Scale, Inverse Kinematics, and Inverse Dynamics tools.

def main():
    try:
        # Define the path to the model file
        model_path = "/home/jules/opensim-workspace/opensim-core-source/Bindings/Python/tutorials/resources/Tutorial 5/gait2354_simbody.osim"

        # 1. Load the model
        print(f"Loading model from: {model_path}")
        model = osim.Model(model_path)
        print(f"Successfully loaded model: {model.getName()}")

        # 2. Instantiate the ScaleTool
        scale_tool = osim.ScaleTool()
        scale_tool.setName("MyScaleTool")
        print(f"Successfully instantiated: {scale_tool.getName()}")

        # 3. Instantiate the InverseKinematicsTool
        ik_tool = osim.InverseKinematicsTool()
        ik_tool.setName("MyIKTool")
        print(f"Successfully instantiated: {ik_tool.getName()}")

        # 4. Instantiate the InverseDynamicsTool
        id_tool = osim.InverseDynamicsTool()
        id_tool.setName("MyIDTool")
        print(f"Successfully instantiated: {id_tool.getName()}")

        print("\nProof-of-concept script completed successfully!")

    except Exception as e:
        print(f"An error occurred: {e}")

if __name__ == "__main__":
    main()