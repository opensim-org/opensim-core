import org.opensim.modeling.*;

class TestIKInterfaces {
    public static void test_SolverConstuctor() {
        Model model = new Model();
        MarkersReference markersRef = new MarkersReference();
        SimTKArrayCoordinateReference coordinateRef = new SimTKArrayCoordinateReference();
        // Create the objects and hold reference so they don't get deleted/garbage_collected
        // before the call to the constructor returns.
        InverseKinematicsSolver solver = new InverseKinematicsSolver(
            model, markersRef, coordinateRef);
    }

    public static void main(String[] args) {
        test_SolverConstuctor();
    }
};
