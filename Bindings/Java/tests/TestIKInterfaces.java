import org.opensim.modeling.*;
import java.io.IOException;

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
    static void test_MarkerWeightSet() {
        try {
            Model model= new Model("gait10dof18musc_subject01.osim");
            model.initSystem();
            SetMarkerWeights markerWeightSet = new SetMarkerWeights();
            for (int i=0; i< model.getMarkerSet().getSize(); i++)
                markerWeightSet.cloneAndAppend(new 
                    MarkerWeight(model.getMarkerSet().get(i).getName(), 1.0));
        }
        catch (IOException ex) {
            System.exit(1);
        }
    }
    public static void main(String[] args) {
        test_SolverConstuctor();
        test_MarkerWeightSet();
    }
};
