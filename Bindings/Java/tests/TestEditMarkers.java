import org.opensim.modeling.*;
import java.io.IOException;
import static java.nio.file.StandardOpenOption.*;
import java.nio.file.*;

class TestEditMarkers {
  public static void main(String[] args) {
    try {
        Model model = new Model("gait10dof18musc_subject01.osim");
        OpenSimContext context=null;
        long t1  = System.currentTimeMillis();
        try {
            context = new OpenSimContext(model.initSystem(), model);
            context.cacheModelAndState();
            MarkerSet markerset = model.getMarkerSet();
            // The following code exercises the Marker->Add New  functionality
            Vec3 offset = new Vec3(0.11, 0.22, 0.33);
            Body body = model.getBodySet().get(0);
            String newMarkerName = "newMarker";
            Marker marker = markerset.addMarker(newMarkerName, offset, body);
            // This exercises Marker -> delete
            //markerset.remove(marker);
            context.restoreStateFromCachedModel();
        } catch (IOException ex) {
            ex.printStackTrace();
            System.out.println("Failed to initialize Context");
            System.exit(-1);
        }
        System.exit(0);
    }
    catch(IOException ex){
        System.out.println("Failed to load model or in initSystem");
        System.exit(-1);
    }
    // TODO to cause test to fail: System.exit(-1);
  }

}
