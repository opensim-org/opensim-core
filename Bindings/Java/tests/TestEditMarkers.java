import org.opensim.modeling.*;
import java.io.IOException;
import static java.nio.file.StandardOpenOption.*;
import java.nio.file.*;

class TestEditMarkers {
  public static void main(String[] args) {
    try {
        // GUI model loading
        Model model = new Model("gait10dof18musc_subject01.osim");
        OpenSimContext context=null;
        context = new OpenSimContext(model.initSystem(), model);
        MarkerSet markerset = model.getMarkerSet();
        // The following code exercises the Marker->Add New  functionality
        Vec3 offset = new Vec3(0.11, 0.22, 0.33);
        Body body = model.getBodySet().get(0);
        String newMarkerName = "newMarker";
        Marker newMarker = new Marker();
        newMarker.setName(newMarkerName);
        newMarker.set_location(offset);
        newMarker.setParentFrame(body);
        context.cacheModelAndState();
        model.addMarker(newMarker);
        try {
            context.restoreStateFromCachedModel();
        } catch (IOException ex) {
            System.exit(1);
        }
        // This exercises Marker -> delete
        context.cacheModelAndState();
        
        Marker findMarker = markerset.get(newMarkerName);
        assert (findMarker != null);
        markerset.remove(findMarker);
        try {
            context.restoreStateFromCachedModel();
        } catch (IOException ex) {
            System.exit(1);
        }
		// Exercise saveToFile
        markerset.print("savedMarkers.xml");
        // Now create a new MarkerSet from the file
        MarkerSet newMarkerSet = new MarkerSet("savedMarkers.xml");
        Marker newMarkerRenamed = newMarkerSet.get(0);
        String addedMarkerName = newMarkerRenamed.getName()+"_renamed";
        newMarkerRenamed.setName(addedMarkerName);
        context.cacheModelAndState();
        markerset.adoptAndAppend(newMarkerRenamed);
        try {
            context.restoreStateFromCachedModel();
        } catch (IOException ex) {
            System.exit(1);
        }
        Marker findMarkerAdded = markerset.get(addedMarkerName);
        assert(findMarkerAdded != null);
        System.exit(0);
    }
    catch(IOException ex){
        System.out.println("Failed to load model or in initSystem");
        System.exit(-1);
    }
    // TODO to cause test to fail: System.exit(-1);
  }

}
