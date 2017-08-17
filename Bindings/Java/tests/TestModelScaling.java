import org.opensim.modeling.*;
import java.io.IOException;

class TestModelScaling {
  public static void main(String[] args) {

  try {
        ScaleTool scaleTool = null;
        Model unscaledModel = new Model("gait2354_simbody.osim");
        try {
            scaleTool = new ScaleTool("subject01_Setup_Scale.xml");
        } catch (IOException ex) {
            System.exit(-1);
        }
        System.out.println("ScaleTool created!");
        MarkerSet extraMarkerSet = new MarkerSet(unscaledModel, "gait2354_Scale_MarkerSet.xml");
        System.out.println("Extra MarkerSet created!");
        unscaledModel.updateMarkerSet(extraMarkerSet);
        System.out.println("Extra MarkerSet added to model!");
        OpenSimContext context = new OpenSimContext(unscaledModel.initSystem(), unscaledModel);
        MeasurementSet measurementSet = scaleTool.getModelScaler().getMeasurementSet();
        System.out.println("MeasurementSet created!");
        MarkerData measurementTrial = new MarkerData("subject01_static.trc");
        System.out.println("MarkerData for static trial loaded!");
        double scaleFactor = context.computeMeasurementScaleFactor(scaleTool.getModelScaler(), unscaledModel, 
                measurementTrial, measurementSet.get(0));
        System.out.println("Measurement computed !");
        System.out.println("Test finished!");
    }
    catch (IOException ex){
        System.exit(-1);
    }
    System.gc();
  }
}
