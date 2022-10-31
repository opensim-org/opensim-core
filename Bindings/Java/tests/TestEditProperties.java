import org.opensim.modeling.*;
import java.io.IOException;
import static java.nio.file.StandardOpenOption.*;
import java.nio.file.*;

class TestEditProperties {
  public static void main(String[] args) {
    try {
        Model model = new Model("gait10dof18musc_subject01.osim");
        OpenSimContext context=null;
        long t1  = System.currentTimeMillis();
        try {
            context = new OpenSimContext(model.initSystem(), model);
        } catch (IOException ex) {
            ex.printStackTrace();
            System.out.println("Failed to initialize Context");
            System.exit(-1);
        }
        long t2  = System.currentTimeMillis();
        System.out.println("Model loading and initialization time (ms):"+(t2 - t1));
        Object gnd = model.getGround();
        Object gndValue = getValue(gnd);
        Body bdy = model.getBodySet().get("pelvis");
        AbstractProperty massProp = bdy.getPropertyByName("mass");
        Object massValue = getValue(massProp);
        // Now perform edits
        for (int ed=0; ed <10; ed++){
                double oldValue = PropertyHelper.getValueDouble(massProp);
                double v = oldValue+1.0;
                context.cacheModelAndState();
                PropertyHelper.setValueDouble(v, massProp);        
                try {
                    context.restoreStateFromCachedModel();
                } catch (IOException iae) {
                    PropertyHelper.setValueDouble(oldValue, massProp);
                    context.restoreStateFromCachedModel(); 
                    System.out.println("Failed to restore on failed edit");
                    System.exit(-1);
                } 
        }
        // Now edit a Vec6 Property (inertia)
        AbstractProperty inertiaProp = bdy.getPropertyByName("inertia");
        // This block populates a stock Vec6 from a prestored inertia property
        double[] inertiaFromFile = new double[]{0.10423369488353, 0.08831473564548, 0.05870749935560, 0, 0, 0};
        Vec6 oldInertia = new Vec6();
        for(int i=0; i<6; i++){
            oldInertia.set(i, PropertyHelper.getValueVec6(inertiaProp, i));
            System.out.println(oldInertia.get(i));
            assert Math.abs(inertiaFromFile[i] - oldInertia.get(i)) < 1e-4;
        }
        // This block sets inertia to [0,1,2,3,4,5]
        for(int i=0; i<6; i++){
            // Bad inertia matrix but we're just testing set/get'
            PropertyHelper.setValueVec6(i, inertiaProp, i); 
        }
        // This block makes sure we can successfully retrieve what we set.
        Vec6 newInertia = new Vec6();
        for(int i=0; i<6; i++){
            double ret = PropertyHelper.getValueVec6(inertiaProp, i);
            newInertia.set(i, ret);
            System.out.println(newInertia.get(i));
            assert newInertia.get(i)==i;
        }
        long t3  = System.currentTimeMillis();
        System.out.println("Time to perform 10 edits (ms):"+(t3 - t2));
        // new style properties which are lists we test append and set 
        // Property class has no interface to remove/delete/shrink the list!
        OutputReporter outputReporter = new OutputReporter();
        AbstractProperty pathsList = outputReporter.getPropertyByName("output_paths");
        PropertyStringList propertyStringList = PropertyStringList.getAs(pathsList);
        propertyStringList.appendValue("Output1");
        propertyStringList.appendValue("Output2");
        assert propertyStringList.findIndex("Output2")==1;
        assert propertyStringList.size()==2;
        System.out.println(propertyStringList.toString());
        propertyStringList.setValue(0, "Output0");
        System.out.println(propertyStringList.toString());
        System.out.println("Test finished!");
        assert propertyStringList.getValue(0).equals("Output0");
        System.exit(0);
    }
    catch(IOException ex){
        System.out.println("Failed to load model or in initSystem");
        System.exit(-1);
    }
    // TODO to cause test to fail: System.exit(-1);
  }

  static public Object getValue(Object propertyOrObject) {
      int idx = -1;
      if (propertyOrObject instanceof OpenSimObject)
        return ( (OpenSimObject) propertyOrObject).getName();
      else {
         AbstractProperty ap = (AbstractProperty)propertyOrObject;
         String propValueType = ap.getTypeName();
         //System.out.println("Processing property ["+ap.getName()+ "] type = ["+propValueType+"] index ="+idx);
         if (ap.isListProperty() && idx==-1 || ap.isOptionalProperty() && idx==-1)
             return ap.toString();
         if(propValueType.equalsIgnoreCase("double")) { 
            return PropertyHelper.getValueDouble(ap, idx);
         } else if(propValueType.equalsIgnoreCase("int")) {
            return PropertyHelper.getValueInt(ap, idx); 
         } else if(propValueType.equalsIgnoreCase("string")) {
            return PropertyHelper.getValueString(ap, idx);  
         } else if(propValueType.equalsIgnoreCase("bool")) {
            return PropertyHelper.getValueBool(ap, idx);
         } else if(propValueType.equalsIgnoreCase("Transform")) {
             return PropertyHelper.getValueTransform(ap, idx);
         } else if(propValueType.equalsIgnoreCase("Object")) {
            return ap.getValueAsObject();
         } else 
             return ap.toString();
      }
    }
}
