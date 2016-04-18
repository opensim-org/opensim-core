import org.opensim.modeling.*;
import java.io.IOException;
import static java.nio.file.StandardOpenOption.*;
import java.nio.file.*;

class TestEditProperties {
  public static void main(String[] args) {
    try {
        Model model = new Model("gait10dof18musc_subject01.osim");
        OpenSimContext context=null;
        try {
            context = new OpenSimContext(model.initSystem(), model);
        } catch (IOException ex) {
            ex.printStackTrace();
            System.out.println("Failed to initialize Context");
            System.exit(-1);
        }
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
        System.out.println("Test finished!");
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