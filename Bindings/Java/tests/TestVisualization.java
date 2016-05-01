import org.opensim.modeling.*;
import java.io.IOException;
import static java.nio.file.StandardOpenOption.*;
import java.nio.file.*;


class TestVisualization {
  public static void main(String[] args) {
    try {
        Model model = new Model("gait10dof18musc_subject01.osim");
        model.initSystem();
        long t1  = System.currentTimeMillis();
        for (int i=0; i<10; i++){
            ComponentsList mcList = model.getComponentsList();
            long t2  = System.currentTimeMillis();
            ComponentIterator mcIter = mcList.begin();
            ModelDisplayHints modelDisplayHints = new ModelDisplayHints();
            ArrayDecorativeGeometry adg = new ArrayDecorativeGeometry();
            while (!mcIter.equals(mcList.end())){
                Component mc = mcIter.__deref__();
                mc.generateDecorations(true, modelDisplayHints, model.getWorkingState(), adg);
                adg.clear();
                mc.generateDecorations(false, modelDisplayHints, model.getWorkingState(), adg);
                adg.clear();
                mcIter.next();
            }
            long t3 = System.currentTimeMillis();
            System.out.println("Test iteration"+i+" : time (ms):"+(t3 - t2));
        }
    }
    catch(IOException ex){
        System.out.println("Failed to load model or in initSystem");
        System.exit(-1);
    }
  }
}
