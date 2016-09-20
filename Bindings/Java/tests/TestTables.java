import org.opensim.modeling.*;
import org.opensim.modeling.DataTable;
import org.opensim.modeling.RowVector;
import org.opensim.modeling.StdVectorString;

class TestTables {
  public static void main(String[] args) {
        DataTable table = new DataTable();
        StdVectorString labels = new StdVectorString();
        labels.add("0");
        labels.add("1");
        labels.add("2");
        table.setColumnLabels(labels);
        RowVector row0 = new RowVector(3, 7.);
        table.appendRow(0, row0);
        double value = row0.__getitem__(2);
        System.out.println("Value ="+value);
        double shouldThrow = row0.__getitem__(4);
  }
}
