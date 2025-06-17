#include "ModelOperators.h"

#include <OpenSim/Common/GCVSplineSet.h>

using namespace OpenSim;

void ModOpPrescribeCoordinateValues::operate(
        Model& model, const std::string&) const {
    model.finalizeFromProperties();
    TimeSeriesTable table = get_coordinate_values().process();
    GCVSplineSet statesSpline(table);

    for (const std::string& pathString : table.getColumnLabels()) {
        ComponentPath path = ComponentPath(pathString);
        if (path.getNumPathLevels() < 3) { continue; }
        std::string jointPath = path.getParentPath().getParentPath().toString();
        if (!model.hasComponent<Joint>(jointPath)) {
            log_warn("Found column label '{}', but it does not match a "
                     "joint coordinate value in the model.", pathString);
            continue;
        }
        Coordinate& q = model.updComponent<Joint>(jointPath).updCoordinate();
        q.setPrescribedFunction(statesSpline.get(pathString));
        q.setDefaultIsPrescribed(true);
    }
}
