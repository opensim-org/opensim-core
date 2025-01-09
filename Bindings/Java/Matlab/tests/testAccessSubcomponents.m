
import org.opensim.modeling.*

model = Model('arm26.osim');

% Individual components
% =====================
muscle = model.getComponent('forceset/BICshort');
assert(strcmp(muscle.getName(), 'BICshort'));
thelenMuscle = Thelen2003Muscle.safeDownCast(muscle);
thelenMuscle.get_max_isometric_force();
muscle = model.updComponent('forceset/BICshort');
updThelenMuscle = Thelen2003Muscle.safeDownCast(muscle)
updThelenMuscle.set_max_isometric_force(100);

% ComponentList
% =============

% Generic.
comps = model.getComponentsList();
it = comps.begin();
countComps = 0;
while ~it.equals(comps.end())
    it.getName(); % Object.
    it.getNumSockets(); % Component.
    it.next();
    countComps = countComps + 1;
end
assert(countComps > 0);

% Specific type of component.
muscleList = model.getMuscleList();
muscleIter = muscleList.begin();
countMuscles = 0;
while ~muscleIter.equals(muscleList.end())
    % SWIG puts all of Muscle's methods on MuscleIterator.
    muscleIter.get_max_isometric_force();
    % Access the underlying Muscle.
    % TODO MATLAB methods can't start with underscore muscle = muscleIter.__deref__();
    % TODO MATLAB methods can't start with underscore muscle.get_max_isometric_force();
    muscleIter.next();
    countMuscles = countMuscles + 1;
end
assert(countMuscles > 0);

% Custom type filter.
% TODO does not work (yet).
% TODO it = comps.begin();
% TODO while ~it.equals(comps.end())
% TODO     disp(it.getConcreteClassName());
% TODO     if it.isA('Thelen2003Muscle')
% TODO         disp(it.getConcreteClassName());
% TODO     end
% TODO     it.next();
% TODO end

% Use ComponentList filter.
muscleList.setFilter(ComponentFilterAbsolutePathNameContainsString('BIC'));
muscleIter = muscleList.begin();
countBICMuscles = 0;
BICnames = {'BIClong', 'BICshort'};
while ~muscleIter.equals(muscleList.end())
    countBICMuscles = countBICMuscles + 1;
    disp(muscleIter.getName());
    assert(strcmp(BICnames{countBICMuscles}, muscleIter.getName()));
    muscleIter.next();
end





