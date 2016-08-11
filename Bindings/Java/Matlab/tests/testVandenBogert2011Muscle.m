
import org.opensim.modeling.*

muscle = VandenBogert2011Muscle();
Lm = 0;
Lce = 0;
a = 0;
Lcedot = 0;
adot = 0;
u = 0;
out = muscle.calcImplicitResidual(Lm, Lce, a, Lcedot, adot, u, 0);
% out should have 3 elements:
%assert(out.size() == 3);
%disp(out.get(0))
%disp(out.get(1))
%disp(out.get(2))
methods(out)

% Make sure that we can access the Mat22:
df_dy = out.getDf_dy();
df_dy.get(0, 0);
df_dy.get(1, 1);
df_dy.set(1, 1, 1.5);


% Ensure the nested struct is wrapped.
impRes = VandenBogert2011MuscleImplicitResidual()
