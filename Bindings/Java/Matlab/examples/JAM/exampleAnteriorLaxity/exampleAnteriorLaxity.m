import org.opensim.modeling.*
Logger.setLevelString('Trace');

file = 'P:\Projects\MAS_opensim_jam\examples\anterior_laxity\inputs\forsim_settings.xml';

fs = ForsimTool(file);
fs.set_model_file('Path')
fs.run();
%IK = ForwardTool();
