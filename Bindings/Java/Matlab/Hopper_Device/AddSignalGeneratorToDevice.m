function AddSignalGeneratorToDevice(device)
% Add a SignalGenerator to a device.
% TODO license

import org.opensim.modeling.*;

signalGen = SignalGenerator();
signalGen.setName('signalGen');

% Try changing the constant value and/or the function (e.g., try a
% LinearFunction).
signalGen.set_function(Constant(0.33));
device.addComponent(signalGen);

% Connect the signal generator's output signal to the controller's activation
% input.
device.updComponent('controller').updInput('activation').connect(...
        signalGen.getOutput('signal'));

end
