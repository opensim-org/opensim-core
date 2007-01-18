trialname = 'ss_walking1';
tstart = 0;
tend = 2.5;
min_excitation = 0.02;

ref_dataFormat = ref_dataFormatDelaware;
[sInfo, tInfo] = ref_trialInfoDelaware;

trial = tInfo.(trialname);

for limb = [ 'R' 'L' ]
    ic.(limb) = trial.ictoMatrix(strmatch(limb,trial.limb),1)./ref_dataFormat.analogRate;
    % add extra time points before first and after last by extrapolating using the average cycle duration
    avgDuration = mean(ic.(limb)(2:end)-ic.(limb)(1:(end-1)));
    ic.(limb) = [ ic.(limb)(1)-avgDuration; ic.(limb); ic.(limb)(end)+avgDuration ];
    indices = find(ic.(limb) >= tstart & ic.(limb) <= tend);
    if indices(1) > 1
        indices = [indices(1)-1; indices];
    end
    if indices(end) < length(ic.(limb))
        indices = [indices; indices(end)+1];
    end
    ic.(limb) = ic.(limb)(indices);
end

constraints = read_excitationGuiOutput('_excitation_ranges.txt');

muscles = { 'glut_med1', 'glut_med2', 'glut_med3', 'glut_min1', 'glut_min2', 'glut_min3', 'semimem', ...
            'semiten', 'bifemlh', 'bifemsh', 'sar', 'add_long', 'add_brev', 'add_mag1', 'add_mag2', ...
            'add_mag3', 'tfl', 'pect', 'grac', 'glut_max1', 'glut_max2', 'glut_max3', 'iliacus', ...
            'psoas', 'quad_fem', 'gem', 'peri', 'rect_fem', 'vas_med', 'vas_int', 'vas_lat', 'med_gas', ...
            'lat_gas', 'soleus', 'tib_post', 'flex_dig', 'flex_hal', 'tib_ant', 'per_brev', 'per_long', ...
            'per_tert', 'ext_dig', 'ext_hal', 'ercspn', 'intobl', 'extobl' };

fid =  fopen('_control_constraints.xml', 'w');
for limb = [ 'R' 'L' ]
    llimb = lower(limb); % lowercase
    for i = 1:length(muscles)
        curves.t = [];
        curves.min = [];
        curves.max = [];
        msl = [muscles{i} '_' llimb];
        if isfield(constraints, msl)
            for cycle = 1:length(ic.(limb))-1
                cycleStart = ic.(limb)(cycle);
                cycleEnd = ic.(limb)(cycle+1);
                curves.t = [curves.t; cycleStart + constraints.(msl).t'./100 .* (cycleEnd-cycleStart)];
                curves.min = [curves.min; constraints.(msl).min_value'];
                curves.max = [curves.max; constraints.(msl).max_value'];
            end

            I = find(curves.t >= tstart & curves.t <= tend);
            curves.t = curves.t(I);
            curves.min = max(min_excitation, curves.min(I));
            curves.max = max(min_excitation, curves.max(I));
        end

        if length(curves.t)
            fprintf(fid, '\t\t<ControlLinear name="%s.excitation">\n', msl);
            fprintf(fid, '\t\t\t<min_nodes>\n');
            for j = 1:length(curves.t)
                fprintf(fid, '\t\t\t\t<ControlLinearNode> <t> %f </t> <value> %f </value> </ControlLinearNode>\n', curves.t(j), curves.min(j));
            end
            fprintf(fid, '\t\t\t</min_nodes>\n');
            fprintf(fid, '\t\t\t<max_nodes>\n');
            for j = 1:length(curves.t)
                fprintf(fid, '\t\t\t\t<ControlLinearNode> <t> %f </t> <value> %f </value> </ControlLinearNode>\n', curves.t(j), curves.max(j));
            end
            fprintf(fid, '\t\t\t</max_nodes>\n');
            fprintf(fid, '\t\t</ControlLinear>\n', msl);
        else
            fprintf(fid, '\t\t<ControlLinear name="%s.excitation" />\n', msl);
        end
    end
end
