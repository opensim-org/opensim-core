allMuscleNames = { ...
    'glut_max1_l' 'glut_max2_l' 'glut_max3_l' ...
    'glut_max1_r' 'glut_max2_r' 'glut_max3_r' ...
    'glut_med1_l' 'glut_med2_l' 'glut_med3_l' ...
    'glut_med1_r' 'glut_med2_r' 'glut_med3_r' ...
    'glut_min1_l' 'glut_min2_l' 'glut_min3_l' ...
    'glut_min1_r' 'glut_min2_r' 'glut_min3_r' ...
    'iliacus_l' 'psoas_l' 'iliacus_r' 'psoas_r' 'sar_l' 'sar_r' ...
    'tfl_l' 'tfl_r' 'vas_med_l' 'vas_int_l' 'vas_lat_l' ...
    'vas_med_r' 'vas_int_r' 'vas_lat_r' 'rect_fem_l' 'rect_fem_r' ...
    'semimem_l' 'semiten_l' 'semimem_r' 'semiten_r' 'bifemlh_l' ...
    'bifemlh_r' 'bifemsh_l' 'bifemsh_r' 'add_brev_l' 'add_long_l' ...
    'add_brev_r' 'add_long_r' 'add_mag1_l' 'add_mag2_l' ...
    'add_mag3_l' 'add_mag1_r' 'add_mag2_r' 'add_mag3_r' ...
    'pect_l' 'grac_l' 'pect_r' 'grac_r' 'quad_fem_l' 'quad_fem_r' ...
    'gem_l' 'gem_r' 'peri_l' 'peri_r' 'med_gas_l' 'lat_gas_l' ...
    'med_gas_r' 'lat_gas_r' 'soleus_l' 'tib_post_l' ...
    'soleus_r' 'tib_post_r' 'flex_dig_l' 'flex_hal_l' ...
    'flex_dig_r' 'flex_hal_r' 'tib_ant_l' 'ext_hal_l' ...
    'tib_ant_r' 'ext_hal_r' 'ext_dig_l' 'per_tert_l' ...
    'ext_dig_r' 'per_tert_r' 'per_long_l' 'per_brev_l' ...
    'per_long_r' 'per_brev_r' 'ercspn_l' 'ercspn_r' ...
    'intobl_l' 'intobl_r' 'extobl_l' 'extobl_r' };

% Also do: each muscle: left and right together in a group

% Hip extensors (glutes)
glut_max_l = { 'glut_max1_l' 'glut_max2_l' 'glut_max3_l' };
glut_max_r = { 'glut_max1_r' 'glut_max2_r' 'glut_max3_r' };
glut_max = [ glut_max_l glut_max_r ];
glut_med_l = { 'glut_med1_l' 'glut_med2_l' 'glut_med3_l' };
glut_med_r = { 'glut_med1_r' 'glut_med2_r' 'glut_med3_r' };
glut_med = [ glut_med_l glut_med_r ];
glut_min_l = { 'glut_min1_l' 'glut_min2_l' 'glut_min3_l' };
glut_min_r = { 'glut_min1_r' 'glut_min2_r' 'glut_min3_r' };
glut_min = [ glut_min_l glut_min_r ];
hip_extensors_l = [ glut_max_l glut_med_l glut_min_l ];
hip_extensors_r = [ glut_max_r glut_med_r glut_min_r ];
hip_extensors = [ hip_extensors_l hip_extensors_r ];
glutes_l = hip_extensors_l;
glutes_r = hip_extensors_r;
glutes = hip_extensors;
glutes1_l = { 'glut_max1_l' 'glut_med1_l' 'glut_min1_l' };
glutes1_r = { 'glut_max1_r' 'glut_med1_r' 'glut_min1_r' };
glutes1 = [ glutes1_l glutes1_r ];

% Hip flexors
iliopsoas_l = { 'iliacus_l' 'psoas_l' };
iliopsoas_r = { 'iliacus_r' 'psoas_r' };
sar_l = { 'sar_l' }; sar_r = { 'sar_r' };
tfl_l = { 'tfl_l' }; tfl_r = { 'tfl_r' };
hip_flexors_l = [ iliopsoas_l sar_l tfl_l ];
hip_flexors_r = [ iliopsoas_r sar_r tfl_r ];
hip_flexors = [ hip_flexors_l hip_flexors_r ];

% Knee extensors (quadriceps or "quads")
vasti_l = { 'vas_med_l' 'vas_int_l' 'vas_lat_l' };
vasti_r = { 'vas_med_r' 'vas_int_r' 'vas_lat_r' };
rect_fem_l = { 'rect_fem_l' }; rect_fem_r = { 'rect_fem_r' };
knee_extensors_l = [ vasti_l rect_fem_l ];
knee_extensors_r = [ vasti_r rect_fem_r ];
knee_extensors = [ knee_extensors_l knee_extensors_r ];
quadriceps_l = knee_extensors_l;
quadriceps_r = knee_extensors_r;
quadriceps = knee_extensors;
quads_l = quadriceps_l;
quads_r = quadriceps_r;
quads = quadriceps;

% Hamstrings (proximal knee flexors)
medial_hamstrings_l = { 'semimem_l' 'semiten_l' };
medial_hamstrings_r = { 'semimem_r' 'semiten_r' };
lateral_hamstrings_l = { 'bifemlh_l' 'bifemsh_l' };
lateral_hamstrings_r = { 'bifemlh_r' 'bifemsh_r' };
hamstrings_l = [ medial_hamstrings_l lateral_hamstrings_l ];
hamstrings_r = [ medial_hamstrings_r lateral_hamstrings_r ];
hamstrings = [ hamstrings_l hamstrings_r ];
proximal_knee_flexors_l = hamstrings_l;
proximal_knee_flexors_r = hamstrings_r;
proximal_knee_flexors = hamstrings;

% Plantarflexors (ankle extensors) and distal knee flexors (gastroc)
gas_l = { 'med_gas_l' 'lat_gas_l' };
gas_r = { 'med_gas_r' 'lat_gas_r' };
gas = [ gas_l gas_r ];
distal_knee_flexors_l = gas_l;
distal_knee_flexors_r = gas_r;
distal_knee_flexors = gas;
knee_flexors_l = [ proximal_knee_flexors_l distal_knee_flexors_l ];
knee_flexors_r = [ proximal_knee_flexors_r distal_knee_flexors_r ];
knee_flexors = [ knee_flexors_l knee_flexors_r ];
toe_flexors_l = { 'flex_dig_l' 'flex_hal_l' };
toe_flexors_r = { 'flex_dig_r' 'flex_hal_r' };
toe_flexors = [ toe_flexors_l toe_flexors_r ];
plantarflexors_l = [ gas_l { 'soleus_l' 'tib_post_l' } toe_flexors_l ];
plantarflexors_r = [ gas_r { 'soleus_r' 'tib_post_r' } toe_flexors_r ];
plantarflexors = [ plantarflexors_l plantarflexors_r ];
ankle_extensors_l = plantarflexors_l;
ankle_extensors_r = plantarflexors_r;
ankle_extensors = plantarflexors;

% Dorsiflexors (ankle flexors)
ta_l = { 'tib_ant_l' }; ta_r = { 'tib_ant_r' }; ta = [ ta_l ta_r ];
toe_extensors_l = { 'ext_hal_l' 'ext_dig_l' 'per_tert_l' };
toe_extensors_r = { 'ext_hal_r' 'ext_dig_r' 'per_tert_r' };
toe_extensors = [ toe_extensors_l toe_extensors_r ];
dorsiflexors_l = [ ta_l toe_extensors_l ];
dorsiflexors_r = [ ta_r toe_extensors_r ];
dorsiflexors = [ dorsiflexors_l dorsiflexors_r ];
ankle_flexors_l = dorsiflexors_l;
ankle_flexors_r = dorsiflexors_r;
ankle_flexors = dorsiflexors;

% Ankle everters
everters_l = { 'per_long_l' 'per_brev_l' };
everters_r = { 'per_long_r' 'per_brev_r' };
everters = [ everters_l everters_r ];

% Hip adductors
adductors_l = { 'add_brev_l' 'add_long_l' ...
    'add_mag1_l' 'add_mag2_l' 'add_mag3_l' };
adductors_r = { 'add_brev_r' 'add_long_r' ...
    'add_mag1_r' 'add_mag2_r' 'add_mag3_r' };
adductors = [ adductors_l adductors_r ];
hip_adductors_l = adductors_l;
hip_adductors_r = adductors_r;
hip_adductors = adductors;

% Hip rotators
medial_rotators_l = { 'pect_l' 'grac_l' };
medial_rotators_r = { 'pect_r' 'grac_r' };
medial_rotators = [ medial_rotators_l medial_rotators_r ];
lateral_rotators_l = { 'quad_fem_l' 'gem_l' 'peri_l' };
lateral_rotators_r = { 'quad_fem_r' 'gem_r' 'peri_r' };
lateral_rotators = [ lateral_rotators_l lateral_rotators_r ];
rotators_l = [ medial_rotators_l lateral_rotators_l ];
rotators_r = [ medial_rotators_r lateral_rotators_r ];
rotators = [ rotators_l rotators_r ];
hip_rotators_l = rotators_l;
hip_rotators_r = rotators_r;
hip_rotators = rotators;
