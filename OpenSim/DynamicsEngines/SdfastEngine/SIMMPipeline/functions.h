/*******************************************************************************

   FUNCTIONS.H

   Authors: Peter Loan
            Krystyne Blaikie

   Copyright (c) 1996-2004 MusculoGraphics, a division of Motion Analysis Corp.
   All rights reserved.

   Description: This file contains function prototypes for all functions which
      are callable by the GMC or SD/FAST user.

*******************************************************************************/

#ifndef FUNCTIONS_H
#define FUNCTIONS_H

int    acpp_main(int argc, char* argv[]);
void   append_if_necessary (char* str, char c);
void   apply_external_forces(double time, MotionData* data);
void   apply_joint_restraint_torques(double q[], double u[]);
void   apply_muscle_forces(dpModelStruct* sdm);
void   assemble_model(double t, double y[]);
void   assign_muscle_states(dpModelStruct* sdm, double time, double state[]);
double calc_active_force(dpMuscleStruct* ms, double norm_fiber_length);
double calc_distance_between_points(double p1[], int n1, double p2[], int n2);
double calc_excitation(dpMuscleStruct* ms, double time, double state[]);
double calc_fiber_force(dpMuscleStruct* ms, double activation,
			double norm_fiber_length, double norm_fiber_velocity);
double calc_fiber_velocity(dpMuscleStruct* ms, double activation,
			   double active_force, double velocity_dependent_force);
double calc_force_velocity(dpMuscleStruct* ms, double norm_fiber_velocity);
double calc_distance_to_ellipsoid(double p1[], double radii[], double projpt[]);//for constraint objects
void   calc_joint_torques(double time, double state[], double dy[]);
void   calc_ligament_force(dpMuscleStruct* ms);
int    calc_line_intersect_cylinder (
	double		p11[3],		/* input:  muscle point 1 */
	double		p22[3],		/* input:  muscle point 2 */
   double		p0[3],      /* input:  point on cylinder axis */
	double		dn[3],      /* input:  direction vector of cyl axis */
	double		r,          /* input:  cylinder radius */
	double		len,        /* input:  cylinder length */
	double*		rlen,		   /* output: length of surface wrap */
	double		r1[],		   /* output: wrap tangent point 1 */
	double		r2[],		   /* output: wrap tangent point 2 */
	double**	wrap_pts,	   /* output: intermediate surface wrap pts */
	int*		num_wrap_pts,	/* output: number of intermediate pts */
	int		wrap_axis,	   /* input:  constraint axis */
	int		wrap_sign,	   /* input:  constraint direction */
	dpMuscleWrapStruct* mwrap, /* in/out: muscle wrap structure */
	int*		p_flag,		   /* output: did wrapping occur? */
	dpWrapObject*	wo);        /* input:  pointer to the cylinder wrap object */
int    calc_line_intersect_ellipsoid (
	double		p1[3],		/* input:  muscle point 1		*/
	double		p2[3],		/* input:  muscle point 2		*/
	double		m[3],		/* input:  ellipsoid origin (0,0,0)	*/
	double		a[3],		/* input:  ellipsoid xyz radii		*/
	double*		rlen,		/* output: length of surface wrap	*/
	double		r1[],		/* output: wrap tangent point 1		*/
	double		r2[],		/* output: wrap tangent point 2		*/
	double**	wrap_pts,	/* output: intermediate surface wrap pts*/
	int*		num_wrap_pts,	/* output: number of intermediate pts	*/
	int		wrap_axis,	/* input:  constraint axis		*/
	int		wrap_sign,	/* input:  constraint direction		*/
	dpMuscleWrapStruct* mwrap, /* in/out: muscle wrap structure	*/
	int*		p_flag,		/* output: did wrapping occur?		*/
	dpWrapObject*	wo);
int    calc_line_intersect_sphere(double p1[], double p2[], double m[],
			       double r, double *rlen, double r1[],
			       double r2[],
			       double** wrap_pts, int *num_wrap_pts,
			       int wrap_axis, int wrap_sign, dpMuscleWrapStruct* mwrap,
			       int *p_flag,
			       dpWrapObject* wo);
int    calc_line_intersect_torus (
	double		p1[3],		/* input:  muscle point 1		*/
	double		p2[3],		/* input:  muscle point 2		*/
	double		m[3],		/* input:  torus origin (0,0,0)	*/
	double		a[3],		/* input:  torus xy[z] radii		*/
	double*		rlen,		/* output: length of surface wrap	*/
	double		r1[],		/* output: wrap tangent point 1		*/
	double		r2[],		/* output: wrap tangent point 2		*/
	double**	wrap_pts,	/* output: intermediate surface wrap pts*/
	int*		num_wrap_pts,	/* output: number of intermediate pts	*/
	int		wrap_axis,	/* input:  constraint axis		*/
	int		wrap_sign,	/* input:  constraint direction		*/
	dpMuscleWrapStruct* mwrap, /* in/out: muscle wrap structure	*/
	int*		p_flag,		/* output: did wrapping occur?		*/
	dpWrapObject*	wo);
void   calc_muscle_power(dpMuscleStruct* ms);
void   calc_muscle_tendon_force(dpMuscleStruct* ms, double activation);
double calc_muscle_tendon_length(dpMuscleStruct* ms);
double calc_muscle_tendon_velocity(dpMuscleStruct* ms);
double calc_nonzero_passive_force(dpMuscleStruct* ms, double norm_fiber_length,
				  double norm_fiber_velocity);
double calc_passive_force(dpMuscleStruct* ms, double norm_fiber_length,
			  double norm_fiber_velocity);
double calc_pennation(double fiber_length, double optimal_fiber_length,
		      double initial_pennation_angle);
void   calc_system_energy(SystemInfo* si);
double calc_tendon_force(dpMuscleStruct* ms, double norm_tendon_length);
void   check_object_info(void);
void   check_wrapping_points(dpMuscleStruct* ms, double state[]);
char*  clean_string(char buf[]);
void   convert_point(double pt[], int frame1, int frame2);
void   convert_string(char str[]);
int    count_muscle_states(dpModelStruct* sdm);
int    enter_gencoord(dpModelStruct* sdm, char gencoord_name[]);
int    enter_segment(dpModelStruct* sdm, char segment_name[]);
void   error(ErrorAction action, char str_buffer[]);
ReturnCode finish_spline_func(dpSplineFunction* sf);
void   free_function(dpSplineFunction* func);
void   free_motion_data(MotionData* md);
void   get_applied_force(double time, MotionData* data, int frc_index, double appl_point[], double force_vec[]);
void   get_applied_torque(double time, MotionData* data, int trq_index, double torque_vec[]);
const  char* get_function_suffix(dpSplineFunction* func);
ReturnCode get_muscle_file_name(char params_file[], char** muscle_file);
int    get_muscle_param_index(dpMuscleStruct* ms, char param_name[]);
double get_muscle_param_value(dpMuscleStruct* ms, char param_name[]);
ReturnCode get_string_pair(char str_buffer[], char str1[], char str2[]);
const  char* get_suffix(const char str[]);
char*  get_temp_file_name(const char* baseFileName);
dpWrapObject* get_wrap_object(char name[]);
ReturnCode get_xypair_from_string(char str_buffer[], double* x, double* y);
void   init_constraint_objects(void);
void   init_joint_functions(void);
void   init_motion(void);
void   init_muscle_states(dpModelStruct* sdm, double state[]);
void   init_object_info(void);
void   init_q_restraint_functions(void);
void   init_qs(void);
void   init_segments(void);
ReturnCode init_spline_func(dpSplineFunction* sf, double Ycoords[], int numpoints, dpSplineType type,
                            double cutoffFrequency);
void   init_wrap_objects(void);
void   integrate(void (*func)(), double *time, double st[], double dst[], double param[],
                 double dt, double *step, int neqin, double tol, double work[], int *err, int *which);
void   invert_3x3matrix(double matrix[][3], double inverse[][3]);
void   invert_4x4matrix(double matrix[][4], double inverse[][4]);
void   invert_4x4transform(double matrix[][4], double inverse[][4]);
void   invert_matrix(double* matrix[], double* inverse[], int size);
dpBoolean is_absolute_path(char *path);
ReturnCode load_kinetics_data(dpModelStruct* sdm, MotionData* data, char filename[]);
ReturnCode make_zero_func(dpSplineFunction* sf, double min_x, double max_x);
ReturnCode malloc_function(dpSplineFunction* func, int numpoints);
ReturnCode mstrcpy(char* dest_str[], char original_str[]);
void   optimize_muscle_activations(double time, double state[]);
char*  parse_string(char str_buffer[], VariableType var_type, void* dest_var);
FILE*  preprocess_file(char infile[], char outfile[]);
void   print_3x3matrix(double mat[][3], char string[], FILE* stream);
void   read_contact_pair(FILE** fp);
ReturnCode read_dllparameters_file(char params_file[], dpModelStruct* sdm);
ReturnCode read_double_array(FILE **fp, char ending[], char name[], dpSplineFunction* func);
ReturnCode read_kinetics_file(dpModelStruct* sdm, MotionData* data, char filename[]);
int    read_line(FILE** fp, char str_buffer[]);
ReturnCode read_motion_file(dpModelStruct* sdm, MotionData* data, char filename[]);
dpMusclePoint* read_muscle_attachment_points(FILE** fp, dpModelStruct* sdm,
					   int* numpoints, int* mp_array_size,
					   dpBoolean* has_wrapping_points);
ReturnCode read_muscle_file(dpModelStruct* sdm, char filename[]);
ReturnCode read_muscle_groups(FILE** fp);
int    read_nonempty_line(FILE** fp, char str_buffer[]);
void   read_object(FILE** fp);
void   read_object_group(FILE** fp);
ReturnCode read_parameters_file(char params_file[], dpModelStruct* sdm,
				                    char** kinetics_file,	char** output_motion_file,
                                char** output_kinetics_file);
ReturnCode read_polyhedron(dpPolyhedronStruct* ph, char filename[]);
int    read_string(FILE** fp, char str_buffer[]);
int    read_til_tokens(FILE** file, char buf[], const char delimiters[]);
ReturnCode realloc_function(dpSplineFunction* func, int size);
int    sduforce(double t, double q[], double u[]);
int    sdumotion(double t, double q[], double u[]);
void   set_fixed_gencoords(void);
void   set_initial_conditions(double y[], double dy[]);
void   set_prescribed_gencoords(double time, double q[], double u[], MotionData* data);
void   set_prescribed_motion(dpQType type, int value);
void   set_up_kinetics_input(char filename[], MotionData** data);
ReturnCode setup_spline_structs(dpModelStruct* sdm, MotionData* data);
FILE*  simm_fopen (const char* name, const char* mode);
void*  simm_malloc(unsigned mem_size);
void*  simm_calloc(unsigned num_elements, unsigned elem_size);
void*  simm_realloc(void* ptr, unsigned mem_size, ReturnCode* rc);
int    sim_message(ErrorAction action, const char* format, ...);
ReturnCode verify_assembly(dpModelStruct* sdm, double state[]);
void   write_kin_out_frame(double time, double y[], double dy[], FILE **fp);

/* Function Prototypes for the user-created muscle-state functions */

/* functions defined in INITS.C */
int muscle_init_func1(dpMuscleStruct *ms, double state[]);
int muscle_init_func2(dpMuscleStruct *ms, double state[]);
int muscle_init_func3(dpMuscleStruct *ms, double state[]);
int muscle_init_func4(dpMuscleStruct *ms, double state[]);
int muscle_init_func5(dpMuscleStruct *ms, double state[]);
int muscle_init_func6(dpMuscleStruct *ms, double state[]);
int muscle_init_func7(dpMuscleStruct *ms, double state[]);
int muscle_init_func8(dpMuscleStruct *ms, double state[]);
int muscle_init_func9(dpMuscleStruct *ms, double state[]);
int muscle_init_func10(dpMuscleStruct *ms, double state[]);

/* functions defined in DERIVS.C */
int muscle_deriv_func1(double time, dpMuscleStruct *ms, double state[],
		       double dstate[], double *muscle_force);
int muscle_deriv_func2(double time, dpMuscleStruct *ms, double state[],
		       double dstate[], double *muscle_force);
int muscle_deriv_func3(double time, dpMuscleStruct *ms, double state[],
		       double dstate[], double *muscle_force);
int muscle_deriv_func4(double time, dpMuscleStruct *ms, double state[],
		       double dstate[], double *muscle_force);
int muscle_deriv_func5(double time, dpMuscleStruct *ms, double state[],
		       double dstate[], double *muscle_force);
int muscle_deriv_func6(double time, dpMuscleStruct *ms, double state[],
		       double dstate[], double *muscle_force);
int muscle_deriv_func7(double time, dpMuscleStruct *ms, double state[],
		       double dstate[], double *muscle_force);
int muscle_deriv_func8(double time, dpMuscleStruct *ms, double state[],
		       double dstate[], double *muscle_force);
int muscle_deriv_func9(double time, dpMuscleStruct *ms, double state[],
		       double dstate[], double *muscle_force);
int muscle_deriv_func10(double time, dpMuscleStruct *ms, double state[],
			double dstate[], double *muscle_force);

/* functions defined in ASSIGNS.C */
int muscle_assign_func1(double time, dpMuscleStruct *ms, double state[]);
int muscle_assign_func2(double time, dpMuscleStruct *ms, double state[]);
int muscle_assign_func3(double time, dpMuscleStruct *ms, double state[]);
int muscle_assign_func4(double time, dpMuscleStruct *ms, double state[]);
int muscle_assign_func5(double time, dpMuscleStruct *ms, double state[]);
int muscle_assign_func6(double time, dpMuscleStruct *ms, double state[]);
int muscle_assign_func7(double time, dpMuscleStruct *ms, double state[]);
int muscle_assign_func8(double time, dpMuscleStruct *ms, double state[]);
int muscle_assign_func9(double time, dpMuscleStruct *ms, double state[]);
int muscle_assign_func10(double time, dpMuscleStruct *ms, double state[]);

/* functions defined in PIPETOOLS.C */
void add_torques_to_motion_data(dpModelStruct* sdm, MotionData* data);
void calc_corrective_torques(double time, double q[], double u[], dpModelStruct* sdm, MotionData* data);
void calc_initial_joint_torques(double time, double state[], double dy[], MotionData* data);
void calc_next_joint_torques(double time, double state[], double dy[], MotionData* data);
int  check_for_sderror(char caller[]);
void convert_point2(int frame1, double p1[], int frame2, double p2[]);
void init_joints(void);

/* functions defined in INVERT.C */
void ludcmp(double* a[], int n, int indx[], double* d);
void lubksb(double* a[], int n, int indx[], double b[]);

/* functions defined in LCP_SOLVER.C */
#if CONTACT_DETECTION
void compute_contact_forces(int impact_flag);
#endif

/* functions defined in IMPACT.C */
#if CONTACT_DETECTION
void handle_impacts(double t, double y[], double dy[]);
#endif

/* functions defined in CONTACT.C */
#if CONTACT_DETECTION
void apply_forces_at_contacts(int n, dpContactInfo list[]);
void check_for_initial_penetration(double y[], double dy[]);
void copy_cnode(dpContactInfo from, dpContactInfo *to);
void create_bilateral_contacts(double y[]);
void determine_contacts(double y[], double dy[]);
void get_bilat_contact_info(void);
void get_contact_info(void);
double get_norm_rel_vel(int i, double vn[]);
double get_rel_vel(int cont, double rel_vel[]);
double get_tang_rel_vel(int i, double vt[]);
void init_contact_info(void);
ReturnCode makepaths(void);
void print_contact_list(dpContactInfo list[], int n);
#endif

/* functions defined in GMC.C */
void calc_muscle_derivatives(double time, double state[], double dstate[], 
	double param[], int* status);

/* functions derived in MAIN.C */
void calc_derivatives(double t, double y[], double dy[], double param[], int *status);

/* functions defined in OUTPUT.c */
void get_reaction_forces(void);
double get_time_now(void);
void print_body_info(void);
void print_final_information(double time, double y[], double dy[], SystemInfo* si);
void print_simulation_info(double time, double state[], SystemInfo* si);
void print_state(double y[], double dy[]);
void set_up_motion_file(FILE **fp, char filename[]);
void show_array(char name[], double vect[], int start, int end);
void show_array_as_matrix(char name[], double vec[], int n, int m);
void show_matrix(char name[], double *mat[], int m, int n);
void show_vector(char name[], double vect[]);
void time_elapsed(double t_in, double t_out);
void create_output_motion(char name[]);
void open_output_motion_file(char name[]);
void store_motion_frame(double time, double state[]);
void get_motion_info(dpDataSetup* dataSetup);
void get_motion_data(dpDataFrame* dataFrame);

/* functions defined in OBJECT.c */
void copy_polyhedron(dpPolyhedronStruct* from, dpPolyhedronStruct* to);
void show_object_info(void);

/* spring-based contact detection routines */
void check_spring_info(void);
void apply_spring_forces(dpModelStruct *sdm);
void show_spring_info(void);
dpBoolean vector_intersects_matte(double pt[], double vec[], dpForceMatte* fm, double inter[]);

/* functions defined in constraint.c */
double calculate_constraint_position_error(dpConstraintObject *co, int status, int point);
double calculate_constraint_velocity_error(dpConstraintObject *co, int point);
double calculate_constraint_acceleration_error(dpConstraintObject *co, int point);
void   determine_projected_point(dpConstraintObject *co, double pt[], double projpt[], double norm[]);
void apply_constraint_forces(dpConstraintObject *co, double multiplier, int j);

#endif /* FUNCTIONS_H */
