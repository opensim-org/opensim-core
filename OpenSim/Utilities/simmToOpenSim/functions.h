/*******************************************************************************

   FUNCTIONS.H

   Author: Peter Loan

   Date: 13-APR-89

   Copyright (c) 1992-5 MusculoGraphics, Inc.
   All rights reserved.
   Portions of this source code are copyrighted by MusculoGraphics, Inc.

*******************************************************************************/

#ifndef FUNCTIONS_H
#define FUNCTIONS_H

#include "basic.h"
#include "f2c.h"

SBoolean _confirm_overwrite (const char* filename);
void _extract_rotations_for_joint(const DMatrix m, const JointStruct*, double* rots);
void _open_files (int argc, char* argv[], SBoolean showDialogBox);
SBoolean _read_til (FILE* file, int c);
int  _read_til_tokens (FILE* file, char* buf, const char* delimiters);
void _strip_outer_whitespace (char* str);
void add_default_motion_objects(ModelStruct*);
MotionObjectInstance* add_foot_print(ModelStruct* model, MotionSequence* motion, const char name[], double scale,
                                     double translation[], double eulerRotation[]);
MotionObjectInstance* add_force_plate(ModelStruct* model, MotionSequence* motion, const char name[], double scale[],
                                      double translation[], double eulerRotation[]);
Marker* add_marker(ModelStruct* model, SegmentStruct* segment, Marker* marker, SBoolean notify);
ReturnCode add_model(char jointfilename[], char musclefilename[], int suggested_win_width,
                     int* mod, SBoolean showTopLevelMessages);
void add_motion_frame(ModelStruct* ms, const char* fullpath);
SBoolean add_motion_to_model(MotionSequence* motion, ModelStruct* ms, SBoolean showTopLevelMessages);
void add_muscle_attachment_point(ModelStruct* model, int muscleIndex, int pointIndex);
PlotStruct* add_plot(void);
void add_preprocessor_option(SBoolean isDefaultOption, const char* format, ...);
void add_text_to_help_struct(char line[], HelpStruct* hp, SBoolean new_line,
                 int text_format, int text_xoffset);
int add_window(WindowParams* win_parameters, WinUnion* win_struct,
           WindowType type, int ref, SBoolean iconified,
           void (*display_function)(WindowParams*, WinUnion*),
           void (*update_function)(WindowParams*, WinUnion*),
           void (*input_handler)(WindowParams*, WinUnion*, SimmEvent));
void   addNameToString(char name[], char string[], int maxStringSize);
void   adjust_main_menu();
ReturnCode alloc_func(dpFunction** func, int pts);
int    allocate_colormap_entry(GLfloat red, GLfloat green, GLfloat blue);
void   annotate_scene(Scene* scene);
void   append_if_necessary (char* str, char c);
#if OPENSMAC
MotionSequence* applyForceMattesToMotion(ModelStruct* ms, MotionSequence* motionKin, MotionSequence* motionAnalog, SBoolean addToModel);
#endif
void   apply_bone_scales_to_vertices(ModelStruct*);
void   apply_material(ModelStruct* ms, int mat, SBoolean highlight);
int    apply_motion_to_model(ModelStruct* ms, MotionSequence* motion, double value, SBoolean update_modelview, SBoolean draw_plot);
SBoolean avoid_gl_clipping_planes();
void   background(WindowParams* win_parameters, WinUnion* win_struct, SimmEvent se);
int    best_fit_color(GLfloat red, GLfloat green, GLfloat blue);
void   boundplot(PlotStruct* p);
void   build_file_filters(char fileFilter[]);
void   build_file_list_from_pattern(const char* pattern, char*** list, int* numFiles);
void   build_full_path(const char* prefPath, const char* filePath, char* fullPath);
void   calc_camera_vector(Scene* scene, GLfloat vector[]);
void   calc_foot_prints(ModelStruct* model, MotionSequence* motion, smC3DStruct* c3d);
void   calc_function_coefficients(dpFunction* sf);
void   calc_marker_events(MotionSequence* motion, smC3DStruct *c3d);
void   calc_joint_transform(int order[], double dofs[], double rmat[][4], double xaxis[],
                              double yaxis[], double zaxis[], double axis1[], double axis2[],
                              double axis3[], int mode, Direction dir, JointStruct* joint);
void   calc_motion_derivatives(MotionSequence* motion);
void   calc_muscle_moment_arms(ModelStruct* ms, int musc, GeneralizedCoord* gencoord);
ReturnCode calc_muscle_tendon_force(ModelStruct* ms, int musc, SBoolean compute_active,
                    SBoolean compute_passive, double activation,
                    SBoolean isometric);
double calc_muscle_tendon_length(ModelStruct *ms, dpMuscleStruct *muscl);
double calc_muscle_tendon_velocity(ModelStruct* ms, int musc);
void   calc_near_and_far_clip_planes(Scene* scene, double viewVecLength);
void   calc_numerical_moment_arms(ModelStruct* ms, int musc, GeneralizedCoord* gencoord);
double calc_vector_length(ModelStruct* ms, double p1[], int frame1, double p2[], int frame2);
SBoolean can_create_file(const char filename[]);
void   change_filename_suffix(const char input[], char output[], const char suffix[], int outputSize);
int    check_checkboxpanel(CheckBoxPanel* check, int mx, int my);
void   check_combobox(ComboBoxPanel* cbpanel, int mx, int my);
int    check_combopanel(ComboBoxPanel* cbpanel, int mx, int my);
ReturnCode check_definitions(ModelStruct* ms);
int    check_form(Form* form, int mx, int my);
void   check_gencoord_usage(ModelStruct* ms, SBoolean change_visibility);
double check_gencoord_wrapping(GeneralizedCoord* gc, double change);
int    check_menu(Menu* menu, int mx, int my);
double check_motion_wrapping(ModelStruct* ms, MotionSequence* motion, double change);
void   calc_segment_arms(ModelStruct* ms, int musc, dpMusclePoint *mp1, dpMusclePoint *mp2);
SBoolean check_slider(Slider* sl, SimmEvent se, void (*callback)(int, double, double), int arg1);
SBoolean check_slider_array(SliderArray* sa, SimmEvent se, void (*callback)(int, double, double));
int    check_title_area(int title_mask, int mx, int my, WindowParams* win_params,
            void** struct_ptr, ModelStruct* ms, PlotStruct* ps,
                        title_area_cb);
void   check_wrapping_points(ModelStruct *ms, dpMuscleStruct *muscl);
void   clear_preprocessor_options();
dpFunction* clone_function(ModelStruct* model, dpFunction* oldFunction, SBoolean isMuscleFunc);
void   collect_system_info(const char folder[]);
void   confirm_action(WindowParams* win_parameters, char mess[],
              void (*confirm_callback)(SBoolean));
void   confirm_input_handler(SimmEvent se);
int    convert(ModelStruct* ms, double p[], int n1, int n2);
void   convertSpacesInString(char str[]);
void   convert_string(char str[], SBoolean prependUnderscore);
int    convert_vector(ModelStruct* ms, double p[], int n1, int n2);
ReturnCode copy_default_muscle(dpMuscleStruct* from, dpMuscleStruct* to, ModelStruct* modelTo);
ReturnCode copy_default_muscle_dp(dpMuscleStruct* from, dpMuscleStruct* to, dpModelStruct* modelTo);
void   copy_dof(DofStruct* from, DofStruct* to);
void   copy_function(dpFunction* from, dpFunction* to);
void   copy_gencoord_info(GeneralizedCoord *from, SaveGencoords *to);
void   copy_material(MaterialStruct* src, MaterialStruct* dst);
ModelStruct* copy_model(ModelStruct* ms);
ReturnCode copy_muscle_path(dpMusclePathStruct *from, dpMusclePathStruct *to);
ReturnCode copy_muscle_point(dpMusclePoint* from, dpMusclePoint* to);
ReturnCode copy_muscle(dpMuscleStruct* from, dpMuscleStruct* to, dpMuscleStruct* deffrom, dpMuscleStruct *defto, ModelStruct* modelTo);
ReturnCode copy_muscle_dp(dpMuscleStruct* from, dpMuscleStruct* to, dpMuscleStruct* deffrom, dpMuscleStruct *defto, dpModelStruct* modelTo);
ReturnCode copy_nddouble(double* from, double** to, double* deffrom, double* defto);
ReturnCode copy_ndint(int* from, int** to, int* deffrom, int* defto);
ReturnCode copy_nndouble(double* from, double** to);
ReturnCode copy_nnint(int* from, int** to);
void   copy_4x4matrix(double from[][4], double to[][4]);
void   copy_1x4vector(double from[], double to[]);
void   copy_point(dpCoord3D *from, dpCoord3D *to);
int    countTokens(char str[]);
int    count_remaining_lines(FILE*, SBoolean countEmptyLines);
void   coverpage(void);
ReturnCode create_default_muscle(ModelStruct* model);
dpMuscleStruct* create_muscle(ModelStruct* model);
#if ! OPENSIM_BUILD
MotionSequence* createMotionFromTRC(ModelStruct* ms, int nq, smGenCoord gcList[], smTRCStruct *trc,
                                    int firstFrame, int lastFrame, int numRows, char motionName[],
                                    glutTRCOptions *options, glutRealtimeOptions *realtimeOptions,
                                    const smC3DStruct *c3d);
#endif
MotionSequence* createMotionStruct(ModelStruct* ms);
void   cropFormDisplay(FormItem* item, int rule, int cursorPosition);
int    define_material(ModelStruct* ms, MaterialStruct* mat);
void   deiconify_message_window();
void   delete_last_char_from_help_struct(HelpStruct* hp);
void   delete_last_line_from_help_struct(HelpStruct* hp);
void   delete_curves(PlotStruct* ps);
void   delete_display_list(GLuint display_list, ModelStruct* model);
void   delete_marker(ModelStruct* model, SegmentStruct* segment, Marker* marker, SBoolean notify);
void   delete_model(ModelStruct* ms);
void   delete_motion(ModelStruct* ms, MotionSequence* motion, SBoolean printMessage);
void   delete_muscle(ModelStruct* model, dpMuscleStruct* muscle);
void   delete_muscle_attachment_point(ModelStruct* model, int muscleIndex, int pointIndex);
void   delete_plot(int plotnum);
void   delete_polyhedron_display_list(PolyhedronStruct* ph, ModelStruct* model);
void   delete_segment_display_lists(SegmentStruct* seg, ModelStruct* model);
void   delete_window(int id);
void   destroy_main_menu();
void   determine_gencoord_type(ModelStruct* model, GeneralizedCoord* gencoord);
void   display_background(WindowParams* win_parameters, WinUnion* win_struct);
int    divide_string(char string[], char* word_array[], int max_words);
void   do_combo_box(ComboBox* cb, XYIntCoord origin, int mx, int my);
void   do_be_help();
void   do_ce_help();
void   do_ge_help();
void   do_je_help();
void   do_ke_help();
void   do_mv_help();
void   do_pm_help();
void   do_se_help();
void   do_we_help();
void   do_help_window(HelpStruct* hp, char* windowName, char* iconName,
              void (*displayFunction)(WindowParams*, WinUnion*),
              void (*updateFunction)(WindowParams*, WinUnion*),
              void (*inputHandler)(WindowParams*, WinUnion*, SimmEvent));
int    domusclemenus(ModelStruct* ms, MuscleMenu mgroup[], int mlist[],
             WindowParams* win_parameters, int mx, int my, int columns[], int *menu);
void   do_tool_window_shut(const char* toolname, int id);
double dot_vectors(double vector1[], double vector2[]);
void   draw_all_muscle_points(ModelStruct* ms, int mx, int my);
void   draw_arrow(ArrowButton* ab);
void   draw_bone(Scene* scene, ModelStruct* ms, int seg_num, int bone_num, ModelDrawOptions* mdo);
void   draw_bones(Scene* scene, ModelStruct* ms, ModelDrawOptions* mdo);
void   draw_bone_polygon(Scene* scene, ModelStruct* ms, int seg_num, int bone_num, int polygon_num);
void   draw_bounding_box_bone(PolyhedronStruct* ph, int seg_num, int bone_num, ModelDrawOptions* mdo, ModelDisplayStruct* dis);
//void   draw_combo_box(ComboBox* cb);
void   draw_combo(ComboBoxPanel *cb);
void   draw_gouraud_bone(ModelStruct* model, PolyhedronStruct* ph, ModelDisplayStruct* dis);
void   draw_flat_bone(ModelStruct* model, PolyhedronStruct* ph, ModelDisplayStruct* dis);
void   draw_marker(ModelStruct* model, SegmentStruct* seg, int segment_index, int marker_index, ModelDrawOptions* mdo);
void   draw_normals(ModelStruct* model, PolyhedronStruct* ph);
void   draw_outlined_bone(Scene* scene, ModelStruct* model, PolyhedronStruct* ph, SegmentStruct* seg, ModelDisplayStruct* dis);
void   draw_solid_fill_bone(ModelStruct* model, PolyhedronStruct* ph, int seg_num, int bone_num, ModelDrawOptions* mdo);
void   draw_wireframe_bone(ModelStruct* model, PolyhedronStruct* ph, int seg_num, int bone_num, ModelDrawOptions* mdo);
void   draw_bordered_rect(int x1, int y1, int x2, int y2, int fill_color, int border_color);
void   draw_box(GUIObjectMode mode, int x1, int y1, int x2, int y2, int col);
void   draw_checkboxpanel(CheckBoxPanel* chpanel);
void   draw_confirm_window(WindowParams* win_parameters, WinUnion* win_struct);
void   draw_diamond_box(GUIObjectMode mode, int x1, int y1, int x2, int y2, int col);
void   draw_form(Form* form);
void   draw_form_item_cursor(Form* form);
void   draw_function(dpFunction* func, float lineWidth, int color, double x1, double x2, int extrapolated_color);
void   draw_help_text(HelpStruct* hp);
void   draw_help_window(HelpStruct* hp);
void   draw_highlighted_object(Scene* scene, ModelStruct* model, PickIndex object, SBoolean highlight);
void   draw_highlighted_polygon(Scene* scene, ModelStruct* ms, int seg_num, int bone_num,
                int polygon_num, SBoolean highlight);
void   draw_ligament(ModelStruct* ms, int lignum, ModelDrawOptions* mdo);
void   draw_me_polygon(Scene* scene, ModelStruct* ms, SelectedPolygon* hp, GLenum buffer);
void   draw_menu(Menu* ms);
void   draw_model(Scene* scene, ModelStruct* ms, ModelDrawOptions* mdo);
void   draw_motion_objects(Scene* scene, ModelStruct*, ModelDrawOptions*);
void   draw_muscle_menu(Menu* ms);
void   draw_muscle_point(Scene* scene, ModelStruct* ms, int musc, int pt, ModelDrawOptions* mdo, SBoolean highlight);
void   draw_muscle_points(Scene* scene, ModelStruct* ms, int musc, ModelDrawOptions* mdo);
void   draw_muscles(Scene* scene, ModelStruct* ms, ModelDrawOptions* mdo);
void   draw_object_selection_box(Scene* scene, char text[], SBoolean for_display);
void   draw_poly_muscle(Scene* scene, ModelStruct* ms, int musc, ModelDrawOptions* mdo);
void   draw_plot(WindowParams* win_parameters, WinUnion* win_struct);
void   draw_plotkey(WindowParams* win_parameters, WinUnion* win_struct);
void   drawscene(WindowParams* win_parameters, WinUnion* win_struct);
void   draw_scene(Scene* scene, ModelStruct* model, ModelDrawOptions* mdo);
void   draw_slider(Slider* sl);
void   draw_slider_array(SliderArray* sa);
void   draw_stiffness_ellipsoids(ModelStruct* ms);
void   draw_title_area(WindowParams* win_params, ModelStruct* ms, PlotStruct* ps, int title_mask);
void   draw_world_object(Scene* scene, ModelStruct* model, int obj_num, ModelDrawOptions* mdo);
void   drawwindows(void);
int    enter_function(ModelStruct* model, int usernum, SBoolean permission_to_add);
GeneralizedCoord* enter_gencoord(ModelStruct* model, const char username[], SBoolean permission_to_add);
int    enter_gencoord_group(ModelStruct* model, const char username[], GeneralizedCoord* gencoord);
int    enter_muscle_group(ModelStruct* model, const char username[], int muscleIndex);
int    enter_joint(ModelStruct* model, const char username[], SBoolean permission_to_add);
int    enter_material(ModelStruct* ms, const char name[], EnterMode emode);
int    enter_preference(const char name[], const char value[]);
int    enter_segment(ModelStruct* model, const char username[], SBoolean permission_to_add);
int    enter_segment_group(ModelStruct* model, const char username[], int segmentIndex);
void   error(ErrorAction action, char str_buffer[]);
double evaluate_dof(ModelStruct* ms, DofStruct* var);
void   evaluate_active_movingpoints(ModelStruct* ms, dpMusclePathStruct* path);
void   evaluate_orig_movingpoints(ModelStruct* ms, dpMusclePathStruct* path);
void   exit_simm_confirm(SBoolean answer);
SBoolean file_exists(const char filename[]);
void   fill_rect(int x1, int y1, int x2, int y2);
int    find_critical_marker(smMotionStruct* motionData, smOrthoTrakMarkerSet marker);
int    find_curve_color(PlotStruct* ps, int num_curves);
void   find_ground_joint(ModelStruct* ms);
int    find_joint_between_frames(ModelStruct* ms, int from_frame, int to_frame, Direction* dir);
int    find_model_ordinal(int modelnum);
int    find_next_active_field(Form* form, int current_field, TextFieldAction tfa);
ModelStruct* find_nth_model(int modcount);
MotionSequence* find_nth_motion(ModelStruct* model, int motcount);
PlotStruct* find_nth_plot(int plotcount);
int    find_plot_ordinal(int plotnum);
smAxes find_primary_direction(double vec[]);
ReturnCode find_segment_drawing_order(ModelStruct* ms);
int    find_string_in_list(const char name[], const char* string_list[], int n);
void   find_world_coords(Scene* scene, int mx, int my, double z_value, double* wx, double* wy, double* wz);
int    findMarkerInModel(ModelStruct* ms, char name[], int *seg);
void   frame_rect(int x1, int y1, int x2, int y2);
void   freeModelStruct(ModelStruct* ms);
void   free_checkbox_panel(CheckBoxPanel* panel);
void   free_combobox_panel(ComboBoxPanel* combo);
void   free_default_muscle(dpMuscleStruct* defmusc);
void   free_form(Form* frm);
void   free_function(dpFunction* func, SBoolean freeTheFuncToo);
void   free_menu(Menu* mn);
void   free_model(int mod);
void   free_motion(MotionSequence* motion, ModelStruct* model);
void   free_motion_object(MotionObject* mo, ModelStruct* ms);
void   free_motion_object_instance(MotionObjectInstance* mi, ModelStruct* model);
void   free_muscle(dpMuscleStruct *muscle, dpMuscleStruct* dm);
void   free_muscles(ModelStruct* model);
void   free_plot(int plotnum);
SBoolean gencoord_in_path(ModelStruct* ms, int n1, int n2, GeneralizedCoord* gencoord);
dpFunction* getFunctionByUserNumber(ModelStruct* model, int userNumber);
int    getFunctionIndex(ModelStruct* model, dpFunction* function);
int    getGencoordIndex(ModelStruct* model, GeneralizedCoord* gencoord);
int    getJointIndex(ModelStruct* model, JointStruct* joint);
int    getLigamentIndex(ModelStruct* model, char lig_name[]);
dpMuscleStruct* getMuscle(ModelStruct* model, int muscleIndex);
dpFunction* getMuscleFunction(ModelStruct* model, dpMuscleStruct* muscle, int functionParam);
int    getMuscleIndex(ModelStruct* model, char muscleName[]);
smUnit getsmUnits(char *str);
const char* getsmUnitsString(smUnit units);
int    getWindowsDialogTextWidth(char text[]);
ModelStruct* get_associated_model(int window);
PlotStruct*  get_associated_plot(int window);
const char* get_bones_folder(void);
const char* get_clipboard_text(void);
const char* get_color_folder(void);
int    get_color_index(char name[]);
int    get_cursor_position(FormItem* item, int xpos, int currentPosition);
double get_double(Form* form, SimmEvent se, TextFieldAction* tfa);
char*  get_drawmode_name(DrawingMode);
const char* get_filename_from_path(const char* pathname);
ModelStruct* get_first_model_in_scene(Scene* scene);
Scene* get_first_scene_containing_model(ModelStruct* model);
#if ! ENGINE
GLfloat* get_float_conversion(ModelStruct* ms, int joint, Direction dir);
int    get_form_item_xpos(FormItem* item);
GLfloat* get_float_ground_conversion(ModelStruct* ms, int seg, GroundDirection gd);
#endif
const char* get_function_tag(dpFunctionType funcType, int beginEnd);
const char* get_function_type_name(dpFunctionType funcType);
DMatrix* get_ground_conversion(ModelStruct* ms, int seg, GroundDirection gd);
const char* get_help_folder(void);
const char* get_mocap_folder(void);
const char* get_mocap_misc_folder(void);
char*  get_simmkey_name(int keynum);
char*  get_mocap_model(void);
ModelStruct* get_model_by_name(const char name[]);
ModelStruct* get_modelstruct(void);
MotionSequence* get_motion_by_name(ModelStruct* model, const char name[]);
int    get_motion_frame_number(MotionSequence* motion, double value);
int    get_motion_number(ModelStruct* ms, MotionSequence* motion);
dpWrapObject* get_muscle_wrap_object(ModelStruct* model, dpMuscleStruct* muscle, dpMusclePoint* mpt);
int    get_name_and_value_from_string(char* string, double* value);
void   get_object_type_and_number(PickIndex object, PickIndex* type, PickIndex* num);
int*   get_path_between_frames(ModelStruct* ms, int frame1, int frame2);
#if ! ENGINE
PlotStruct* get_plot_by_name(const char name[]);
#endif
const char* get_preference(const char name[]);
void   get_pure_path_from_path (const char* fullPath, char** purePath);
Scene* get_scene(void);
int    get_simm_event(SimmEvent* se);
int    get_string(Form* form, SimmEvent se, TextFieldAction* tfa, SBoolean spacesAllowed);
ReturnCode get_string_pair(char str_buffer[], char str1[], char str2[]);
char*  get_suffix(char str[]);
char*  get_tradeshow_end(void);
void   get_transform_between_segs(ModelStruct* ms, double tmat[][4], int n1, int n2);
int    get_window_index(WindowType type, int ref);
dpWrapObject* get_wrap_object(ModelStruct* model, char username[]);
DMatrix* get_conversion(ModelStruct* ms, int joint, Direction dir);
char*  getjointvarname(int num);
int    getjointvarnum(char string[]);
int    getMusclePointSegment(dpMuscleStruct *muscle, int pointIndex);
int    getGlutWindowIndexByName(const char windowName[]);
int    getSimmWindowIndexByNameAndType(const char windowName[], WindowType type);
void   hack_tool_updates(ModelStruct* model, int model_index);
void   handle_combo_selection(WindowParams* win_parameters, ComboBoxPanel *panel, int item);
void   highlight_form_item(Form* form, int selected_item, SBoolean draw_cursor, SBoolean draw_in_front);
void   highlight_menu_item(Menu* menu, int num, OnOffSwitch state, SBoolean draw_in_front);
void   highlight_muscmenu_item(Menu* menu, int num, OnOffSwitch state, SBoolean draw_in_front);
void   import_model_archive();
void   init_color_database(void);
ReturnCode init_default_muscle(ModelStruct* ms);
ReturnCode init_dynamic_param_array(dpMuscleStruct* muscle, dpMuscleStruct* default_muscle);
void init_gencoord(GeneralizedCoord* gencoord);
ReturnCode init_gencoords(ModelStruct* ms);
void   init_global_lighting(void);
void   init_joint(ModelStruct* ms, JointStruct* jointstr);
void   init_main_menu(void);
void   init_materials(ModelStruct* ms);
ReturnCode init_model(ModelStruct* ms);
void   init_model_lighting(void);
void   init_pick_map(void);
void   init_preferences(void);
ReturnCode init_scene(Scene* sc);
void   init_segment(ModelStruct* ms, SegmentStruct* seg);
void   initialize(void);
void   initializeHTRDataStruct(smHTRData *htrData, smModel* model, smTRCStruct *trc);
void   initializeHTRFrame(smHTRFrame *htrFrame, int numSegments);
void   initializeTRC(smTRCStruct *predicted, smTRCStruct *actual);
void   initwrapobject(dpWrapObject* wo);
void   initconstraintobject(ConstraintObject *co);
void initconstraintpoint(ConstraintPoint *pt);
ReturnCode init_model_display(ModelStruct* ms);
ReturnCode init_muscle(ModelStruct* model, dpMuscleStruct* muscle, dpMuscleStruct* default_muscle);
ReturnCode initMusclePath(dpMusclePathStruct *musclepoints);
ReturnCode init_musclepoint(dpMusclePoint *mp);
ReturnCode initplot(int plotnum);
void   install_move_scene_tracker(SimmEvent);
double interpolate_function(double abscissa, dpFunction* func, Derivative deriv, double velocity, double acceleration);
void   invalidate_joint_matrix(ModelStruct* model, JointStruct* joint);
void   invalidate_joints_using_func(ModelStruct* ms, dpFunction* function);
void   invert_3x3matrix(double matrix[][3], double inverse[][3]);
void   invert_4x4matrix(double matrix[][4], double inverse[][4]);
void   invert_4x4transform(double matrix[][4], double inverse[][4]);
void   invert_matrix(double* matrix[], double* inverse[], int size);
SBoolean is_absolute_path(const char* pathname);
SBoolean is_in_demo_mode();
SBoolean is_demo_model_open(int* model_index);
SBoolean is_marker_visible(double pt[]);
SBoolean is_marker_visible_meters(double pt[]);
SBoolean is_marker_data_visible(MotionSequence* motion, int motionObject, int row);
SBoolean is_model_in_scene(Scene* scene, ModelStruct* model);
RTConnection is_model_realtime(ModelStruct* ms);
SBoolean is_muscle_function_inherited(ModelStruct* model, dpMuscleStruct* muscle, int funcNum);
SBoolean is_object_selected(ModelStruct* ms, PickIndex object);
SBoolean is_preference_on(const char value[]);
SBoolean isVisible(double pt[]);
SBoolean isC3DOtherData(int index, const smC3DStruct* c3d);
void   link_derivs_to_model(ModelStruct* ms, MotionSequence* motion);
void   load_camera_transform(Scene* scene);
void   load_double_matrix(double mat[][4]);
MotionSequence* load_motion(char filename[], int mod, SBoolean showTopLevelMessages);
dpFunction* load_simm_function(ModelStruct* model, dpFunction* func, SBoolean isMuscleFunc);
dpFunction* load_user_function(ModelStruct* model, dpFunction* func);
ReturnCode load_plot(const char* filename);
void   load_preferences_file(SBoolean verbose);
#if ! OPENSIM_BUILD
ReturnCode loadTrackedFile(ModelStruct *ms, glutTRCOptions *options, smC3DStruct *c3d, SBoolean freeData);
#endif
void   lock_model(ModelStruct *ms);
ReturnCode lookup_polyhedron(PolyhedronStruct*, char filename[], ModelStruct*);
FileReturnCode lookup_texture_file(PolyhedronStruct* ph, char filename[], ModelStruct* ms);
FileReturnCode lookup_texture_coord_file(PolyhedronStruct* ph, char filename[], ModelStruct* ms);
int    lookup_simm_key(const char* keyname);
void   lowerstr(char*);
#if ! ENGINE
void   simm_event_handler(void);
void   make_ambient_color(GLfloat old_color[], GLfloat new_color[]);
void   make_and_queue_simm_event(unsigned int event_code, int model_index, void* struct_ptr1, void* struct_ptr2, int field1, int field2);
ComboBox* make_combo_box(char* defaultName, int x1, int y1, int x2, int y2, long menu,
                         void (*menuCallback)(int menuValue, void* userData), void* userData);
void   make_function_menu(ModelStruct* ms);
void   make_gencoord_help_text(ModelStruct* ms);
void   make_force_trails(ModelStruct* model, MotionSequence* motion);
void   make_marker_trails(ModelStruct* model, MotionSequence* motion);
#endif
void   make_gencoord_popup_menus(ModelStruct* model);
void   make_ground_conversion(ModelStruct* ms, int seg);
void   make_muscle_curve_unique(ModelStruct* model, dpMuscleStruct *muscle, int funcNum);
void   make_muscle_curve_inherit_default(ModelStruct* model, dpMuscleStruct *muscle, int funcNum);
#if ! ENGINE
void   make_help_window(char help_file[], HelpStruct* hp, int num_lines);
void   make_highlight_color(GLfloat old_color[], GLfloat new_color[]);
void   make_mat_display_list(MaterialStruct*);
void   make_highlight_mat_display_list(MaterialStruct*);
void   make_message_port(void);
void   make_motion_curve_menu(ModelStruct *model, MotionSequence* motion, int mnum);
ReturnCode make_muscle_menus(ModelStruct* ms);
ReturnCode make_selectors(void);
void   make_slider(Slider* sl, SliderType type, IntBox bbox, int thumb_thickness,
           double value, double min_value, double max_value,
           double arrow_step, char label[], void* data);
void make_cropslider(CropSlider* sl, int originX, int originY, IntBox bbox, int thumb_thickness, double start_value, 
                     double end_value, double min_value, double max_value, double arrow_step, 
                     char start_label[], char end_label[], void* data);
#endif
void   make_string_lower_case(char str_buffer[]);
void   make_time_string(char** time_string);
void   make_tools(void);
void   make_wrap_cylinder(dpWrapObject* wo);
void   make_wrap_torus(dpWrapObject* wo);
void   make_conversion(ModelStruct* ms, int joint);
int    makeDir(const char aDirName[]);
void   makeMarkerMenu(ModelStruct* model);
#if ! ENGINE
void   make_specular_color(GLfloat old_color[], GLfloat new_color[]);
ReturnCode makegencform(ModelStruct* ms);
ReturnCode makedynparamsform(ModelStruct* ms);
void   makelabels(PlotStruct* ps);
#endif
ReturnCode makepaths(ModelStruct* ms);
ReturnCode malloc_function(dpFunction* func, int size);
int    mark_unconstrained_dof(ModelStruct* ms, GeneralizedCoord* gencoord, int* jnt, int* dof, SBoolean* constrained);
void*  memalloc(MEMORYBLOCK* block, int mem_size, SBoolean* moved_block);
void   message(char message_str[], int format, int xoffset);
SBoolean modelHasMuscles(ModelStruct* ms);
void   model_deletion_confirm(SBoolean answer);
void   scene_input(WindowParams* win_parameters, WinUnion* win_struct, SimmEvent se);
void   movetoframe(ModelStruct* ms, int from, int to);
ReturnCode mstrcat(char* old_str[], const char append_str[]);
#if ! MEMORY_LEAK
ReturnCode mstrcpy(char* dest_str[], const char original_str[]);
#endif
SBoolean muscle_has_force_params(dpMuscleStruct* ms);
void   musclemenu(ModelStruct* ms);
int    name_is_body_segment(ModelStruct* ms, char name[], int* motion_object, int* component,
                            dpFunctionType* functionType, double* cutoffFrequency, SBoolean stripEnd);
int    name_is_forceplate(ModelStruct* ms, char name[], int* motion_object, int* component,
                          dpFunctionType* functionType, double* cutoffFrequency, SBoolean stripEnd);
GeneralizedCoord* name_is_gencoord(char name[], ModelStruct* model, char suffix[],
                                   dpFunctionType* functionType, double* cutoffFrequency, SBoolean stripEnd);
int    name_is_marker(ModelStruct* ms, char name[], int* motion_object, int* component,
                      dpFunctionType* functionType, double* cutoffFrequency, SBoolean stripEnd);
int    name_is_muscle(ModelStruct* ms, char name[], char suffix[],
                      dpFunctionType* functionType, double* cutoffFrequency, SBoolean stripEnd);
void   new_motion_file(ModelStruct* ms, const char* fullpath);
ReturnCode newplotwindow(PlotStruct* ps, char plot_name[]);
void   normalize_motion(MotionSequence* motion, ModelStruct* model);
SBoolean onSameWrapObject(ModelStruct* ms, dpMuscleStruct* muscl, int pt1, int pt2);
ReturnCode open_c3d_file(const char c3dFile[], int modelIndex, SBoolean showDialogBox);
FILE*  open_forceplate_calibration_file(const char dataFile[]);
FILE*  open_import_variables_file(const char dataFile[]);
#if ! ENGINE
ReturnCode open_demo_arm_model();
ReturnCode open_demo_leg_model();
ReturnCode open_demo_neck_model();
void   open_demo_mac();
void   open_main_window();
ReturnCode open_model_archive(char archiveFilename[], int* modelIndex);
ReturnCode open_motion_analysis_file(const char gaitFile[], int modelIndex, int numAnalogFiles, const char* analogFiles[]);
ReturnCode open_opensim_mocap_model(glutOpenSimConverterOptions* options);
ReturnCode open_opensim_trb_file(glutOpenSimConverterOptions* options);
ReturnCode open_mocap_model(SBoolean mac_demo, const char staticFile[], SBoolean showDialogBox);
ReturnCode open_tracked_file(const char gaitFile[], int modelIndex, int numAnalogFiles, const char* analogFiles[], SBoolean showDialogBox);
PickIndex pack_bone_value(int seg, int bone);
PickIndex pack_muscle_value(int musc);
PickIndex pack_point_value(int musc, int pt);
PickIndex pack_polygon_value(int bone, int polygon);
void   pack_int_into_color(PickIndex value, GLubyte color_array[]);
PickIndex pack_world_value(int world_obj);
#endif
void   partialvelocity(ModelStruct* ms, double p[], int from, int to, GeneralizedCoord* gencoord, dpMusclePoint *mpt, double result[]);
#if ! ENGINE
SBoolean peek_simm_event(SimmEvent* se);
int    place_musclemenu(int menunum, int xoffset, int yoffset, int columns[],
                int columns_needed, int column_width, MuscleMenu mgroup[]);
void   place_slider(Slider*, IntBox bbox);
void   plot_depthchange(int action, int win);
void   plotinc(double start, double end, int* numticks, double* starttick,
           double* step, char format[]);
void   plotinput(WindowParams* win_parameters, WinUnion* win_struct, SimmEvent se);
void   plotkeyinput(WindowParams* win_parameters, WinUnion* win_struct, SimmEvent se);
void   plotlabels(WindowParams* win_parameters, PlotStruct* p);
void   plotvport(WindowParams* win_parameters, PlotStruct* p);
void   plotxadj(WindowParams* win_parameters, PlotStruct* p);
#endif
void   popframe(ModelStruct* ms);
ReturnCode post_init_muscle_path(dpMusclePathStruct* path, int numWrapStructs);
void   post_motion_event(ModelStruct* ms, MotionSequence* motion, int eventCode);
void   post_process_bones(ModelStruct* ms);
void   calc_transformation(ModelStruct* ms, int from, int to, DMatrix mat);
FILE*  preprocess_file(char infile[], const char outfile[]);
void   print_duration(char text_string[]);
void   print_4x4matrix(double matrix[][4]);
void   print_simm_event(SimmEvent se);
void   print_simm_event_queue();
void   print_time(void);
void   printmuscle(dpMuscleStruct* musc);
void   printpaths(ModelStruct* ms);
int    purge_simm_events_for_model(int model_index, unsigned int event_code);
int    purge_simm_events_for_struct(void* struct_ptr, unsigned int event_code);
void   pushframe(ModelStruct* ms);
void   queue_model_redraw(ModelStruct* model);
void   queue_window_redraw(WindowType type, int ref);
void   queue_simm_event(SimmEvent se);
ReturnCode read_color_file(SBoolean refreshWindows, SBoolean verbose);
ReturnCode read_double(FILE*, double*);
ReturnCode read_double_array(FILE* fp, const char ending[], const char name[], dpFunction* func);
ReturnCode read_double_tab(FILE*, double*);
ReturnCode read_drawmode(FILE*, DrawingMode*);
int    read_line(FILE* fp, char str_buffer[]);
ReturnCode read_material(ModelStruct *ms, FILE* fp);
ReturnCode read_model_file(Scene* scene, ModelStruct* ms, char filename[], SBoolean showTopLevelMessages);
ReturnCode read_muscle_attachment_points(ModelStruct* ms, FILE* fp, dpMuscleStruct *muscle);
ReturnCode read_muscle_file(ModelStruct* ms, char filename[], SBoolean* file_exists, SBoolean showTopLevelMessages);
int*   read_muscle_groups(ModelStruct* ms, FILE* fp, int* num_groups, int muscle_number);
int    read_nonempty_line(FILE* fp, char str_buffer[]);
CurveStruct* readplotfile(FILE* fp);
int    read_string(FILE* fp, char str_buffer[]);
ReturnCode read_world_object(ModelStruct* ms, FILE* fp);
ReturnCode read_function(ModelStruct* ms, FILE* fp, SBoolean muscleFunc, dpFunctionType funcType, const char endTag[]);
FileReturnCode read_tiff_image(const char* filename, int* width, int* height, unsigned** image);
ReturnCode realloc_function(dpFunction* func, int size);
void   recalc_constraint_xforms(ConstraintObject *co);
void   remove_preference(const char name[]);
ReturnCode rename_muscle(ModelStruct* model, dpMuscleStruct* muscle, char newName[]);
#if ! ENGINE
ToolStruct* register_tool(int struct_size, unsigned int event_mask,
              void (*event_handler)(SimmEvent),
              void (*command_handler)(char*),
              SBoolean (*query_handler)(QueryType, void*),
              char name[], int* ref_number);
void   replace_help_text(char new_help_file[], HelpStruct* hp);
void   reshapewindow(void);
void   resize_model_display(Scene* scene, ModelStruct*);
#endif
ReturnCode restore_all_muscles(ModelStruct* ms);
ReturnCode restore_default_muscle(ModelStruct* ms);
void   restore_dofs(void);
void   restore_function(ModelStruct* model, dpFunction* function);
ReturnCode restore_muscle(ModelStruct* ms, dpMuscleStruct *muscle);
ReturnCode restore_muscle_groups(ModelStruct* ms);
ReturnCode   save_all_muscles(ModelStruct* ms, SBoolean verbose);
ReturnCode   save_default_muscle(ModelStruct* ms);
void   save_function(ModelStruct* model, dpFunction* function);
ReturnCode   save_muscle(ModelStruct* ms, dpMuscleStruct *muscle, SBoolean verbose);
void   save_muscle_groups(ModelStruct* ms);
void   save_preferences_file(SBoolean verbose);
void   scale_model(ModelStruct*, const dpCoord3D* seg_scales, ScaleModelOptions* options);
void   select_form_region(Form* form, int mx, SimmEvent se);
void   select_muscle_attachment(ModelStruct* model, dpMuscleStruct* muscle, int pointIndex, SBoolean state);
void   set_clipboard_text(const char text[]);
void   set_combobox_item(ComboBox* cb, int menuValue);
void   set_gencoord_info(ModelStruct* ms);
int    set_gencoord_value(ModelStruct* ms, GeneralizedCoord* gencoord, double value, SBoolean solveLoops);
void   set_gencoord_velocity(ModelStruct* ms, GeneralizedCoord* gencoord, double value);
void   set_hourglass_cursor(double percent);
void   set_interpolated_color(int col1, int col2, double factor);
void   set_ortho2(double x1, double x2, double y1, double y2);
void   set_ortho2i(GLint ortho[]);
void   set_ortho2o(Ortho box);
void   set_plot_cursors(ModelStruct *model, MotionSequence* motion);
void   set_plot_ortho(PlotStruct* p);
void   set_prefform(void);
void   set_qs(ModelStruct* ms);
void   set_us(ModelStruct* ms);
void   set_viewport(int x1, int y1, int xsize, int ysize);
void   setMuscleFunction(ModelStruct* model, dpMuscleStruct* muscle, int functionParam, dpFunction* function);
void   setMusclePointSegment(dpMuscleStruct *muscle, int pointIndex, int newSeg);
void   setmvmenus(WindowParams* win_parameters, WinUnion* win_struct);
ReturnCode setup_motion_derivatives(MotionSequence* motion);
//dkb void   setup_muscle_wrapping(dpMuscleStruct* muscl);
int    shape_window(WindowParams* win, SBoolean setortho);
void   show_musclemenus(ModelStruct* ms, MuscleMenu mgroup[], int mlist[]);
void   show_opening_and_saving_help();
void   show_tutorial(int whichTutorial);
SBoolean show_window(const char* windowName, WindowType type);
#if ! MEMORY_LEAK
void*  simm_calloc(unsigned num_elements, unsigned elem_size);
#endif
void   simm_color(int index);
void   simm_exit(void);
FILE*  simm_fopen(const char* name, const char* mode);
FILE*  simm_lookup_file(char* pathList, const char* fileName, const char* mode);
#if ! MEMORY_LEAK
void*  simm_malloc(unsigned mem_size);
#endif
int    simm_open(const char *name, int oflag, ...);
void   simm_pre_exit(void);
int    simm_printf(SBoolean hilite_text, const char* format, ...);
#if ! MEMORY_LEAK
void*  simm_realloc(void* ptr, unsigned mem_size, ReturnCode* rc);
#endif
void   size_plotkey(PlotStruct* ps);
void   size_model(Scene* scene, ModelStruct*, BoundingCube* model_bounds);
void   size_model_display(Scene* scene, ModelStruct* ms, int* suggested_width, int* suggested_height);
void   simmPrintMultiLines(char string[], SBoolean hilite, int lineSize, int pixelIndent);
void   smooth_motion(MotionSequence* motion, ModelStruct* model, double cutoff_frequency);
void   sort_events(MotionSequence* motion, int* index);
void   start_simm();
void   start_timer(void);
void   stop_timer(void);
void   storeDoubleInForm(FormItem* item, double value, int decimal_places);
void   storeIntInForm(FormItem* item, int value);
void   storeStringInForm(FormItem* item, const char str[]);
void   strcat3(char dest[], const char str1[], const char str2[], const char str3[], int destSize);
int    strings_equal_case_insensitive(const char str1[], const char str2[]);
int    strings_equal_n_case_insensitive(const char str1[], const char str2[], int n);
void   strip_brackets_from_string(char name[]);
void   strip_trailing_white_space(char string[]);
int    strrcspn(const char* string, const char* strCharSet);
void   toggle_continuous_motion(ModelStruct* model, OnOffSwitch state);
void   update_background(WindowParams* win_parameters, WinUnion* win_struct);
void   update_scene(WindowParams* win_parameters, WinUnion* win_struct);
void   update_drawmode_menus(ModelStruct* ms);
void   update_drawplot(WindowParams* win_parameters, WinUnion* win_struct);
void   update_drawplotkey(WindowParams* win_parameters, WinUnion* win_struct);
void   update_ligament_path(ModelStruct* model, LigamentStruct* ligament);
void   updatemodelmenu(void);
void   updateplotmenu(void);
void   unlock_model(ModelStruct *ms);
void   unpack_bone_value(PickIndex value, int* seg, int* bone);
void   unpack_muscle_value(PickIndex value, int* musc);
void   unpack_point_value(PickIndex value, int* musc, int* pt);
void   unpack_polygon_value(PickIndex value, int* bone, int* polygon);
void   unpack_int_from_color(PickIndex *value, GLubyte color_array[]);
void   unpack_world_value(PickIndex value, int* world_obj);
void   update_muscle_path(ModelStruct* model, dpMuscleStruct* muscle);
void   upperstr(char*);
void   verify_key(void);
int    which_window(int gid);
ReturnCode write_motion(MotionSequence *motion, const char filename[]);
ReturnCode write_opensim_model(ModelStruct* ms, char filename[], char geometryDirectory[],
                               const char* markerSetOut, int angleUnits);
#if EXPERIMENTAL
ReturnCode write_motion_with_deriv(MotionSequence *motion, const char filename[]);
#endif
void   write_motion_object(MotionObject*, FILE*);
ReturnCode write_trc_file(smMotionStruct* data, const char filename[]);

char* get_snapshot_filename(Scene* scene, char* buf);
void  write_tif_frame(Scene* scene);

/******* Function Prototypes for About Box window *******/
void   show_about_box();

/******* Function Prototypes for JointEditor *******/
void make_jointeditor(int rootWindowX, int rootWindowY, SBoolean iconified);
void jointeditor(WindowParams* win_parameters, WinUnion* win_struct, SimmEvent se);
void display_jointeditor(WindowParams* win_parameters, WinUnion* win_struct);
void update_jointeditor(WindowParams* win_parameters, WinUnion* win_struct);
void je_simm_event_handler(SimmEvent se);
SBoolean je_query_handler(QueryType, void* data);
void draw_je_help_window(WindowParams* win_parameters, WinUnion* win_struct);
void je_help_input(WindowParams* win_parameters, WinUnion* win_struct, SimmEvent se);
void move_je_help_text(int dummy_int, double slider_value, double delta);

/******* Function Prototypes for GencoordEditor *******/
void make_gencoordeditor(int rootWindowX, int rootWindowY, SBoolean iconified);
void gencoordeditor(WindowParams* win_parameters, WinUnion* win_struct, SimmEvent se);
void display_gencoordeditor(WindowParams* win_parameters, WinUnion* win_struct);
void update_gencoordeditor(WindowParams* win_parameters, WinUnion* win_struct);
void ge_simm_event_handler(SimmEvent se);
SBoolean ge_query_handler(QueryType, void* data);
void draw_ge_help_window(WindowParams* win_parameters, WinUnion* win_struct);
void ge_help_input(WindowParams* win_parameters, WinUnion* win_struct, SimmEvent se);
void move_ge_help_text(int dummy_int, double slider_value, double delta);
void ge_set_default_value(GeneralizedCoord* gencoord, double value);

/******* Function Prototypes for MusclePointEditor *******/
void make_muscleeditor(int rootWindowX, int rootWindowY, SBoolean iconified);
void muscleeditor(WindowParams* win_parameters, WinUnion* win_struct, SimmEvent se);
void display_muscleeditor(WindowParams* win_parameters, WinUnion* win_struct);
void update_muscleeditor(WindowParams* win_parameters, WinUnion* win_struct);
void me_simm_event_handler(SimmEvent se);
SBoolean me_query_handler(QueryType, void* data);
void draw_me_help_window(WindowParams* win_parameters, WinUnion* win_struct);
void me_help_input(WindowParams* win_parameters, WinUnion* win_struct, SimmEvent se);
void move_me_help_text(int dummy_int, double slider_value, double delta);
public void slide_me(int arg1, double value, double delta);

/******* Function Prototypes for MarkerEditor *******/
void make_markereditor(int rootWindowX, int rootWindowY, SBoolean iconified);
void markereditor(WindowParams* win_parameters, WinUnion* win_struct, SimmEvent se);
void display_markereditor(WindowParams* win_parameters, WinUnion* win_struct);
void update_markereditor(WindowParams* win_parameters, WinUnion* win_struct);
void ke_simm_event_handler(SimmEvent se);
SBoolean ke_query_handler(QueryType query, void* data);
void draw_ke_help_window(WindowParams* win_parameters, WinUnion* win_struct);
void ke_help_input(WindowParams* win_parameters, WinUnion* win_struct, SimmEvent se);
void move_ke_help_text(int dummy_int, double slider_value, double delta);


/******* Function Prototypes for PlotMaker *******/
void make_plotmaker(int rootWindowX, int rootWindowY, SBoolean iconified);
void plotmaker(WindowParams* win_parameters, WinUnion* win_struct, SimmEvent se);
void display_plotmaker(WindowParams* win_parameters, WinUnion* win_struct);
void update_plotmaker(WindowParams* win_parameters, WinUnion* win_struct);
void pm_simm_event_handler(SimmEvent se);
SBoolean pm_query_handler(QueryType, void* data);
void draw_pm_help_window(WindowParams* win_parameters, WinUnion* win_struct);
void pm_help_input(WindowParams* win_parameters, WinUnion* win_struct, SimmEvent se);
void move_pm_help_text(int dummy_int, double slider_value, double delta);
public void slide_pm(int arg1, double value, double delta);
public void pm_command_handler(char command[]);

/* -------------------------------------------------------------------------
   CURVE_VALUE - this macro is the only way to access a curve's values.
      It allows curve value arrays to be circularized so that realtime data
      can be continuously streamed into them.   -- added KMS 2/25/00
---------------------------------------------------------------------------- */
int circularize_curve_index(CurveStruct*, void* array, int val);

#define CIRCULARIZE_CURVE_INDEX(CURVE, ARRAY, INDEX) \
   (((CURVE)->realtime_circular_index == 0 /*|| (CURVE)->ARRAY == (CURVE)->xvalues*/) ? \
        (INDEX) : circularize_curve_index(CURVE, (CURVE)->ARRAY, INDEX))

#define CURVE_VALUE(CURVE, ARRAY, INDEX) \
   (((double*) (CURVE)->ARRAY)[CIRCULARIZE_CURVE_INDEX(CURVE, ARRAY, INDEX)])


/******* Function Prototypes for Model Viewer *******/
void make_modelviewer(int rootWindowX, int rootWindowY, SBoolean iconified);
void modelviewer(WindowParams* win_parameters, WinUnion* win_struct, SimmEvent se);
void display_modelviewer(WindowParams* win_parameters, WinUnion* win_struct);
void update_modelviewer(WindowParams* win_parameters, WinUnion* win_struct);
void mv_simm_event_handler(SimmEvent se);
SBoolean mv_query_handler(QueryType, void* data);
void draw_mv_help_window(WindowParams* win_parameters, WinUnion* win_struct);
void mv_help_input(WindowParams* win_parameters, WinUnion* win_struct, SimmEvent se);
void move_mv_help_text(int dummy_int, double slider_value, double delta);
void slide_gencoord(int genc, double value, double delta);
void slide_mv(int arg1, double value, double delta);
void reposition_gencoord_sliders(ModelStruct*);
void mv_startstop_printf(ModelStruct*, SBoolean hiliteButton, const char* format, ...);

/******* Function Prototypes for Muscle Editor *******/
void makeUniqueMuscleName(ModelStruct* model, char name[]);
SBoolean isValidMuscleName(ModelStruct* model, char name[]);
public void me_command_handler(char command[]);

/******* Function Prototypes for file writing *******/
void write_plot_file(PlotStruct*, PLOTFILEFORMAT, const char* fullpath);
void write_model_joints(ModelStruct*, const char* fullpath, SBoolean showMessage);
void write_model_muscles(ModelStruct*, const char* fullpath, SBoolean showMessage);


/******* Function Prototypes for Preferences *******/
void make_preferences(int rootWindowX, int rootWindowY, SBoolean iconified);
void preferences(WindowParams* win_parameters, WinUnion* win_struct, SimmEvent se);
void display_preferences(WindowParams* win_parameters, WinUnion* win_struct);
void update_preferences(WindowParams* win_parameters, WinUnion* win_struct);
void pr_simm_event_handler(SimmEvent se);
SBoolean pr_query_handler(QueryType, void* data);
void draw_pr_help_window(WindowParams* win_parameters, WinUnion* win_struct);
void pr_help_input(WindowParams* win_parameters, WinUnion* win_struct, SimmEvent se);
void move_pr_help_text(int dummy_int, double slider_value, double delta);


/******* Function Prototypes for GaitLoader *******/
void make_gaitloader(int rootWindowX, int rootWindowY, SBoolean iconified);
void init_gaitloader_struct(void* buffer, int ref);
void gaitloader(WindowParams* win_parameters, WinUnion* win_struct, SimmEvent se);
void display_gaitloader(WindowParams* win_parameters, WinUnion* win_struct);
void update_gaitloader(WindowParams* win_parameters, WinUnion* win_struct);
void gl_simm_event_handler(SimmEvent se);
SBoolean gl_query_handler(QueryType, void* data);
void draw_gl_help_window(WindowParams* win_parameters, WinUnion* win_struct);
void gl_help_input(WindowParams* win_parameters, WinUnion* win_struct, SimmEvent se);
void move_gl_help_text(int dummy_int, double slider_value, double delta);


/******* Function Prototypes for the Bone Editor *******/
public void make_boneeditor(int rootWindowX, int rootWindowY, SBoolean iconified);
public void boneeditor(WindowParams* win_parameters, WinUnion* win_struct, SimmEvent se);
public void display_boneeditor(WindowParams* win_parameters, WinUnion* win_struct);
public void update_boneeditor(WindowParams* win_parameters, WinUnion* win_struct);
public void be_simm_event_handler(SimmEvent se);
SBoolean be_query_handler(QueryType, void* data);
public void slide_boneed(int arg1, double value, double delta);
public void draw_be_help_window(WindowParams* win_parameters, WinUnion* win_struct);
public void move_be_help_text(int dummy_int, double slider_value, double delta);
public void be_help_input(WindowParams* win_parameters, WinUnion* win_struct, SimmEvent se);
public void draw_bone_editor_stuff(Scene* scene, ModelStruct* ms, ModelDrawOptions* mdo);
public void be_command_handler(char command[]);


/******* Function Prototypes for Motion Module help *******/
SBoolean is_mac_demo_open(void);

/******* Function Prototypes for WrapEditor *******/
void make_wrapeditor(int rootWindowX, int rootWindowY, SBoolean iconified);
void wrapeditor(WindowParams* win_parameters, WinUnion* win_struct, SimmEvent se);
void display_wrapeditor(WindowParams* win_parameters, WinUnion* win_struct);
void update_wrapeditor(WindowParams* win_parameters, WinUnion* win_struct);
void we_simm_event_handler(SimmEvent se);
SBoolean we_query_handler(QueryType, void* data);
void draw_we_help_window(WindowParams* win_parameters, WinUnion* win_struct);
void we_help_input(WindowParams* win_parameters, WinUnion* win_struct, SimmEvent se);
void move_we_help_text(int dummy_int, double slider_value, double delta);
public void slide_we(int arg1, double value, double delta);

/******* Function Prototypes for ConstraintEditor *******/
void make_constrainteditor(int rootWindowX, int rootWindowY, SBoolean iconified);
void constrainteditor(WindowParams* win_parameters, WinUnion* win_struct, SimmEvent se);
void display_constrainteditor(WindowParams* win_parameters, WinUnion* win_struct);
void update_constrainteditor(WindowParams* win_parameters, WinUnion* win_struct);
void ce_simm_event_handler(SimmEvent se);
SBoolean ce_query_handler(QueryType, void* data);
void draw_ce_help_window(WindowParams* win_parameters, WinUnion* win_struct);
void ce_help_input(WindowParams* win_parameters, WinUnion* win_struct, SimmEvent se);
void move_ce_help_text(int dummy_int, double slider_value, double delta);
public void slide_ce(int arg1, double value, double delta);


/******* Function Prototypes for Segment Editor *********/
public void se_simm_event_handler(SimmEvent sevent);
SBoolean se_query_handler(QueryType, void* data);
void display_segmenteditor(WindowParams* win_parameters, WinUnion* win_struct);
void update_segmenteditor(WindowParams* win_parameters, WinUnion* win_struct);
void make_segmenteditor(int rootWindowX, int rootWindowY, SBoolean iconified);
void segmenteditor(WindowParams* win_parameters, WinUnion* win_struct, SimmEvent se);
void draw_se_help_window(WindowParams* win_parameters, WinUnion* win_struct);
void se_help_input(WindowParams* win_parameters, WinUnion* win_struct, SimmEvent se);
void move_se_help_text(int dummy_int, double slider_value, double delta);
//public void slide_se(int arg1, double value, double delta);

/******* Function Prototypes for DP Tool *********/
public void dp_simm_event_handler(SimmEvent se);
SBoolean dp_query_handler(QueryType, void* data);
void display_dptool(WindowParams* win_parameters, WinUnion* win_struct);
void update_dptool(WindowParams* win_parameters, WinUnion* win_struct);
void make_dptool(int rootWindowX, int rootWindowY, SBoolean iconified);
void dptool(WindowParams* win_parameters, WinUnion* win_struct, SimmEvent se);
void draw_dptool_help_window(WindowParams* win_parameters, WinUnion* win_struct);
void dptool_help_input(WindowParams* win_parameters, WinUnion* win_struct, SimmEvent se);
void move_dptool_help_text(int dummy_int, double slider_value, double delta);
void do_dptool_help(void);
//public void slide_dp(int arg1, double value, double delta);


/******* Function Prototypes for DeformEditor *******/
void write_deformities(FILE*, ModelStruct*);
void write_deform(FILE*, SegmentStruct*, DeformObject*);
ReturnCode read_deform(FILE*, SegmentStruct*, int segmentnum);
ReturnCode read_deformity(ModelStruct*, FILE*);

/******* Function Prototypes for MotionEditor *******/
void make_motioneditor(int rootWindowX, int rootWindowY, SBoolean iconified);
void motioneditor(WindowParams* win_parameters, WinUnion* win_struct, SimmEvent se);
void display_motioneditor(WindowParams* win_parameters, WinUnion* win_struct);
void update_motioneditor(WindowParams* win_parameters, WinUnion* win_struct);
void mt_simm_event_handler(SimmEvent se);
SBoolean mt_query_handler(QueryType, void* data);
void draw_mt_help_window(WindowParams* win_parameters, WinUnion* win_struct);
void mt_help_input(WindowParams* win_parameters, WinUnion* win_struct, SimmEvent se);
void move_mt_help_text(int dummy_int, double slider_value, double delta);
public void slide_mt(int arg1, double value, double delta);
public void mt_command_handler(char command[]);


#if INCLUDE_COMMAND_PARSER
/******* Function Prototypes for the Command Parser *******/
public void make_commandparser(int rootWindowX, int rootWindowY, SBoolean iconified);
public void cp_simm_event_handler(SimmEvent se);
public SBoolean cp_query_handler(QueryType query, void* data);
public void commandparser(WindowParams* win_parameters, WinUnion* win_struct, SimmEvent se);
public void display_commandparser(WindowParams* win_parameters, WinUnion* win_struct);
public void update_commandparser(WindowParams* win_parameters, WinUnion* win_struct);
public void move_command_text(int dummy_int, double slider_value, double delta);
public void draw_cp_help_window(WindowParams* win_parameters, WinUnion* win_struct);
public void cp_help_input(WindowParams* win_parameters, WinUnion* win_struct, SimmEvent se);
public void move_cp_help_text(int dummy_int, double slider_value, double delta);
public int execute_simm_command(void);
#endif

/******* Function Prototypes for the Message Window *******/
public void draw_message_window(WindowParams* win_parameters, WinUnion* win_struct);
public void update_message_window(WindowParams* win_parameters, WinUnion* win_struct);
public void messages_input(WindowParams* win_parameters, WinUnion* win_struct, SimmEvent se);
public void move_message_text(int dummy_int, double slider_value, double delta);

/******* Function Prototypes for the persistent SIMM state mechanism *******/
void load_simm_state();
void save_simm_state();

const char* get_simm_state_value(const char* name);
void        set_simm_state_value(const char* name, const char* value);


int convertNEW(ModelStruct* ms, double p[], int path[], int len);
int findHighestUserFuncNum(ModelStruct* ms);
int countUsedFunctions(ModelStruct* ms);

/********************** Function Prototypes in IK_solver *******************/
int lmdif_(int (*fcn) (void *data, int *m, int *n, double x[], double fvec[],
           int *iflag), integer *m, integer *n, doublereal *x, 
           doublereal *fvec, 
           doublereal *ftol, doublereal *xtol, doublereal *gtol, 
           integer *maxfev, 
    doublereal *epsfcn, doublereal *diag, integer *mode, doublereal *factor, 
   integer *nprint, integer *info, 
   integer *nfev, doublereal *fjac, integer *ldfjac, integer *ipvt, 
    doublereal *qtf, doublereal *wa1, doublereal *wa2, doublereal *wa3, 
   doublereal *wa4, logical *firstframe, doublereal *qrjac, 
   doublereal *rdiag, void *data);
int hessian_(integer *nres, int *nq);
int hessdecomp_(integer *ierror, int *nq);
int dpodi_(doublereal *a, integer *lda, integer *n, doublereal *det, 
           integer *job);
int dpofa_(doublereal *a, integer *lda, integer *n, integer *info);


/**************** Function Prototypes for loop and constraint solving *******/
SBoolean loopsToSolve(ModelStruct *model);
SBoolean constraintsToSolve(ModelStruct *model);

/************** Function Prototypes in IK.c **********************/
SBoolean resolveClosedLoops(ModelStruct *ms, int q_num, double *q_value);
void calculateResiduals(void *data, int *nres, int *ndof, double q[], 
                    double resid[], int *iflag);

/**************** Function Prototypes in Loop.c ************************/
int getLoopPath(int seg, int *path, int dir, LoopStruct *loop);
int setGencoordValue2(ModelStruct *ms, GeneralizedCoord* gencoord, double value);
void checkGencoordRange(GeneralizedCoord* gencoord, double *value);
SBoolean makeLoops(ModelStruct *ms);
void markLoopJoints(ModelStruct *ms);

/**************** Function Prototypes in constraint.c ************************/
void calculateConstraintResids(void *data, int numQ, double q[], int numResid, 
                               double resid[], int startIndex, int endIndex,
                               int *iflag);
void updateConstraintInfo(ModelStruct *ms);
void markAffectedGencoords(ModelStruct *ms);


/**************** Function Prototypes in LCSolver.c ************************/
void solve_initial_loops_and_constraints(ModelStruct *model);
void recalc_default_loops_and_constraints(ModelStruct *ms);
void solveAllLoopsAndConstraints(ModelStruct *ms, LoopStatus *loopStatus,  
                                 ConstraintStatus *constraintStatus, 
                                 SBoolean enforce_constraints);
SBoolean solveLCAffectedByJNT(ModelStruct *ms, int joint, LoopStatus *loopStatus,  ConstraintStatus *constraintStatus);
SBoolean solveLCAffectedByGC(ModelStruct *ms, GeneralizedCoord* controlled_gc, double *gc_value);
void approveNewDefaultGCs(ModelStruct *ms);
void evaluateLoopsAndConstraintsInCurrentConfiguration(ModelStruct *ms, LoopStatus *loopStatus, 
                                 ConstraintStatus *constraintStatus, 
                                 SBoolean enforce_constraints);
int convert2(ModelStruct *ms, double p[], int n1, int n2);

#endif /*FUNCTIONS_H*/

