/*******************************************************************************

   SDFUNCTIONS.H

   Author: Peter Loan

   Date: 17-JUL-08

   Copyright (c) 2008 MusculoGraphics, Inc.
   All rights reserved.

*******************************************************************************/

#ifndef SDFUNCTIONS
#define SDFUNCTIONS

/* defined in sdf.c */
ReturnCode make_sdfast_model(int mod, char filename[], SBoolean write_file,
                             int addQuestionMarks);
void make_sdfast_joint(int mod, FILE** fp, JointSDF* jntsdf, int jointnum, int sdnum,
			              SegmentSDF segs[], int* dofcount,
			              int* constrainedcount, SBoolean write_file, int addQuestionMarks);
char* make_sdfast_seg_name(char seg_name[], int times_split);
SBoolean valid_sdfast_model(ModelStruct* ms);
void write_dllparams(char filename[], ModelStruct* model);
void write_forparams(char filename[], ModelStruct* model);
void write_invparams(char filename[], ModelStruct* model);

/* defined in sdftools.c */
dpJointType identify_joint_type(int mod, int jointnum);
void find_sdfast_joint_order(ModelStruct* ms, JointSDF jnts[],
				                 SegmentSDF segs[], int joint_order[]);
int find_nth_rotation(JointStruct* jnt, int n);
int find_rotation_axis(JointStruct* jnt, double axis[]);
int find_translation_axis(JointStruct* jnt, double axis[], DofType type, int num);
char*  get_joint_type_name(dpJointType type, Direction dir);
JointStruct* find_nth_q_joint(int mod, int n);
void name_dofs(int mod);
void check_dynamic_params(ModelStruct* ms);
ReturnCode copyModelToDPModel(ModelStruct* ms, dpModelStruct* dp, int muscleList[]);
int get_sd_seg_num(char simm_name[]);
DofStruct* find_unconstrained_sd_dof(int mod, int gc);
DofStruct* find_nth_q_dof(int mod, int n);

/* defined in sdcode.c */
ReturnCode write_sdheader_file(int mod, char filename[]);
ReturnCode write_sdforward(int mod, char filename[]);
ReturnCode write_sdinverse(int mod, char filename[]);

#endif /*SDFUNCTIONS_H*/

