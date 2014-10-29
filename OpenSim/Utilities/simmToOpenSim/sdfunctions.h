/*******************************************************************************

   SDFUNCTIONS.H

   Author: Peter Loan

   Date: 17-JUL-08

   Copyright (c) 2008 MusculoGraphics, Inc.
   All rights reserved.

*******************************************************************************/

#ifndef SDFUNCTIONS
#define SDFUNCTIONS

#define SDFAST_GROUND 0  /* indices into the ground_name[] array */
#define SIMBODY_GROUND 1

/* defined in sdf.c */
ReturnCode make_sdfast_model(ModelStruct* ms, char filename[], SBoolean write_file,
                             int addQuestionMarks);
void make_sdfast_joint(ModelStruct* ms, FILE* fp, JointSDF* jntsdf, int jointnum, int sdnum,
                          SegmentSDF segs[], int* dofcount,
                          int* constrainedcount, SBoolean write_file, int addQuestionMarks);
char* make_sdfast_seg_name(char seg_name[], int times_split);
SBoolean valid_sdfast_model(ModelStruct* ms);
void write_dllparams(char filename[], ModelStruct* model);
void write_forparams(char filename[], ModelStruct* model);
void write_invparams(char filename[], ModelStruct* model);

/* defined in sdftools.c */
dpJointType identify_joint_type(ModelStruct* ms, int jointnum);
void find_sdfast_joint_order(ModelStruct* ms, JointSDF jnts[],
                                 SegmentSDF segs[], int joint_order[], int ground_name_index);
int find_nth_rotation(JointStruct* jnt, int n);
int find_rotation_axis(JointStruct* jnt, double axis[]);
int find_translation_axis(JointStruct* jnt, double axis[], DofType type, int num);
char*  get_joint_type_name(dpJointType type, Direction dir);
JointStruct* find_nth_q_joint(ModelStruct* ms, int n);
void name_dofs(ModelStruct* ms);
void check_dynamic_params(ModelStruct* ms);
dpModelStruct* copyModelToDPModel(ModelStruct* ms, int muscleList[]);
dp420ModelStruct* copyModelToDP420Model(ModelStruct* ms, int muscleList[]);
int get_sd_seg_num(char simm_name[]);
DofStruct* find_unconstrained_sd_dof(ModelStruct* model, GeneralizedCoord* gencoord);
DofStruct* find_nth_q_dof(ModelStruct* ms, int n);
void freeDPModelStruct(dpModelStruct* dp);
void freeDP420ModelStruct(dp420ModelStruct* dp);
void freeDPPolyhedron(dpPolyhedronStruct* ph);
void copyPolyhedronToDPPolyhedron(PolyhedronStruct* from, dpPolyhedronStruct** to,
                                             int SimmSegNum, int SDSegNum, ModelStruct* ms);

/* defined in sdcode.c */
ReturnCode write_sdheader_file(ModelStruct* ms, char filename[]);
ReturnCode write_sdforward(ModelStruct* ms, char filename[]);
ReturnCode write_sdinverse(ModelStruct* ms, char filename[]);

#endif /*SDFUNCTIONS_H*/

