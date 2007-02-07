/*******************************************************************************

   sdfor.c

   Created by NMBLTS (from model Dynamic Mocap Model)

   Time of creation: 02/07/2007 05:08:19 PM

   Description: This file contains the routines needed to perform a forward
      dynamics simulation of an SD/FAST model. The specific routines that it
      contains depend on the SIMM model from which this code was generated.

*******************************************************************************/

#include "universal.h"
#include "model.h"

/*************** DEFINES (for this file only) *********************************/
#define BAUMGARTE_STAB 20

/* Defines for the joints, Qs, and body segments are now found in model.h */



/*************** STATIC GLOBAL VARIABLES (for this file only) *****************/

static dpSplineFunction knee_flexion_r_tx_func;
static dpSplineFunction knee_flexion_r_ty_func;
static dpSplineFunction tib_pat_r_tx_func;
static dpSplineFunction tib_pat_r_ty_func;
static dpSplineFunction tib_pat_r_r1_func;
static dpSplineFunction knee_flexion_l_tx_func;
static dpSplineFunction knee_flexion_l_ty_func;
static dpSplineFunction tib_pat_l_tx_func;
static dpSplineFunction tib_pat_l_ty_func;
static dpSplineFunction tib_pat_l_r1_func;

static double q_restraint_func1_data[][2] = {
{0.0000000000, 0.0000000000},
{0.1745329252, 20.0000000000},
{0.5235987756, 70.0000000000}
};


static double q_restraint_func2_data[][2] = {
{0.0000000000, 0.0000000000},
{0.1745329252, 20.0000000000},
{0.5235987756, 70.0000000000}
};


static double q_restraint_func3_data[][2] = {
{0.0000000000, 0.0000000000},
{0.1745329252, 20.0000000000},
{0.5235987756, 70.0000000000}
};


static double q_restraint_func4_data[][2] = {
{0.0000000000, 0.0000000000},
{0.1745329252, 20.0000000000},
{0.5235987756, 70.0000000000}
};


static double q_restraint_func5_data[][2] = {
{0.0000000000, 0.0000000000},
{0.1745329252, 20.0000000000},
{0.5235987756, 70.0000000000}
};


static double q_restraint_func6_data[][2] = {
{0.0000000000, 0.0000000000},
{0.1745329252, 20.0000000000},
{0.5235987756, 70.0000000000}
};


static double q_restraint_func7_data[][2] = {
{0.0000000000, 0.0000000000},
{0.1745329252, 20.0000000000},
{0.5235987756, 70.0000000000}
};


static double q_restraint_func8_data[][2] = {
{0.0000000000, 0.0000000000},
{0.1745329252, 20.0000000000},
{0.5235987756, 70.0000000000}
};


static double q_restraint_func9_data[][2] = {
{0.0000000000, 0.0000000000},
{0.1745329252, 20.0000000000},
{0.5235987756, 70.0000000000}
};


static double q_restraint_func10_data[][2] = {
{0.0000000000, 0.0000000000},
{0.1745329252, 20.0000000000},
{0.5235987756, 70.0000000000}
};


static double q_restraint_func11_data[][2] = {
{0.0000000000, 0.0000000000},
{0.1745329252, 20.0000000000},
{0.5235987756, 70.0000000000}
};


static double q_restraint_func12_data[][2] = {
{0.0000000000, 0.0000000000},
{0.1745329252, 20.0000000000},
{0.5235987756, 70.0000000000}
};


static double q_restraint_func13_data[][2] = {
{0.0000000000, 0.0000000000},
{0.1745329252, 20.0000000000},
{0.5235987756, 70.0000000000}
};


static double q_restraint_func14_data[][2] = {
{0.0000000000, 0.0000000000},
{0.1745329252, 20.0000000000},
{0.5235987756, 70.0000000000}
};


static double q_restraint_func15_data[][2] = {
{0.0000000000, 0.0000000000},
{0.1745329252, 20.0000000000},
{0.5235987756, 70.0000000000}
};


static double q_restraint_func16_data[][2] = {
{0.0000000000, 0.0000000000},
{0.1745329252, 20.0000000000},
{0.5235987756, 70.0000000000}
};


static double q_restraint_func17_data[][2] = {
{0.0000000000, 0.0000000000},
{0.1745329252, 20.0000000000},
{0.5235987756, 70.0000000000}
};


static double q_restraint_func18_data[][2] = {
{0.0000000000, 0.0000000000},
{0.1745329252, 20.0000000000},
{0.5235987756, 70.0000000000}
};


static double q_restraint_func19_data[][2] = {
{0.0000000000, 0.0000000000},
{0.1745329252, 20.0000000000},
{0.5235987756, 70.0000000000}
};


static double q_restraint_func20_data[][2] = {
{0.0000000000, 0.0000000000},
{0.1745329252, 20.0000000000},
{0.5235987756, 70.0000000000}
};


static double q_restraint_func21_data[][2] = {
{0.0000000000, 0.0000000000},
{0.1745329252, 20.0000000000},
{0.5235987756, 70.0000000000}
};


static double q_restraint_func22_data[][2] = {
{0.0000000000, 0.0000000000},
{0.1745329252, 20.0000000000},
{0.5235987756, 70.0000000000}
};


static double q_restraint_func23_data[][2] = {
{0.0000000000, 0.0000000000},
{0.1745329252, 20.0000000000},
{0.5235987756, 70.0000000000}
};


static double q_restraint_func24_data[][2] = {
{0.0000000000, 0.0000000000},
{0.1745329252, 20.0000000000},
{0.5235987756, 70.0000000000}
};


static double q_restraint_func25_data[][2] = {
{0.0000000000, 0.0000000000},
{0.1745329252, 20.0000000000},
{0.5235987756, 70.0000000000}
};


static double q_restraint_func26_data[][2] = {
{0.0000000000, 0.0000000000},
{0.1745329252, 20.0000000000},
{0.5235987756, 70.0000000000}
};


static double q_restraint_func27_data[][2] = {
{0.0000000000, 0.0000000000},
{0.1745329252, 20.0000000000},
{0.5235987756, 70.0000000000}
};


static double q_restraint_func28_data[][2] = {
{0.0000000000, 0.0000000000},
{0.1745329252, 20.0000000000},
{0.5235987756, 70.0000000000}
};


static double q_restraint_func29_data[][2] = {
{0.0000000000, 0.0000000000},
{0.1745329252, 20.0000000000},
{0.5235987756, 70.0000000000}
};


static double q_restraint_func30_data[][2] = {
{0.0000000000, 0.0000000000},
{0.1745329252, 20.0000000000},
{0.5235987756, 70.0000000000}
};


static double q_restraint_func31_data[][2] = {
{0.0000000000, 0.0000000000},
{0.1745329252, 20.0000000000},
{0.5235987756, 70.0000000000}
};


static double q_restraint_func32_data[][2] = {
{0.0000000000, 0.0000000000},
{0.1745329252, 20.0000000000},
{0.5235987756, 70.0000000000}
};


static double q_restraint_func33_data[][2] = {
{0.0000000000, 0.0000000000},
{0.1745329252, 20.0000000000},
{0.5235987756, 70.0000000000}
};


static double q_restraint_func34_data[][2] = {
{0.0000000000, 0.0000000000},
{0.1745329252, 20.0000000000},
{0.5235987756, 70.0000000000}
};


static double q_restraint_func35_data[][2] = {
{0.0000000000, 0.0000000000},
{0.1745329252, 20.0000000000},
{0.5235987756, 70.0000000000}
};


static double q_restraint_func36_data[][2] = {
{0.0000000000, 0.0000000000},
{0.1745329252, 20.0000000000},
{0.5235987756, 70.0000000000}
};


static double q_restraint_func37_data[][2] = {
{0.0000000000, 0.0000000000},
{0.1745329252, 20.0000000000},
{0.5235987756, 70.0000000000}
};


static double q_restraint_func38_data[][2] = {
{0.0000000000, 0.0000000000},
{0.1745329252, 20.0000000000},
{0.5235987756, 70.0000000000}
};


static double q_restraint_func39_data[][2] = {
{0.0000000000, 0.0000000000},
{0.1745329252, 20.0000000000},
{0.5235987756, 70.0000000000}
};


static double q_restraint_func40_data[][2] = {
{0.0000000000, 0.0000000000},
{0.1745329252, 20.0000000000},
{0.5235987756, 70.0000000000}
};


static double q_restraint_func41_data[][2] = {
{0.0000000000, 0.0000000000},
{0.1745329252, 20.0000000000},
{0.5235987756, 70.0000000000}
};


static double q_restraint_func42_data[][2] = {
{0.0000000000, 0.0000000000},
{0.1745329252, 20.0000000000},
{0.5235987756, 70.0000000000}
};


static double q_restraint_func43_data[][2] = {
{0.0000000000, 0.0000000000},
{0.1745329252, 20.0000000000},
{0.5235987756, 70.0000000000}
};


static double q_restraint_func44_data[][2] = {
{0.0000000000, 0.0000000000},
{0.1745329252, 20.0000000000},
{0.5235987756, 70.0000000000}
};


static double q_restraint_func45_data[][2] = {
{0.0000000000, 0.0000000000},
{0.1745329252, 20.0000000000},
{0.5235987756, 70.0000000000}
};


static double q_restraint_func46_data[][2] = {
{0.0000000000, 0.0000000000},
{0.1745329252, 20.0000000000},
{0.5235987756, 70.0000000000}
};


static double q_restraint_func47_data[][2] = {
{0.0000000000, 0.0000000000},
{0.1745329252, 20.0000000000},
{0.5235987756, 70.0000000000}
};


static double q_restraint_func48_data[][2] = {
{0.0000000000, 0.0000000000},
{0.1745329252, 20.0000000000},
{0.5235987756, 70.0000000000}
};


static double q_restraint_func49_data[][2] = {
{0.0000000000, 0.0000000000},
{0.1745329252, 20.0000000000},
{0.5235987756, 70.0000000000}
};


static double q_restraint_func50_data[][2] = {
{0.0000000000, 0.0000000000},
{0.1745329252, 20.0000000000},
{0.5235987756, 70.0000000000}
};


static double q_restraint_func51_data[][2] = {
{0.0000000000, 0.0000000000},
{0.1745329252, 20.0000000000},
{0.5235987756, 70.0000000000}
};


static double q_restraint_func52_data[][2] = {
{0.0000000000, 0.0000000000},
{0.1745329252, 20.0000000000},
{0.5235987756, 70.0000000000}
};


static double q_restraint_func53_data[][2] = {
{0.0000000000, 0.0000000000},
{0.1745329252, 20.0000000000},
{0.5235987756, 70.0000000000}
};


static double q_restraint_func54_data[][2] = {
{0.0000000000, 0.0000000000},
{0.1745329252, 20.0000000000},
{0.5235987756, 70.0000000000}
};


static double q_restraint_func55_data[][2] = {
{0.0000000000, 0.0000000000},
{0.1745329252, 20.0000000000},
{0.5235987756, 70.0000000000}
};


static double q_restraint_func56_data[][2] = {
{0.0000000000, 0.0000000000},
{0.1745329252, 20.0000000000},
{0.5235987756, 70.0000000000}
};


static double q_restraint_func57_data[][2] = {
{0.0000000000, 0.0000000000},
{0.1745329252, 20.0000000000},
{0.5235987756, 70.0000000000}
};


static double q_restraint_func58_data[][2] = {
{0.0000000000, 0.0000000000},
{0.1745329252, 20.0000000000},
{0.5235987756, 70.0000000000}
};


static double q_restraint_func59_data[][2] = {
{0.0000000000, 0.0000000000},
{0.1745329252, 20.0000000000},
{0.5235987756, 70.0000000000}
};


static double q_restraint_func60_data[][2] = {
{0.0000000000, 0.0000000000},
{0.1745329252, 20.0000000000},
{0.5235987756, 70.0000000000}
};


static double q_restraint_func61_data[][2] = {
{0.0000000000, 0.0000000000},
{0.1745329252, 20.0000000000},
{0.5235987756, 70.0000000000}
};


static double q_restraint_func62_data[][2] = {
{0.0000000000, 0.0000000000},
{0.1745329252, 20.0000000000},
{0.5235987756, 70.0000000000}
};


static double q_restraint_func63_data[][2] = {
{0.0000000000, 0.0000000000},
{0.1745329252, 20.0000000000},
{0.5235987756, 70.0000000000}
};


static double q_restraint_func64_data[][2] = {
{0.0000000000, 0.0000000000},
{0.1745329252, 20.0000000000},
{0.5235987756, 70.0000000000}
};


static double q_restraint_func65_data[][2] = {
{0.0000000000, 0.0000000000},
{0.1745329252, 20.0000000000},
{0.5235987756, 70.0000000000}
};


static double q_restraint_func66_data[][2] = {
{0.0000000000, 0.0000000000},
{0.1745329252, 20.0000000000},
{0.5235987756, 70.0000000000}
};


static double q_restraint_func67_data[][2] = {
{0.0000000000, 0.0000000000},
{0.1745329252, 20.0000000000},
{0.5235987756, 70.0000000000}
};


static double q_restraint_func68_data[][2] = {
{0.0000000000, 0.0000000000},
{0.1745329252, 20.0000000000},
{0.5235987756, 70.0000000000}
};


static dpSplineFunction q_restraint_func[68];



/**************** GLOBAL VARIABLES (used in only a few files) *****************/
extern dpModelStruct sdm;


/*************** EXTERNED VARIABLES (declared in another file) ****************/



/*************** PROTOTYPES for STATIC FUNCTIONS (for this file only) *********/


/* INIT_QS: this routine initializes the array of structures
 * that hold information about the Qs (gencoords).
 */

void init_qs(void)
{

   int i;

   sdm.q = (dpQStruct*)simm_malloc(sdm.nq*sizeof(dpQStruct));
   mstrcpy(&sdm.q[lower_torso_TX].name,"lower_torso_TX");
   sdm.q[lower_torso_TX].type = dpUnconstrainedQ;
   sdm.q[lower_torso_TX].joint = gnd_pelvis;
   sdm.q[lower_torso_TX].axis = 0;
   // initial_value will now be set using setCoordinateInitialValues
   // sdm.q[lower_torso_TX].initial_value = 0.0000000000;
   sdm.q[lower_torso_TX].initial_velocity = 0.0;
   sdm.q[lower_torso_TX].range_start = -10.0000000000;
   sdm.q[lower_torso_TX].range_end = 10.0000000000;
   sdm.q[lower_torso_TX].restraint_func = NULL;
   sdm.q[lower_torso_TX].min_restraint_func = NULL;
   sdm.q[lower_torso_TX].max_restraint_func = NULL;
   sdm.q[lower_torso_TX].function_active = dpNo;
   sdm.q[lower_torso_TX].constraint_func = NULL;
   sdm.q[lower_torso_TX].constraint_num = -1;
   sdm.q[lower_torso_TX].q_ind = -1;
   sdm.q[lower_torso_TX].output = dpYes;
   sdm.q[lower_torso_TX].pd_stiffness = 0.0;
   sdm.q[lower_torso_TX].torque = 0.0;

   mstrcpy(&sdm.q[lower_torso_TY].name,"lower_torso_TY");
   sdm.q[lower_torso_TY].type = dpUnconstrainedQ;
   sdm.q[lower_torso_TY].joint = gnd_pelvis;
   sdm.q[lower_torso_TY].axis = 1;
   // initial_value will now be set using setCoordinateInitialValues
   // sdm.q[lower_torso_TY].initial_value = 0.0000000000;
   sdm.q[lower_torso_TY].initial_velocity = 0.0;
   sdm.q[lower_torso_TY].range_start = -10.0000000000;
   sdm.q[lower_torso_TY].range_end = 10.0000000000;
   sdm.q[lower_torso_TY].restraint_func = NULL;
   sdm.q[lower_torso_TY].min_restraint_func = NULL;
   sdm.q[lower_torso_TY].max_restraint_func = NULL;
   sdm.q[lower_torso_TY].function_active = dpNo;
   sdm.q[lower_torso_TY].constraint_func = NULL;
   sdm.q[lower_torso_TY].constraint_num = -1;
   sdm.q[lower_torso_TY].q_ind = -1;
   sdm.q[lower_torso_TY].output = dpYes;
   sdm.q[lower_torso_TY].pd_stiffness = 0.0;
   sdm.q[lower_torso_TY].torque = 0.0;

   mstrcpy(&sdm.q[lower_torso_TZ].name,"lower_torso_TZ");
   sdm.q[lower_torso_TZ].type = dpUnconstrainedQ;
   sdm.q[lower_torso_TZ].joint = gnd_pelvis;
   sdm.q[lower_torso_TZ].axis = 2;
   // initial_value will now be set using setCoordinateInitialValues
   // sdm.q[lower_torso_TZ].initial_value = 0.9690000000;
   sdm.q[lower_torso_TZ].initial_velocity = 0.0;
   sdm.q[lower_torso_TZ].range_start = -10.0000000000;
   sdm.q[lower_torso_TZ].range_end = 10.0000000000;
   sdm.q[lower_torso_TZ].restraint_func = NULL;
   sdm.q[lower_torso_TZ].min_restraint_func = NULL;
   sdm.q[lower_torso_TZ].max_restraint_func = NULL;
   sdm.q[lower_torso_TZ].function_active = dpNo;
   sdm.q[lower_torso_TZ].constraint_func = NULL;
   sdm.q[lower_torso_TZ].constraint_num = -1;
   sdm.q[lower_torso_TZ].q_ind = -1;
   sdm.q[lower_torso_TZ].output = dpYes;
   sdm.q[lower_torso_TZ].pd_stiffness = 0.0;
   sdm.q[lower_torso_TZ].torque = 0.0;

   mstrcpy(&sdm.q[lower_torso_RX].name,"lower_torso_RX");
   sdm.q[lower_torso_RX].type = dpUnconstrainedQ;
   sdm.q[lower_torso_RX].joint = gnd_pelvis;
   sdm.q[lower_torso_RX].axis = 3;
   // initial_value will now be set using setCoordinateInitialValues
   // sdm.q[lower_torso_RX].initial_value = 1.5707963268;
   sdm.q[lower_torso_RX].initial_velocity = 0.0;
   sdm.q[lower_torso_RX].range_start = -4.7123889804;
   sdm.q[lower_torso_RX].range_end = 4.7123889804;
   sdm.q[lower_torso_RX].restraint_func = NULL;
   sdm.q[lower_torso_RX].min_restraint_func = NULL;
   sdm.q[lower_torso_RX].max_restraint_func = NULL;
   sdm.q[lower_torso_RX].function_active = dpNo;
   sdm.q[lower_torso_RX].constraint_func = NULL;
   sdm.q[lower_torso_RX].constraint_num = -1;
   sdm.q[lower_torso_RX].q_ind = -1;
   sdm.q[lower_torso_RX].output = dpYes;
   sdm.q[lower_torso_RX].pd_stiffness = 0.0;
   sdm.q[lower_torso_RX].torque = 0.0;

   mstrcpy(&sdm.q[lower_torso_RY].name,"lower_torso_RY");
   sdm.q[lower_torso_RY].type = dpUnconstrainedQ;
   sdm.q[lower_torso_RY].joint = gnd_pelvis;
   sdm.q[lower_torso_RY].axis = 4;
   // initial_value will now be set using setCoordinateInitialValues
   // sdm.q[lower_torso_RY].initial_value = 0.0000000000;
   sdm.q[lower_torso_RY].initial_velocity = 0.0;
   sdm.q[lower_torso_RY].range_start = -4.7123889804;
   sdm.q[lower_torso_RY].range_end = 4.7123889804;
   sdm.q[lower_torso_RY].restraint_func = NULL;
   sdm.q[lower_torso_RY].min_restraint_func = NULL;
   sdm.q[lower_torso_RY].max_restraint_func = NULL;
   sdm.q[lower_torso_RY].function_active = dpNo;
   sdm.q[lower_torso_RY].constraint_func = NULL;
   sdm.q[lower_torso_RY].constraint_num = -1;
   sdm.q[lower_torso_RY].q_ind = -1;
   sdm.q[lower_torso_RY].output = dpYes;
   sdm.q[lower_torso_RY].pd_stiffness = 0.0;
   sdm.q[lower_torso_RY].torque = 0.0;

   mstrcpy(&sdm.q[lower_torso_RZ].name,"lower_torso_RZ");
   sdm.q[lower_torso_RZ].type = dpUnconstrainedQ;
   sdm.q[lower_torso_RZ].joint = gnd_pelvis;
   sdm.q[lower_torso_RZ].axis = 5;
   // initial_value will now be set using setCoordinateInitialValues
   // sdm.q[lower_torso_RZ].initial_value = 0.0000000000;
   sdm.q[lower_torso_RZ].initial_velocity = 0.0;
   sdm.q[lower_torso_RZ].range_start = -4.7123889804;
   sdm.q[lower_torso_RZ].range_end = 4.7123889804;
   sdm.q[lower_torso_RZ].restraint_func = NULL;
   sdm.q[lower_torso_RZ].min_restraint_func = NULL;
   sdm.q[lower_torso_RZ].max_restraint_func = NULL;
   sdm.q[lower_torso_RZ].function_active = dpNo;
   sdm.q[lower_torso_RZ].constraint_func = NULL;
   sdm.q[lower_torso_RZ].constraint_num = -1;
   sdm.q[lower_torso_RZ].q_ind = -1;
   sdm.q[lower_torso_RZ].output = dpYes;
   sdm.q[lower_torso_RZ].pd_stiffness = 0.0;
   sdm.q[lower_torso_RZ].torque = 0.0;

   mstrcpy(&sdm.q[lumbar_pitch].name,"lumbar_pitch");
   sdm.q[lumbar_pitch].type = dpUnconstrainedQ;
   sdm.q[lumbar_pitch].joint = pelvis_torso;
   sdm.q[lumbar_pitch].axis = 0;
   // initial_value will now be set using setCoordinateInitialValues
   // sdm.q[lumbar_pitch].initial_value = 0.0000000000;
   sdm.q[lumbar_pitch].initial_velocity = 0.0;
   sdm.q[lumbar_pitch].range_start = -0.7853981634;
   sdm.q[lumbar_pitch].range_end = 0.7853981634;
   sdm.q[lumbar_pitch].restraint_func = NULL;
   sdm.q[lumbar_pitch].min_restraint_func = &q_restraint_func[4];
   sdm.q[lumbar_pitch].max_restraint_func = &q_restraint_func[5];
   sdm.q[lumbar_pitch].function_active = dpNo;
   sdm.q[lumbar_pitch].constraint_func = NULL;
   sdm.q[lumbar_pitch].constraint_num = -1;
   sdm.q[lumbar_pitch].q_ind = -1;
   sdm.q[lumbar_pitch].output = dpYes;
   sdm.q[lumbar_pitch].pd_stiffness = 0.0;
   sdm.q[lumbar_pitch].torque = 0.0;

   mstrcpy(&sdm.q[lumbar_roll].name,"lumbar_roll");
   sdm.q[lumbar_roll].type = dpUnconstrainedQ;
   sdm.q[lumbar_roll].joint = pelvis_torso;
   sdm.q[lumbar_roll].axis = 1;
   // initial_value will now be set using setCoordinateInitialValues
   // sdm.q[lumbar_roll].initial_value = 0.0000000000;
   sdm.q[lumbar_roll].initial_velocity = 0.0;
   sdm.q[lumbar_roll].range_start = -0.7853981634;
   sdm.q[lumbar_roll].range_end = 0.7853981634;
   sdm.q[lumbar_roll].restraint_func = NULL;
   sdm.q[lumbar_roll].min_restraint_func = &q_restraint_func[0];
   sdm.q[lumbar_roll].max_restraint_func = &q_restraint_func[1];
   sdm.q[lumbar_roll].function_active = dpNo;
   sdm.q[lumbar_roll].constraint_func = NULL;
   sdm.q[lumbar_roll].constraint_num = -1;
   sdm.q[lumbar_roll].q_ind = -1;
   sdm.q[lumbar_roll].output = dpYes;
   sdm.q[lumbar_roll].pd_stiffness = 0.0;
   sdm.q[lumbar_roll].torque = 0.0;

   mstrcpy(&sdm.q[lumbar_yaw].name,"lumbar_yaw");
   sdm.q[lumbar_yaw].type = dpUnconstrainedQ;
   sdm.q[lumbar_yaw].joint = pelvis_torso;
   sdm.q[lumbar_yaw].axis = 2;
   // initial_value will now be set using setCoordinateInitialValues
   // sdm.q[lumbar_yaw].initial_value = 0.0000000000;
   sdm.q[lumbar_yaw].initial_velocity = 0.0;
   sdm.q[lumbar_yaw].range_start = -1.0471975512;
   sdm.q[lumbar_yaw].range_end = 1.0471975512;
   sdm.q[lumbar_yaw].restraint_func = NULL;
   sdm.q[lumbar_yaw].min_restraint_func = &q_restraint_func[2];
   sdm.q[lumbar_yaw].max_restraint_func = &q_restraint_func[3];
   sdm.q[lumbar_yaw].function_active = dpNo;
   sdm.q[lumbar_yaw].constraint_func = NULL;
   sdm.q[lumbar_yaw].constraint_num = -1;
   sdm.q[lumbar_yaw].q_ind = -1;
   sdm.q[lumbar_yaw].output = dpYes;
   sdm.q[lumbar_yaw].pd_stiffness = 0.0;
   sdm.q[lumbar_yaw].torque = 0.0;

   mstrcpy(&sdm.q[neck_pitch].name,"neck_pitch");
   sdm.q[neck_pitch].type = dpUnconstrainedQ;
   sdm.q[neck_pitch].joint = torso_neckhead;
   sdm.q[neck_pitch].axis = 0;
   // initial_value will now be set using setCoordinateInitialValues
   // sdm.q[neck_pitch].initial_value = 0.0000000000;
   sdm.q[neck_pitch].initial_velocity = 0.0;
   sdm.q[neck_pitch].range_start = -0.8726646260;
   sdm.q[neck_pitch].range_end = 1.2566370614;
   sdm.q[neck_pitch].restraint_func = NULL;
   sdm.q[neck_pitch].min_restraint_func = &q_restraint_func[10];
   sdm.q[neck_pitch].max_restraint_func = &q_restraint_func[11];
   sdm.q[neck_pitch].function_active = dpNo;
   sdm.q[neck_pitch].constraint_func = NULL;
   sdm.q[neck_pitch].constraint_num = -1;
   sdm.q[neck_pitch].q_ind = -1;
   sdm.q[neck_pitch].output = dpYes;
   sdm.q[neck_pitch].pd_stiffness = 0.0;
   sdm.q[neck_pitch].torque = 0.0;

   mstrcpy(&sdm.q[neck_roll].name,"neck_roll");
   sdm.q[neck_roll].type = dpUnconstrainedQ;
   sdm.q[neck_roll].joint = torso_neckhead;
   sdm.q[neck_roll].axis = 1;
   // initial_value will now be set using setCoordinateInitialValues
   // sdm.q[neck_roll].initial_value = 0.0000000000;
   sdm.q[neck_roll].initial_velocity = 0.0;
   sdm.q[neck_roll].range_start = -0.6981317008;
   sdm.q[neck_roll].range_end = 0.6981317008;
   sdm.q[neck_roll].restraint_func = NULL;
   sdm.q[neck_roll].min_restraint_func = &q_restraint_func[6];
   sdm.q[neck_roll].max_restraint_func = &q_restraint_func[7];
   sdm.q[neck_roll].function_active = dpNo;
   sdm.q[neck_roll].constraint_func = NULL;
   sdm.q[neck_roll].constraint_num = -1;
   sdm.q[neck_roll].q_ind = -1;
   sdm.q[neck_roll].output = dpYes;
   sdm.q[neck_roll].pd_stiffness = 0.0;
   sdm.q[neck_roll].torque = 0.0;

   mstrcpy(&sdm.q[neck_yaw].name,"neck_yaw");
   sdm.q[neck_yaw].type = dpUnconstrainedQ;
   sdm.q[neck_yaw].joint = torso_neckhead;
   sdm.q[neck_yaw].axis = 2;
   // initial_value will now be set using setCoordinateInitialValues
   // sdm.q[neck_yaw].initial_value = 0.0000000000;
   sdm.q[neck_yaw].initial_velocity = 0.0;
   sdm.q[neck_yaw].range_start = -1.1519173063;
   sdm.q[neck_yaw].range_end = 1.1519173063;
   sdm.q[neck_yaw].restraint_func = NULL;
   sdm.q[neck_yaw].min_restraint_func = &q_restraint_func[8];
   sdm.q[neck_yaw].max_restraint_func = &q_restraint_func[9];
   sdm.q[neck_yaw].function_active = dpNo;
   sdm.q[neck_yaw].constraint_func = NULL;
   sdm.q[neck_yaw].constraint_num = -1;
   sdm.q[neck_yaw].q_ind = -1;
   sdm.q[neck_yaw].output = dpYes;
   sdm.q[neck_yaw].pd_stiffness = 0.0;
   sdm.q[neck_yaw].torque = 0.0;

   mstrcpy(&sdm.q[arm_add_r].name,"arm_add_r");
   sdm.q[arm_add_r].type = dpUnconstrainedQ;
   sdm.q[arm_add_r].joint = acromial_r;
   sdm.q[arm_add_r].axis = 0;
   // initial_value will now be set using setCoordinateInitialValues
   // sdm.q[arm_add_r].initial_value = 0.0000000000;
   sdm.q[arm_add_r].initial_velocity = 0.0;
   sdm.q[arm_add_r].range_start = -2.0943951024;
   sdm.q[arm_add_r].range_end = 1.5707963268;
   sdm.q[arm_add_r].restraint_func = NULL;
   sdm.q[arm_add_r].min_restraint_func = &q_restraint_func[12];
   sdm.q[arm_add_r].max_restraint_func = &q_restraint_func[13];
   sdm.q[arm_add_r].function_active = dpNo;
   sdm.q[arm_add_r].constraint_func = NULL;
   sdm.q[arm_add_r].constraint_num = -1;
   sdm.q[arm_add_r].q_ind = -1;
   sdm.q[arm_add_r].output = dpYes;
   sdm.q[arm_add_r].pd_stiffness = 0.0;
   sdm.q[arm_add_r].torque = 0.0;

   mstrcpy(&sdm.q[arm_flex_r].name,"arm_flex_r");
   sdm.q[arm_flex_r].type = dpUnconstrainedQ;
   sdm.q[arm_flex_r].joint = acromial_r;
   sdm.q[arm_flex_r].axis = 1;
   // initial_value will now be set using setCoordinateInitialValues
   // sdm.q[arm_flex_r].initial_value = 0.0000000000;
   sdm.q[arm_flex_r].initial_velocity = 0.0;
   sdm.q[arm_flex_r].range_start = -0.7853981634;
   sdm.q[arm_flex_r].range_end = 1.5707963268;
   sdm.q[arm_flex_r].restraint_func = NULL;
   sdm.q[arm_flex_r].min_restraint_func = &q_restraint_func[16];
   sdm.q[arm_flex_r].max_restraint_func = &q_restraint_func[17];
   sdm.q[arm_flex_r].function_active = dpNo;
   sdm.q[arm_flex_r].constraint_func = NULL;
   sdm.q[arm_flex_r].constraint_num = -1;
   sdm.q[arm_flex_r].q_ind = -1;
   sdm.q[arm_flex_r].output = dpYes;
   sdm.q[arm_flex_r].pd_stiffness = 0.0;
   sdm.q[arm_flex_r].torque = 0.0;

   mstrcpy(&sdm.q[arm_rot_r].name,"arm_rot_r");
   sdm.q[arm_rot_r].type = dpUnconstrainedQ;
   sdm.q[arm_rot_r].joint = acromial_r;
   sdm.q[arm_rot_r].axis = 2;
   // initial_value will now be set using setCoordinateInitialValues
   // sdm.q[arm_rot_r].initial_value = 0.0000000000;
   sdm.q[arm_rot_r].initial_velocity = 0.0;
   sdm.q[arm_rot_r].range_start = -0.1745329252;
   sdm.q[arm_rot_r].range_end = 0.1745329252;
   sdm.q[arm_rot_r].restraint_func = NULL;
   sdm.q[arm_rot_r].min_restraint_func = &q_restraint_func[14];
   sdm.q[arm_rot_r].max_restraint_func = &q_restraint_func[15];
   sdm.q[arm_rot_r].function_active = dpNo;
   sdm.q[arm_rot_r].constraint_func = NULL;
   sdm.q[arm_rot_r].constraint_num = -1;
   sdm.q[arm_rot_r].q_ind = -1;
   sdm.q[arm_rot_r].output = dpYes;
   sdm.q[arm_rot_r].pd_stiffness = 0.0;
   sdm.q[arm_rot_r].torque = 0.0;

   mstrcpy(&sdm.q[elbow_flex_r].name,"elbow_flex_r");
   sdm.q[elbow_flex_r].type = dpUnconstrainedQ;
   sdm.q[elbow_flex_r].joint = elbow_r;
   sdm.q[elbow_flex_r].axis = 0;
   // initial_value will now be set using setCoordinateInitialValues
   // sdm.q[elbow_flex_r].initial_value = 0.0000000000;
   sdm.q[elbow_flex_r].initial_velocity = 0.0;
   sdm.q[elbow_flex_r].range_start = 0.0000000000;
   sdm.q[elbow_flex_r].range_end = 2.2689280276;
   sdm.q[elbow_flex_r].restraint_func = NULL;
   sdm.q[elbow_flex_r].min_restraint_func = &q_restraint_func[18];
   sdm.q[elbow_flex_r].max_restraint_func = &q_restraint_func[19];
   sdm.q[elbow_flex_r].function_active = dpNo;
   sdm.q[elbow_flex_r].constraint_func = NULL;
   sdm.q[elbow_flex_r].constraint_num = -1;
   sdm.q[elbow_flex_r].q_ind = -1;
   sdm.q[elbow_flex_r].output = dpYes;
   sdm.q[elbow_flex_r].pd_stiffness = 0.0;
   sdm.q[elbow_flex_r].torque = 0.0;

   mstrcpy(&sdm.q[pro_sup_r].name,"pro_sup_r");
   sdm.q[pro_sup_r].type = dpUnconstrainedQ;
   sdm.q[pro_sup_r].joint = radioulnar_r;
   sdm.q[pro_sup_r].axis = 0;
   // initial_value will now be set using setCoordinateInitialValues
   // sdm.q[pro_sup_r].initial_value = 1.0471975512;
   sdm.q[pro_sup_r].initial_velocity = 0.0;
   sdm.q[pro_sup_r].range_start = 0.8726646260;
   sdm.q[pro_sup_r].range_end = 1.2217304764;
   sdm.q[pro_sup_r].restraint_func = NULL;
   sdm.q[pro_sup_r].min_restraint_func = &q_restraint_func[20];
   sdm.q[pro_sup_r].max_restraint_func = &q_restraint_func[21];
   sdm.q[pro_sup_r].function_active = dpNo;
   sdm.q[pro_sup_r].constraint_func = NULL;
   sdm.q[pro_sup_r].constraint_num = -1;
   sdm.q[pro_sup_r].q_ind = -1;
   sdm.q[pro_sup_r].output = dpYes;
   sdm.q[pro_sup_r].pd_stiffness = 0.0;
   sdm.q[pro_sup_r].torque = 0.0;

   mstrcpy(&sdm.q[wrist_flex_r].name,"wrist_flex_r");
   sdm.q[wrist_flex_r].type = dpUnconstrainedQ;
   sdm.q[wrist_flex_r].joint = radius_hand_r;
   sdm.q[wrist_flex_r].axis = 0;
   // initial_value will now be set using setCoordinateInitialValues
   // sdm.q[wrist_flex_r].initial_value = 0.0000000000;
   sdm.q[wrist_flex_r].initial_velocity = 0.0;
   sdm.q[wrist_flex_r].range_start = -1.2217304764;
   sdm.q[wrist_flex_r].range_end = 1.2217304764;
   sdm.q[wrist_flex_r].restraint_func = NULL;
   sdm.q[wrist_flex_r].min_restraint_func = &q_restraint_func[24];
   sdm.q[wrist_flex_r].max_restraint_func = &q_restraint_func[25];
   sdm.q[wrist_flex_r].function_active = dpNo;
   sdm.q[wrist_flex_r].constraint_func = NULL;
   sdm.q[wrist_flex_r].constraint_num = -1;
   sdm.q[wrist_flex_r].q_ind = -1;
   sdm.q[wrist_flex_r].output = dpYes;
   sdm.q[wrist_flex_r].pd_stiffness = 0.0;
   sdm.q[wrist_flex_r].torque = 0.0;

   mstrcpy(&sdm.q[wrist_dev_r].name,"wrist_dev_r");
   sdm.q[wrist_dev_r].type = dpUnconstrainedQ;
   sdm.q[wrist_dev_r].joint = radius_hand_r;
   sdm.q[wrist_dev_r].axis = 1;
   // initial_value will now be set using setCoordinateInitialValues
   // sdm.q[wrist_dev_r].initial_value = 0.0000000000;
   sdm.q[wrist_dev_r].initial_velocity = 0.0;
   sdm.q[wrist_dev_r].range_start = -0.4363323130;
   sdm.q[wrist_dev_r].range_end = 0.6108652382;
   sdm.q[wrist_dev_r].restraint_func = NULL;
   sdm.q[wrist_dev_r].min_restraint_func = &q_restraint_func[22];
   sdm.q[wrist_dev_r].max_restraint_func = &q_restraint_func[23];
   sdm.q[wrist_dev_r].function_active = dpNo;
   sdm.q[wrist_dev_r].constraint_func = NULL;
   sdm.q[wrist_dev_r].constraint_num = -1;
   sdm.q[wrist_dev_r].q_ind = -1;
   sdm.q[wrist_dev_r].output = dpYes;
   sdm.q[wrist_dev_r].pd_stiffness = 0.0;
   sdm.q[wrist_dev_r].torque = 0.0;

   mstrcpy(&sdm.q[arm_add_l].name,"arm_add_l");
   sdm.q[arm_add_l].type = dpUnconstrainedQ;
   sdm.q[arm_add_l].joint = acromial_l;
   sdm.q[arm_add_l].axis = 0;
   // initial_value will now be set using setCoordinateInitialValues
   // sdm.q[arm_add_l].initial_value = 0.0000000000;
   sdm.q[arm_add_l].initial_velocity = 0.0;
   sdm.q[arm_add_l].range_start = -2.0943951024;
   sdm.q[arm_add_l].range_end = 1.5707963268;
   sdm.q[arm_add_l].restraint_func = NULL;
   sdm.q[arm_add_l].min_restraint_func = &q_restraint_func[26];
   sdm.q[arm_add_l].max_restraint_func = &q_restraint_func[27];
   sdm.q[arm_add_l].function_active = dpNo;
   sdm.q[arm_add_l].constraint_func = NULL;
   sdm.q[arm_add_l].constraint_num = -1;
   sdm.q[arm_add_l].q_ind = -1;
   sdm.q[arm_add_l].output = dpYes;
   sdm.q[arm_add_l].pd_stiffness = 0.0;
   sdm.q[arm_add_l].torque = 0.0;

   mstrcpy(&sdm.q[arm_flex_l].name,"arm_flex_l");
   sdm.q[arm_flex_l].type = dpUnconstrainedQ;
   sdm.q[arm_flex_l].joint = acromial_l;
   sdm.q[arm_flex_l].axis = 1;
   // initial_value will now be set using setCoordinateInitialValues
   // sdm.q[arm_flex_l].initial_value = 0.0000000000;
   sdm.q[arm_flex_l].initial_velocity = 0.0;
   sdm.q[arm_flex_l].range_start = -0.7853981634;
   sdm.q[arm_flex_l].range_end = 1.5707963268;
   sdm.q[arm_flex_l].restraint_func = NULL;
   sdm.q[arm_flex_l].min_restraint_func = &q_restraint_func[30];
   sdm.q[arm_flex_l].max_restraint_func = &q_restraint_func[31];
   sdm.q[arm_flex_l].function_active = dpNo;
   sdm.q[arm_flex_l].constraint_func = NULL;
   sdm.q[arm_flex_l].constraint_num = -1;
   sdm.q[arm_flex_l].q_ind = -1;
   sdm.q[arm_flex_l].output = dpYes;
   sdm.q[arm_flex_l].pd_stiffness = 0.0;
   sdm.q[arm_flex_l].torque = 0.0;

   mstrcpy(&sdm.q[arm_rot_l].name,"arm_rot_l");
   sdm.q[arm_rot_l].type = dpUnconstrainedQ;
   sdm.q[arm_rot_l].joint = acromial_l;
   sdm.q[arm_rot_l].axis = 2;
   // initial_value will now be set using setCoordinateInitialValues
   // sdm.q[arm_rot_l].initial_value = 0.0000000000;
   sdm.q[arm_rot_l].initial_velocity = 0.0;
   sdm.q[arm_rot_l].range_start = -0.1745329252;
   sdm.q[arm_rot_l].range_end = 0.1745329252;
   sdm.q[arm_rot_l].restraint_func = NULL;
   sdm.q[arm_rot_l].min_restraint_func = &q_restraint_func[28];
   sdm.q[arm_rot_l].max_restraint_func = &q_restraint_func[29];
   sdm.q[arm_rot_l].function_active = dpNo;
   sdm.q[arm_rot_l].constraint_func = NULL;
   sdm.q[arm_rot_l].constraint_num = -1;
   sdm.q[arm_rot_l].q_ind = -1;
   sdm.q[arm_rot_l].output = dpYes;
   sdm.q[arm_rot_l].pd_stiffness = 0.0;
   sdm.q[arm_rot_l].torque = 0.0;

   mstrcpy(&sdm.q[elbow_flex_l].name,"elbow_flex_l");
   sdm.q[elbow_flex_l].type = dpUnconstrainedQ;
   sdm.q[elbow_flex_l].joint = elbow_l;
   sdm.q[elbow_flex_l].axis = 0;
   // initial_value will now be set using setCoordinateInitialValues
   // sdm.q[elbow_flex_l].initial_value = 0.0000000000;
   sdm.q[elbow_flex_l].initial_velocity = 0.0;
   sdm.q[elbow_flex_l].range_start = 0.0000000000;
   sdm.q[elbow_flex_l].range_end = 2.2689280276;
   sdm.q[elbow_flex_l].restraint_func = NULL;
   sdm.q[elbow_flex_l].min_restraint_func = &q_restraint_func[32];
   sdm.q[elbow_flex_l].max_restraint_func = &q_restraint_func[33];
   sdm.q[elbow_flex_l].function_active = dpNo;
   sdm.q[elbow_flex_l].constraint_func = NULL;
   sdm.q[elbow_flex_l].constraint_num = -1;
   sdm.q[elbow_flex_l].q_ind = -1;
   sdm.q[elbow_flex_l].output = dpYes;
   sdm.q[elbow_flex_l].pd_stiffness = 0.0;
   sdm.q[elbow_flex_l].torque = 0.0;

   mstrcpy(&sdm.q[pro_sup_l].name,"pro_sup_l");
   sdm.q[pro_sup_l].type = dpUnconstrainedQ;
   sdm.q[pro_sup_l].joint = radioulnar_l;
   sdm.q[pro_sup_l].axis = 0;
   // initial_value will now be set using setCoordinateInitialValues
   // sdm.q[pro_sup_l].initial_value = 1.0471975512;
   sdm.q[pro_sup_l].initial_velocity = 0.0;
   sdm.q[pro_sup_l].range_start = 0.8726646260;
   sdm.q[pro_sup_l].range_end = 1.2217304764;
   sdm.q[pro_sup_l].restraint_func = NULL;
   sdm.q[pro_sup_l].min_restraint_func = &q_restraint_func[34];
   sdm.q[pro_sup_l].max_restraint_func = &q_restraint_func[35];
   sdm.q[pro_sup_l].function_active = dpNo;
   sdm.q[pro_sup_l].constraint_func = NULL;
   sdm.q[pro_sup_l].constraint_num = -1;
   sdm.q[pro_sup_l].q_ind = -1;
   sdm.q[pro_sup_l].output = dpYes;
   sdm.q[pro_sup_l].pd_stiffness = 0.0;
   sdm.q[pro_sup_l].torque = 0.0;

   mstrcpy(&sdm.q[wrist_flex_l].name,"wrist_flex_l");
   sdm.q[wrist_flex_l].type = dpUnconstrainedQ;
   sdm.q[wrist_flex_l].joint = radius_hand_l;
   sdm.q[wrist_flex_l].axis = 0;
   // initial_value will now be set using setCoordinateInitialValues
   // sdm.q[wrist_flex_l].initial_value = 0.0000000000;
   sdm.q[wrist_flex_l].initial_velocity = 0.0;
   sdm.q[wrist_flex_l].range_start = -1.2217304764;
   sdm.q[wrist_flex_l].range_end = 1.2217304764;
   sdm.q[wrist_flex_l].restraint_func = NULL;
   sdm.q[wrist_flex_l].min_restraint_func = &q_restraint_func[38];
   sdm.q[wrist_flex_l].max_restraint_func = &q_restraint_func[39];
   sdm.q[wrist_flex_l].function_active = dpNo;
   sdm.q[wrist_flex_l].constraint_func = NULL;
   sdm.q[wrist_flex_l].constraint_num = -1;
   sdm.q[wrist_flex_l].q_ind = -1;
   sdm.q[wrist_flex_l].output = dpYes;
   sdm.q[wrist_flex_l].pd_stiffness = 0.0;
   sdm.q[wrist_flex_l].torque = 0.0;

   mstrcpy(&sdm.q[wrist_dev_l].name,"wrist_dev_l");
   sdm.q[wrist_dev_l].type = dpUnconstrainedQ;
   sdm.q[wrist_dev_l].joint = radius_hand_l;
   sdm.q[wrist_dev_l].axis = 1;
   // initial_value will now be set using setCoordinateInitialValues
   // sdm.q[wrist_dev_l].initial_value = 0.0000000000;
   sdm.q[wrist_dev_l].initial_velocity = 0.0;
   sdm.q[wrist_dev_l].range_start = -0.4363323130;
   sdm.q[wrist_dev_l].range_end = 0.6108652382;
   sdm.q[wrist_dev_l].restraint_func = NULL;
   sdm.q[wrist_dev_l].min_restraint_func = &q_restraint_func[36];
   sdm.q[wrist_dev_l].max_restraint_func = &q_restraint_func[37];
   sdm.q[wrist_dev_l].function_active = dpNo;
   sdm.q[wrist_dev_l].constraint_func = NULL;
   sdm.q[wrist_dev_l].constraint_num = -1;
   sdm.q[wrist_dev_l].q_ind = -1;
   sdm.q[wrist_dev_l].output = dpYes;
   sdm.q[wrist_dev_l].pd_stiffness = 0.0;
   sdm.q[wrist_dev_l].torque = 0.0;

   mstrcpy(&sdm.q[hip_flex_r].name,"hip_flex_r");
   sdm.q[hip_flex_r].type = dpUnconstrainedQ;
   sdm.q[hip_flex_r].joint = hip_r;
   sdm.q[hip_flex_r].axis = 0;
   // initial_value will now be set using setCoordinateInitialValues
   // sdm.q[hip_flex_r].initial_value = 0.0000000000;
   sdm.q[hip_flex_r].initial_velocity = 0.0;
   sdm.q[hip_flex_r].range_start = -0.1919862177;
   sdm.q[hip_flex_r].range_end = 1.6580627894;
   sdm.q[hip_flex_r].restraint_func = NULL;
   sdm.q[hip_flex_r].min_restraint_func = &q_restraint_func[44];
   sdm.q[hip_flex_r].max_restraint_func = &q_restraint_func[45];
   sdm.q[hip_flex_r].function_active = dpNo;
   sdm.q[hip_flex_r].constraint_func = NULL;
   sdm.q[hip_flex_r].constraint_num = -1;
   sdm.q[hip_flex_r].q_ind = -1;
   sdm.q[hip_flex_r].output = dpYes;
   sdm.q[hip_flex_r].pd_stiffness = 0.0;
   sdm.q[hip_flex_r].torque = 0.0;

   mstrcpy(&sdm.q[hip_add_r].name,"hip_add_r");
   sdm.q[hip_add_r].type = dpUnconstrainedQ;
   sdm.q[hip_add_r].joint = hip_r;
   sdm.q[hip_add_r].axis = 1;
   // initial_value will now be set using setCoordinateInitialValues
   // sdm.q[hip_add_r].initial_value = 0.0000000000;
   sdm.q[hip_add_r].initial_velocity = 0.0;
   sdm.q[hip_add_r].range_start = -0.8726646260;
   sdm.q[hip_add_r].range_end = 0.3490658504;
   sdm.q[hip_add_r].restraint_func = NULL;
   sdm.q[hip_add_r].min_restraint_func = &q_restraint_func[40];
   sdm.q[hip_add_r].max_restraint_func = &q_restraint_func[41];
   sdm.q[hip_add_r].function_active = dpNo;
   sdm.q[hip_add_r].constraint_func = NULL;
   sdm.q[hip_add_r].constraint_num = -1;
   sdm.q[hip_add_r].q_ind = -1;
   sdm.q[hip_add_r].output = dpYes;
   sdm.q[hip_add_r].pd_stiffness = 0.0;
   sdm.q[hip_add_r].torque = 0.0;

   mstrcpy(&sdm.q[hip_rot_r].name,"hip_rot_r");
   sdm.q[hip_rot_r].type = dpUnconstrainedQ;
   sdm.q[hip_rot_r].joint = hip_r;
   sdm.q[hip_rot_r].axis = 2;
   // initial_value will now be set using setCoordinateInitialValues
   // sdm.q[hip_rot_r].initial_value = 0.0000000000;
   sdm.q[hip_rot_r].initial_velocity = 0.0;
   sdm.q[hip_rot_r].range_start = -0.6981317008;
   sdm.q[hip_rot_r].range_end = 0.6981317008;
   sdm.q[hip_rot_r].restraint_func = NULL;
   sdm.q[hip_rot_r].min_restraint_func = &q_restraint_func[42];
   sdm.q[hip_rot_r].max_restraint_func = &q_restraint_func[43];
   sdm.q[hip_rot_r].function_active = dpNo;
   sdm.q[hip_rot_r].constraint_func = NULL;
   sdm.q[hip_rot_r].constraint_num = -1;
   sdm.q[hip_rot_r].q_ind = -1;
   sdm.q[hip_rot_r].output = dpYes;
   sdm.q[hip_rot_r].pd_stiffness = 0.0;
   sdm.q[hip_rot_r].torque = 0.0;

   mstrcpy(&sdm.q[knee_flexion_r_tx].name,"knee_flexion_r_tx");
   sdm.q[knee_flexion_r_tx].type = dpConstrainedQ;
   sdm.q[knee_flexion_r_tx].joint = knee_flexion_r;
   sdm.q[knee_flexion_r_tx].axis = 0;
   // initial_value will now be set using setCoordinateInitialValues
   // sdm.q[knee_flexion_r_tx].initial_value = -0.0052500000;
   sdm.q[knee_flexion_r_tx].initial_velocity = 0.0;
   sdm.q[knee_flexion_r_tx].range_start = -99999.9;
   sdm.q[knee_flexion_r_tx].range_end = 99999.9;
   sdm.q[knee_flexion_r_tx].restraint_func = NULL;
   sdm.q[knee_flexion_r_tx].min_restraint_func = NULL;
   sdm.q[knee_flexion_r_tx].max_restraint_func = NULL;
   sdm.q[knee_flexion_r_tx].function_active = dpNo;
   sdm.q[knee_flexion_r_tx].constraint_func = &knee_flexion_r_tx_func;
   sdm.q[knee_flexion_r_tx].constraint_num = knee_flexion_r_tx_con;
   sdm.q[knee_flexion_r_tx].q_ind = knee_flex_r;
   sdm.q[knee_flexion_r_tx].output = dpNo;
   sdm.q[knee_flexion_r_tx].pd_stiffness = 0.0;
   sdm.q[knee_flexion_r_tx].torque = 0.0;

   mstrcpy(&sdm.q[knee_flexion_r_ty].name,"knee_flexion_r_ty");
   sdm.q[knee_flexion_r_ty].type = dpConstrainedQ;
   sdm.q[knee_flexion_r_ty].joint = knee_flexion_r;
   sdm.q[knee_flexion_r_ty].axis = 1;
   // initial_value will now be set using setCoordinateInitialValues
   // sdm.q[knee_flexion_r_ty].initial_value = -0.3960000000;
   sdm.q[knee_flexion_r_ty].initial_velocity = 0.0;
   sdm.q[knee_flexion_r_ty].range_start = -99999.9;
   sdm.q[knee_flexion_r_ty].range_end = 99999.9;
   sdm.q[knee_flexion_r_ty].restraint_func = NULL;
   sdm.q[knee_flexion_r_ty].min_restraint_func = NULL;
   sdm.q[knee_flexion_r_ty].max_restraint_func = NULL;
   sdm.q[knee_flexion_r_ty].function_active = dpNo;
   sdm.q[knee_flexion_r_ty].constraint_func = &knee_flexion_r_ty_func;
   sdm.q[knee_flexion_r_ty].constraint_num = knee_flexion_r_ty_con;
   sdm.q[knee_flexion_r_ty].q_ind = knee_flex_r;
   sdm.q[knee_flexion_r_ty].output = dpNo;
   sdm.q[knee_flexion_r_ty].pd_stiffness = 0.0;
   sdm.q[knee_flexion_r_ty].torque = 0.0;

   mstrcpy(&sdm.q[knee_flex_r].name,"knee_flex_r");
   sdm.q[knee_flex_r].type = dpUnconstrainedQ;
   sdm.q[knee_flex_r].joint = knee_flexion_r;
   sdm.q[knee_flex_r].axis = 2;
   // initial_value will now be set using setCoordinateInitialValues
   // sdm.q[knee_flex_r].initial_value = 0.0000000000;
   sdm.q[knee_flex_r].initial_velocity = 0.0;
   sdm.q[knee_flex_r].range_start = -0.1745329252;
   sdm.q[knee_flex_r].range_end = 2.1118483949;
   sdm.q[knee_flex_r].restraint_func = NULL;
   sdm.q[knee_flex_r].min_restraint_func = &q_restraint_func[46];
   sdm.q[knee_flex_r].max_restraint_func = &q_restraint_func[47];
   sdm.q[knee_flex_r].function_active = dpNo;
   sdm.q[knee_flex_r].constraint_func = NULL;
   sdm.q[knee_flex_r].constraint_num = -1;
   sdm.q[knee_flex_r].q_ind = -1;
   sdm.q[knee_flex_r].output = dpYes;
   sdm.q[knee_flex_r].pd_stiffness = 0.0;
   sdm.q[knee_flex_r].torque = 0.0;

   mstrcpy(&sdm.q[tib_pat_r_tx].name,"tib_pat_r_tx");
   sdm.q[tib_pat_r_tx].type = dpConstrainedQ;
   sdm.q[tib_pat_r_tx].joint = tib_pat_r;
   sdm.q[tib_pat_r_tx].axis = 0;
   // initial_value will now be set using setCoordinateInitialValues
   // sdm.q[tib_pat_r_tx].initial_value = 0.0496000000;
   sdm.q[tib_pat_r_tx].initial_velocity = 0.0;
   sdm.q[tib_pat_r_tx].range_start = -99999.9;
   sdm.q[tib_pat_r_tx].range_end = 99999.9;
   sdm.q[tib_pat_r_tx].restraint_func = NULL;
   sdm.q[tib_pat_r_tx].min_restraint_func = NULL;
   sdm.q[tib_pat_r_tx].max_restraint_func = NULL;
   sdm.q[tib_pat_r_tx].function_active = dpNo;
   sdm.q[tib_pat_r_tx].constraint_func = &tib_pat_r_tx_func;
   sdm.q[tib_pat_r_tx].constraint_num = tib_pat_r_tx_con;
   sdm.q[tib_pat_r_tx].q_ind = knee_flex_r;
   sdm.q[tib_pat_r_tx].output = dpNo;
   sdm.q[tib_pat_r_tx].pd_stiffness = 0.0;
   sdm.q[tib_pat_r_tx].torque = 0.0;

   mstrcpy(&sdm.q[tib_pat_r_ty].name,"tib_pat_r_ty");
   sdm.q[tib_pat_r_ty].type = dpConstrainedQ;
   sdm.q[tib_pat_r_ty].joint = tib_pat_r;
   sdm.q[tib_pat_r_ty].axis = 1;
   // initial_value will now be set using setCoordinateInitialValues
   // sdm.q[tib_pat_r_ty].initial_value = -0.0227120772;
   sdm.q[tib_pat_r_ty].initial_velocity = 0.0;
   sdm.q[tib_pat_r_ty].range_start = -99999.9;
   sdm.q[tib_pat_r_ty].range_end = 99999.9;
   sdm.q[tib_pat_r_ty].restraint_func = NULL;
   sdm.q[tib_pat_r_ty].min_restraint_func = NULL;
   sdm.q[tib_pat_r_ty].max_restraint_func = NULL;
   sdm.q[tib_pat_r_ty].function_active = dpNo;
   sdm.q[tib_pat_r_ty].constraint_func = &tib_pat_r_ty_func;
   sdm.q[tib_pat_r_ty].constraint_num = tib_pat_r_ty_con;
   sdm.q[tib_pat_r_ty].q_ind = knee_flex_r;
   sdm.q[tib_pat_r_ty].output = dpNo;
   sdm.q[tib_pat_r_ty].pd_stiffness = 0.0;
   sdm.q[tib_pat_r_ty].torque = 0.0;

   mstrcpy(&sdm.q[tib_pat_r_r1].name,"tib_pat_r_r1");
   sdm.q[tib_pat_r_r1].type = dpConstrainedQ;
   sdm.q[tib_pat_r_r1].joint = tib_pat_r;
   sdm.q[tib_pat_r_r1].axis = 2;
   // initial_value will now be set using setCoordinateInitialValues
   // sdm.q[tib_pat_r_r1].initial_value = -0.0410736345;
   sdm.q[tib_pat_r_r1].initial_velocity = 0.0;
   sdm.q[tib_pat_r_r1].range_start = -99999.9;
   sdm.q[tib_pat_r_r1].range_end = 99999.9;
   sdm.q[tib_pat_r_r1].restraint_func = NULL;
   sdm.q[tib_pat_r_r1].min_restraint_func = NULL;
   sdm.q[tib_pat_r_r1].max_restraint_func = NULL;
   sdm.q[tib_pat_r_r1].function_active = dpNo;
   sdm.q[tib_pat_r_r1].constraint_func = &tib_pat_r_r1_func;
   sdm.q[tib_pat_r_r1].constraint_num = tib_pat_r_r1_con;
   sdm.q[tib_pat_r_r1].q_ind = knee_flex_r;
   sdm.q[tib_pat_r_r1].output = dpNo;
   sdm.q[tib_pat_r_r1].pd_stiffness = 0.0;
   sdm.q[tib_pat_r_r1].torque = 0.0;

   mstrcpy(&sdm.q[ankle_flex_r].name,"ankle_flex_r");
   sdm.q[ankle_flex_r].type = dpUnconstrainedQ;
   sdm.q[ankle_flex_r].joint = ankle_r;
   sdm.q[ankle_flex_r].axis = 0;
   // initial_value will now be set using setCoordinateInitialValues
   // sdm.q[ankle_flex_r].initial_value = 0.0000000000;
   sdm.q[ankle_flex_r].initial_velocity = 0.0;
   sdm.q[ankle_flex_r].range_start = -0.5235987756;
   sdm.q[ankle_flex_r].range_end = 0.5235987756;
   sdm.q[ankle_flex_r].restraint_func = NULL;
   sdm.q[ankle_flex_r].min_restraint_func = &q_restraint_func[48];
   sdm.q[ankle_flex_r].max_restraint_func = &q_restraint_func[49];
   sdm.q[ankle_flex_r].function_active = dpNo;
   sdm.q[ankle_flex_r].constraint_func = NULL;
   sdm.q[ankle_flex_r].constraint_num = -1;
   sdm.q[ankle_flex_r].q_ind = -1;
   sdm.q[ankle_flex_r].output = dpYes;
   sdm.q[ankle_flex_r].pd_stiffness = 0.0;
   sdm.q[ankle_flex_r].torque = 0.0;

   mstrcpy(&sdm.q[subt_angle_r].name,"subt_angle_r");
   sdm.q[subt_angle_r].type = dpUnconstrainedQ;
   sdm.q[subt_angle_r].joint = subtalar_r;
   sdm.q[subt_angle_r].axis = 0;
   // initial_value will now be set using setCoordinateInitialValues
   // sdm.q[subt_angle_r].initial_value = 0.0000000000;
   sdm.q[subt_angle_r].initial_velocity = 0.0;
   sdm.q[subt_angle_r].range_start = -0.3490658504;
   sdm.q[subt_angle_r].range_end = 0.3490658504;
   sdm.q[subt_angle_r].restraint_func = NULL;
   sdm.q[subt_angle_r].min_restraint_func = &q_restraint_func[50];
   sdm.q[subt_angle_r].max_restraint_func = &q_restraint_func[51];
   sdm.q[subt_angle_r].function_active = dpNo;
   sdm.q[subt_angle_r].constraint_func = NULL;
   sdm.q[subt_angle_r].constraint_num = -1;
   sdm.q[subt_angle_r].q_ind = -1;
   sdm.q[subt_angle_r].output = dpYes;
   sdm.q[subt_angle_r].pd_stiffness = 0.0;
   sdm.q[subt_angle_r].torque = 0.0;

   mstrcpy(&sdm.q[toe_angle_r].name,"toe_angle_r");
   sdm.q[toe_angle_r].type = dpUnconstrainedQ;
   sdm.q[toe_angle_r].joint = toes_r;
   sdm.q[toe_angle_r].axis = 0;
   // initial_value will now be set using setCoordinateInitialValues
   // sdm.q[toe_angle_r].initial_value = 0.0000000000;
   sdm.q[toe_angle_r].initial_velocity = 0.0;
   sdm.q[toe_angle_r].range_start = -0.5235987756;
   sdm.q[toe_angle_r].range_end = 0.5235987756;
   sdm.q[toe_angle_r].restraint_func = NULL;
   sdm.q[toe_angle_r].min_restraint_func = &q_restraint_func[52];
   sdm.q[toe_angle_r].max_restraint_func = &q_restraint_func[53];
   sdm.q[toe_angle_r].function_active = dpNo;
   sdm.q[toe_angle_r].constraint_func = NULL;
   sdm.q[toe_angle_r].constraint_num = -1;
   sdm.q[toe_angle_r].q_ind = -1;
   sdm.q[toe_angle_r].output = dpYes;
   sdm.q[toe_angle_r].pd_stiffness = 0.0;
   sdm.q[toe_angle_r].torque = 0.0;

   mstrcpy(&sdm.q[hip_flex_l].name,"hip_flex_l");
   sdm.q[hip_flex_l].type = dpUnconstrainedQ;
   sdm.q[hip_flex_l].joint = hip_l;
   sdm.q[hip_flex_l].axis = 0;
   // initial_value will now be set using setCoordinateInitialValues
   // sdm.q[hip_flex_l].initial_value = 0.0000000000;
   sdm.q[hip_flex_l].initial_velocity = 0.0;
   sdm.q[hip_flex_l].range_start = -0.1919862177;
   sdm.q[hip_flex_l].range_end = 1.6580627894;
   sdm.q[hip_flex_l].restraint_func = NULL;
   sdm.q[hip_flex_l].min_restraint_func = &q_restraint_func[58];
   sdm.q[hip_flex_l].max_restraint_func = &q_restraint_func[59];
   sdm.q[hip_flex_l].function_active = dpNo;
   sdm.q[hip_flex_l].constraint_func = NULL;
   sdm.q[hip_flex_l].constraint_num = -1;
   sdm.q[hip_flex_l].q_ind = -1;
   sdm.q[hip_flex_l].output = dpYes;
   sdm.q[hip_flex_l].pd_stiffness = 0.0;
   sdm.q[hip_flex_l].torque = 0.0;

   mstrcpy(&sdm.q[hip_add_l].name,"hip_add_l");
   sdm.q[hip_add_l].type = dpUnconstrainedQ;
   sdm.q[hip_add_l].joint = hip_l;
   sdm.q[hip_add_l].axis = 1;
   // initial_value will now be set using setCoordinateInitialValues
   // sdm.q[hip_add_l].initial_value = 0.0000000000;
   sdm.q[hip_add_l].initial_velocity = 0.0;
   sdm.q[hip_add_l].range_start = -0.8726646260;
   sdm.q[hip_add_l].range_end = 0.3490658504;
   sdm.q[hip_add_l].restraint_func = NULL;
   sdm.q[hip_add_l].min_restraint_func = &q_restraint_func[54];
   sdm.q[hip_add_l].max_restraint_func = &q_restraint_func[55];
   sdm.q[hip_add_l].function_active = dpNo;
   sdm.q[hip_add_l].constraint_func = NULL;
   sdm.q[hip_add_l].constraint_num = -1;
   sdm.q[hip_add_l].q_ind = -1;
   sdm.q[hip_add_l].output = dpYes;
   sdm.q[hip_add_l].pd_stiffness = 0.0;
   sdm.q[hip_add_l].torque = 0.0;

   mstrcpy(&sdm.q[hip_rot_l].name,"hip_rot_l");
   sdm.q[hip_rot_l].type = dpUnconstrainedQ;
   sdm.q[hip_rot_l].joint = hip_l;
   sdm.q[hip_rot_l].axis = 2;
   // initial_value will now be set using setCoordinateInitialValues
   // sdm.q[hip_rot_l].initial_value = 0.0000000000;
   sdm.q[hip_rot_l].initial_velocity = 0.0;
   sdm.q[hip_rot_l].range_start = -0.6981317008;
   sdm.q[hip_rot_l].range_end = 0.6981317008;
   sdm.q[hip_rot_l].restraint_func = NULL;
   sdm.q[hip_rot_l].min_restraint_func = &q_restraint_func[56];
   sdm.q[hip_rot_l].max_restraint_func = &q_restraint_func[57];
   sdm.q[hip_rot_l].function_active = dpNo;
   sdm.q[hip_rot_l].constraint_func = NULL;
   sdm.q[hip_rot_l].constraint_num = -1;
   sdm.q[hip_rot_l].q_ind = -1;
   sdm.q[hip_rot_l].output = dpYes;
   sdm.q[hip_rot_l].pd_stiffness = 0.0;
   sdm.q[hip_rot_l].torque = 0.0;

   mstrcpy(&sdm.q[knee_flexion_l_tx].name,"knee_flexion_l_tx");
   sdm.q[knee_flexion_l_tx].type = dpConstrainedQ;
   sdm.q[knee_flexion_l_tx].joint = knee_flexion_l;
   sdm.q[knee_flexion_l_tx].axis = 0;
   // initial_value will now be set using setCoordinateInitialValues
   // sdm.q[knee_flexion_l_tx].initial_value = -0.0052500000;
   sdm.q[knee_flexion_l_tx].initial_velocity = 0.0;
   sdm.q[knee_flexion_l_tx].range_start = -99999.9;
   sdm.q[knee_flexion_l_tx].range_end = 99999.9;
   sdm.q[knee_flexion_l_tx].restraint_func = NULL;
   sdm.q[knee_flexion_l_tx].min_restraint_func = NULL;
   sdm.q[knee_flexion_l_tx].max_restraint_func = NULL;
   sdm.q[knee_flexion_l_tx].function_active = dpNo;
   sdm.q[knee_flexion_l_tx].constraint_func = &knee_flexion_l_tx_func;
   sdm.q[knee_flexion_l_tx].constraint_num = knee_flexion_l_tx_con;
   sdm.q[knee_flexion_l_tx].q_ind = knee_flex_l;
   sdm.q[knee_flexion_l_tx].output = dpNo;
   sdm.q[knee_flexion_l_tx].pd_stiffness = 0.0;
   sdm.q[knee_flexion_l_tx].torque = 0.0;

   mstrcpy(&sdm.q[knee_flexion_l_ty].name,"knee_flexion_l_ty");
   sdm.q[knee_flexion_l_ty].type = dpConstrainedQ;
   sdm.q[knee_flexion_l_ty].joint = knee_flexion_l;
   sdm.q[knee_flexion_l_ty].axis = 1;
   // initial_value will now be set using setCoordinateInitialValues
   // sdm.q[knee_flexion_l_ty].initial_value = -0.3960000000;
   sdm.q[knee_flexion_l_ty].initial_velocity = 0.0;
   sdm.q[knee_flexion_l_ty].range_start = -99999.9;
   sdm.q[knee_flexion_l_ty].range_end = 99999.9;
   sdm.q[knee_flexion_l_ty].restraint_func = NULL;
   sdm.q[knee_flexion_l_ty].min_restraint_func = NULL;
   sdm.q[knee_flexion_l_ty].max_restraint_func = NULL;
   sdm.q[knee_flexion_l_ty].function_active = dpNo;
   sdm.q[knee_flexion_l_ty].constraint_func = &knee_flexion_l_ty_func;
   sdm.q[knee_flexion_l_ty].constraint_num = knee_flexion_l_ty_con;
   sdm.q[knee_flexion_l_ty].q_ind = knee_flex_l;
   sdm.q[knee_flexion_l_ty].output = dpNo;
   sdm.q[knee_flexion_l_ty].pd_stiffness = 0.0;
   sdm.q[knee_flexion_l_ty].torque = 0.0;

   mstrcpy(&sdm.q[knee_flex_l].name,"knee_flex_l");
   sdm.q[knee_flex_l].type = dpUnconstrainedQ;
   sdm.q[knee_flex_l].joint = knee_flexion_l;
   sdm.q[knee_flex_l].axis = 2;
   // initial_value will now be set using setCoordinateInitialValues
   // sdm.q[knee_flex_l].initial_value = 0.0000000000;
   sdm.q[knee_flex_l].initial_velocity = 0.0;
   sdm.q[knee_flex_l].range_start = -0.1745329252;
   sdm.q[knee_flex_l].range_end = 2.1118483949;
   sdm.q[knee_flex_l].restraint_func = NULL;
   sdm.q[knee_flex_l].min_restraint_func = &q_restraint_func[60];
   sdm.q[knee_flex_l].max_restraint_func = &q_restraint_func[61];
   sdm.q[knee_flex_l].function_active = dpNo;
   sdm.q[knee_flex_l].constraint_func = NULL;
   sdm.q[knee_flex_l].constraint_num = -1;
   sdm.q[knee_flex_l].q_ind = -1;
   sdm.q[knee_flex_l].output = dpYes;
   sdm.q[knee_flex_l].pd_stiffness = 0.0;
   sdm.q[knee_flex_l].torque = 0.0;

   mstrcpy(&sdm.q[tib_pat_l_tx].name,"tib_pat_l_tx");
   sdm.q[tib_pat_l_tx].type = dpConstrainedQ;
   sdm.q[tib_pat_l_tx].joint = tib_pat_l;
   sdm.q[tib_pat_l_tx].axis = 0;
   // initial_value will now be set using setCoordinateInitialValues
   // sdm.q[tib_pat_l_tx].initial_value = 0.0496000000;
   sdm.q[tib_pat_l_tx].initial_velocity = 0.0;
   sdm.q[tib_pat_l_tx].range_start = -99999.9;
   sdm.q[tib_pat_l_tx].range_end = 99999.9;
   sdm.q[tib_pat_l_tx].restraint_func = NULL;
   sdm.q[tib_pat_l_tx].min_restraint_func = NULL;
   sdm.q[tib_pat_l_tx].max_restraint_func = NULL;
   sdm.q[tib_pat_l_tx].function_active = dpNo;
   sdm.q[tib_pat_l_tx].constraint_func = &tib_pat_l_tx_func;
   sdm.q[tib_pat_l_tx].constraint_num = tib_pat_l_tx_con;
   sdm.q[tib_pat_l_tx].q_ind = knee_flex_l;
   sdm.q[tib_pat_l_tx].output = dpNo;
   sdm.q[tib_pat_l_tx].pd_stiffness = 0.0;
   sdm.q[tib_pat_l_tx].torque = 0.0;

   mstrcpy(&sdm.q[tib_pat_l_ty].name,"tib_pat_l_ty");
   sdm.q[tib_pat_l_ty].type = dpConstrainedQ;
   sdm.q[tib_pat_l_ty].joint = tib_pat_l;
   sdm.q[tib_pat_l_ty].axis = 1;
   // initial_value will now be set using setCoordinateInitialValues
   // sdm.q[tib_pat_l_ty].initial_value = -0.0227120772;
   sdm.q[tib_pat_l_ty].initial_velocity = 0.0;
   sdm.q[tib_pat_l_ty].range_start = -99999.9;
   sdm.q[tib_pat_l_ty].range_end = 99999.9;
   sdm.q[tib_pat_l_ty].restraint_func = NULL;
   sdm.q[tib_pat_l_ty].min_restraint_func = NULL;
   sdm.q[tib_pat_l_ty].max_restraint_func = NULL;
   sdm.q[tib_pat_l_ty].function_active = dpNo;
   sdm.q[tib_pat_l_ty].constraint_func = &tib_pat_l_ty_func;
   sdm.q[tib_pat_l_ty].constraint_num = tib_pat_l_ty_con;
   sdm.q[tib_pat_l_ty].q_ind = knee_flex_l;
   sdm.q[tib_pat_l_ty].output = dpNo;
   sdm.q[tib_pat_l_ty].pd_stiffness = 0.0;
   sdm.q[tib_pat_l_ty].torque = 0.0;

   mstrcpy(&sdm.q[tib_pat_l_r1].name,"tib_pat_l_r1");
   sdm.q[tib_pat_l_r1].type = dpConstrainedQ;
   sdm.q[tib_pat_l_r1].joint = tib_pat_l;
   sdm.q[tib_pat_l_r1].axis = 2;
   // initial_value will now be set using setCoordinateInitialValues
   // sdm.q[tib_pat_l_r1].initial_value = -0.0410736345;
   sdm.q[tib_pat_l_r1].initial_velocity = 0.0;
   sdm.q[tib_pat_l_r1].range_start = -99999.9;
   sdm.q[tib_pat_l_r1].range_end = 99999.9;
   sdm.q[tib_pat_l_r1].restraint_func = NULL;
   sdm.q[tib_pat_l_r1].min_restraint_func = NULL;
   sdm.q[tib_pat_l_r1].max_restraint_func = NULL;
   sdm.q[tib_pat_l_r1].function_active = dpNo;
   sdm.q[tib_pat_l_r1].constraint_func = &tib_pat_l_r1_func;
   sdm.q[tib_pat_l_r1].constraint_num = tib_pat_l_r1_con;
   sdm.q[tib_pat_l_r1].q_ind = knee_flex_l;
   sdm.q[tib_pat_l_r1].output = dpNo;
   sdm.q[tib_pat_l_r1].pd_stiffness = 0.0;
   sdm.q[tib_pat_l_r1].torque = 0.0;

   mstrcpy(&sdm.q[ankle_flex_l].name,"ankle_flex_l");
   sdm.q[ankle_flex_l].type = dpUnconstrainedQ;
   sdm.q[ankle_flex_l].joint = ankle_l;
   sdm.q[ankle_flex_l].axis = 0;
   // initial_value will now be set using setCoordinateInitialValues
   // sdm.q[ankle_flex_l].initial_value = 0.0000000000;
   sdm.q[ankle_flex_l].initial_velocity = 0.0;
   sdm.q[ankle_flex_l].range_start = -0.5235987756;
   sdm.q[ankle_flex_l].range_end = 0.5235987756;
   sdm.q[ankle_flex_l].restraint_func = NULL;
   sdm.q[ankle_flex_l].min_restraint_func = &q_restraint_func[62];
   sdm.q[ankle_flex_l].max_restraint_func = &q_restraint_func[63];
   sdm.q[ankle_flex_l].function_active = dpNo;
   sdm.q[ankle_flex_l].constraint_func = NULL;
   sdm.q[ankle_flex_l].constraint_num = -1;
   sdm.q[ankle_flex_l].q_ind = -1;
   sdm.q[ankle_flex_l].output = dpYes;
   sdm.q[ankle_flex_l].pd_stiffness = 0.0;
   sdm.q[ankle_flex_l].torque = 0.0;

   mstrcpy(&sdm.q[subt_angle_l].name,"subt_angle_l");
   sdm.q[subt_angle_l].type = dpUnconstrainedQ;
   sdm.q[subt_angle_l].joint = subtalar_l;
   sdm.q[subt_angle_l].axis = 0;
   // initial_value will now be set using setCoordinateInitialValues
   // sdm.q[subt_angle_l].initial_value = 0.0000000000;
   sdm.q[subt_angle_l].initial_velocity = 0.0;
   sdm.q[subt_angle_l].range_start = -0.3490658504;
   sdm.q[subt_angle_l].range_end = 0.3490658504;
   sdm.q[subt_angle_l].restraint_func = NULL;
   sdm.q[subt_angle_l].min_restraint_func = &q_restraint_func[64];
   sdm.q[subt_angle_l].max_restraint_func = &q_restraint_func[65];
   sdm.q[subt_angle_l].function_active = dpNo;
   sdm.q[subt_angle_l].constraint_func = NULL;
   sdm.q[subt_angle_l].constraint_num = -1;
   sdm.q[subt_angle_l].q_ind = -1;
   sdm.q[subt_angle_l].output = dpYes;
   sdm.q[subt_angle_l].pd_stiffness = 0.0;
   sdm.q[subt_angle_l].torque = 0.0;

   mstrcpy(&sdm.q[toe_angle_l].name,"toe_angle_l");
   sdm.q[toe_angle_l].type = dpUnconstrainedQ;
   sdm.q[toe_angle_l].joint = toes_l;
   sdm.q[toe_angle_l].axis = 0;
   // initial_value will now be set using setCoordinateInitialValues
   // sdm.q[toe_angle_l].initial_value = 0.0000000000;
   sdm.q[toe_angle_l].initial_velocity = 0.0;
   sdm.q[toe_angle_l].range_start = -0.5235987756;
   sdm.q[toe_angle_l].range_end = 0.5235987756;
   sdm.q[toe_angle_l].restraint_func = NULL;
   sdm.q[toe_angle_l].min_restraint_func = &q_restraint_func[66];
   sdm.q[toe_angle_l].max_restraint_func = &q_restraint_func[67];
   sdm.q[toe_angle_l].function_active = dpNo;
   sdm.q[toe_angle_l].constraint_func = NULL;
   sdm.q[toe_angle_l].constraint_num = -1;
   sdm.q[toe_angle_l].q_ind = -1;
   sdm.q[toe_angle_l].output = dpYes;
   sdm.q[toe_angle_l].pd_stiffness = 0.0;
   sdm.q[toe_angle_l].torque = 0.0;

   for (i=0, sdm.num_gencoords=0; i<sdm.nq; i++)
      if (sdm.q[i].type == dpUnconstrainedQ)
         sdm.num_gencoords++;

   check_for_sderror("INIT_QS");
}


/* INIT_Q_RESTRAINT_FUNCTIONS: this routine initializes the restraint
 * functions which are used to keep the Qs from exceeding their ranges of motion.
 */

void init_q_restraint_functions(void)
{
   int i, numpts;

   numpts = sizeof(q_restraint_func1_data)/(sizeof(double)*2);
   (void)malloc_function(&q_restraint_func[0],numpts);
   q_restraint_func[0].numpoints = numpts;
   for (i=0; i<numpts; i++)
   {
      q_restraint_func[0].x[i] = q_restraint_func1_data[i][0];
      q_restraint_func[0].y[i] = q_restraint_func1_data[i][1];
   }
   calc_spline_coefficients(&q_restraint_func[0]);

   numpts = sizeof(q_restraint_func2_data)/(sizeof(double)*2);
   (void)malloc_function(&q_restraint_func[1],numpts);
   q_restraint_func[1].numpoints = numpts;
   for (i=0; i<numpts; i++)
   {
      q_restraint_func[1].x[i] = q_restraint_func2_data[i][0];
      q_restraint_func[1].y[i] = q_restraint_func2_data[i][1];
   }
   calc_spline_coefficients(&q_restraint_func[1]);

   numpts = sizeof(q_restraint_func3_data)/(sizeof(double)*2);
   (void)malloc_function(&q_restraint_func[2],numpts);
   q_restraint_func[2].numpoints = numpts;
   for (i=0; i<numpts; i++)
   {
      q_restraint_func[2].x[i] = q_restraint_func3_data[i][0];
      q_restraint_func[2].y[i] = q_restraint_func3_data[i][1];
   }
   calc_spline_coefficients(&q_restraint_func[2]);

   numpts = sizeof(q_restraint_func4_data)/(sizeof(double)*2);
   (void)malloc_function(&q_restraint_func[3],numpts);
   q_restraint_func[3].numpoints = numpts;
   for (i=0; i<numpts; i++)
   {
      q_restraint_func[3].x[i] = q_restraint_func4_data[i][0];
      q_restraint_func[3].y[i] = q_restraint_func4_data[i][1];
   }
   calc_spline_coefficients(&q_restraint_func[3]);

   numpts = sizeof(q_restraint_func5_data)/(sizeof(double)*2);
   (void)malloc_function(&q_restraint_func[4],numpts);
   q_restraint_func[4].numpoints = numpts;
   for (i=0; i<numpts; i++)
   {
      q_restraint_func[4].x[i] = q_restraint_func5_data[i][0];
      q_restraint_func[4].y[i] = q_restraint_func5_data[i][1];
   }
   calc_spline_coefficients(&q_restraint_func[4]);

   numpts = sizeof(q_restraint_func6_data)/(sizeof(double)*2);
   (void)malloc_function(&q_restraint_func[5],numpts);
   q_restraint_func[5].numpoints = numpts;
   for (i=0; i<numpts; i++)
   {
      q_restraint_func[5].x[i] = q_restraint_func6_data[i][0];
      q_restraint_func[5].y[i] = q_restraint_func6_data[i][1];
   }
   calc_spline_coefficients(&q_restraint_func[5]);

   numpts = sizeof(q_restraint_func7_data)/(sizeof(double)*2);
   (void)malloc_function(&q_restraint_func[6],numpts);
   q_restraint_func[6].numpoints = numpts;
   for (i=0; i<numpts; i++)
   {
      q_restraint_func[6].x[i] = q_restraint_func7_data[i][0];
      q_restraint_func[6].y[i] = q_restraint_func7_data[i][1];
   }
   calc_spline_coefficients(&q_restraint_func[6]);

   numpts = sizeof(q_restraint_func8_data)/(sizeof(double)*2);
   (void)malloc_function(&q_restraint_func[7],numpts);
   q_restraint_func[7].numpoints = numpts;
   for (i=0; i<numpts; i++)
   {
      q_restraint_func[7].x[i] = q_restraint_func8_data[i][0];
      q_restraint_func[7].y[i] = q_restraint_func8_data[i][1];
   }
   calc_spline_coefficients(&q_restraint_func[7]);

   numpts = sizeof(q_restraint_func9_data)/(sizeof(double)*2);
   (void)malloc_function(&q_restraint_func[8],numpts);
   q_restraint_func[8].numpoints = numpts;
   for (i=0; i<numpts; i++)
   {
      q_restraint_func[8].x[i] = q_restraint_func9_data[i][0];
      q_restraint_func[8].y[i] = q_restraint_func9_data[i][1];
   }
   calc_spline_coefficients(&q_restraint_func[8]);

   numpts = sizeof(q_restraint_func10_data)/(sizeof(double)*2);
   (void)malloc_function(&q_restraint_func[9],numpts);
   q_restraint_func[9].numpoints = numpts;
   for (i=0; i<numpts; i++)
   {
      q_restraint_func[9].x[i] = q_restraint_func10_data[i][0];
      q_restraint_func[9].y[i] = q_restraint_func10_data[i][1];
   }
   calc_spline_coefficients(&q_restraint_func[9]);

   numpts = sizeof(q_restraint_func11_data)/(sizeof(double)*2);
   (void)malloc_function(&q_restraint_func[10],numpts);
   q_restraint_func[10].numpoints = numpts;
   for (i=0; i<numpts; i++)
   {
      q_restraint_func[10].x[i] = q_restraint_func11_data[i][0];
      q_restraint_func[10].y[i] = q_restraint_func11_data[i][1];
   }
   calc_spline_coefficients(&q_restraint_func[10]);

   numpts = sizeof(q_restraint_func12_data)/(sizeof(double)*2);
   (void)malloc_function(&q_restraint_func[11],numpts);
   q_restraint_func[11].numpoints = numpts;
   for (i=0; i<numpts; i++)
   {
      q_restraint_func[11].x[i] = q_restraint_func12_data[i][0];
      q_restraint_func[11].y[i] = q_restraint_func12_data[i][1];
   }
   calc_spline_coefficients(&q_restraint_func[11]);

   numpts = sizeof(q_restraint_func13_data)/(sizeof(double)*2);
   (void)malloc_function(&q_restraint_func[12],numpts);
   q_restraint_func[12].numpoints = numpts;
   for (i=0; i<numpts; i++)
   {
      q_restraint_func[12].x[i] = q_restraint_func13_data[i][0];
      q_restraint_func[12].y[i] = q_restraint_func13_data[i][1];
   }
   calc_spline_coefficients(&q_restraint_func[12]);

   numpts = sizeof(q_restraint_func14_data)/(sizeof(double)*2);
   (void)malloc_function(&q_restraint_func[13],numpts);
   q_restraint_func[13].numpoints = numpts;
   for (i=0; i<numpts; i++)
   {
      q_restraint_func[13].x[i] = q_restraint_func14_data[i][0];
      q_restraint_func[13].y[i] = q_restraint_func14_data[i][1];
   }
   calc_spline_coefficients(&q_restraint_func[13]);

   numpts = sizeof(q_restraint_func15_data)/(sizeof(double)*2);
   (void)malloc_function(&q_restraint_func[14],numpts);
   q_restraint_func[14].numpoints = numpts;
   for (i=0; i<numpts; i++)
   {
      q_restraint_func[14].x[i] = q_restraint_func15_data[i][0];
      q_restraint_func[14].y[i] = q_restraint_func15_data[i][1];
   }
   calc_spline_coefficients(&q_restraint_func[14]);

   numpts = sizeof(q_restraint_func16_data)/(sizeof(double)*2);
   (void)malloc_function(&q_restraint_func[15],numpts);
   q_restraint_func[15].numpoints = numpts;
   for (i=0; i<numpts; i++)
   {
      q_restraint_func[15].x[i] = q_restraint_func16_data[i][0];
      q_restraint_func[15].y[i] = q_restraint_func16_data[i][1];
   }
   calc_spline_coefficients(&q_restraint_func[15]);

   numpts = sizeof(q_restraint_func17_data)/(sizeof(double)*2);
   (void)malloc_function(&q_restraint_func[16],numpts);
   q_restraint_func[16].numpoints = numpts;
   for (i=0; i<numpts; i++)
   {
      q_restraint_func[16].x[i] = q_restraint_func17_data[i][0];
      q_restraint_func[16].y[i] = q_restraint_func17_data[i][1];
   }
   calc_spline_coefficients(&q_restraint_func[16]);

   numpts = sizeof(q_restraint_func18_data)/(sizeof(double)*2);
   (void)malloc_function(&q_restraint_func[17],numpts);
   q_restraint_func[17].numpoints = numpts;
   for (i=0; i<numpts; i++)
   {
      q_restraint_func[17].x[i] = q_restraint_func18_data[i][0];
      q_restraint_func[17].y[i] = q_restraint_func18_data[i][1];
   }
   calc_spline_coefficients(&q_restraint_func[17]);

   numpts = sizeof(q_restraint_func19_data)/(sizeof(double)*2);
   (void)malloc_function(&q_restraint_func[18],numpts);
   q_restraint_func[18].numpoints = numpts;
   for (i=0; i<numpts; i++)
   {
      q_restraint_func[18].x[i] = q_restraint_func19_data[i][0];
      q_restraint_func[18].y[i] = q_restraint_func19_data[i][1];
   }
   calc_spline_coefficients(&q_restraint_func[18]);

   numpts = sizeof(q_restraint_func20_data)/(sizeof(double)*2);
   (void)malloc_function(&q_restraint_func[19],numpts);
   q_restraint_func[19].numpoints = numpts;
   for (i=0; i<numpts; i++)
   {
      q_restraint_func[19].x[i] = q_restraint_func20_data[i][0];
      q_restraint_func[19].y[i] = q_restraint_func20_data[i][1];
   }
   calc_spline_coefficients(&q_restraint_func[19]);

   numpts = sizeof(q_restraint_func21_data)/(sizeof(double)*2);
   (void)malloc_function(&q_restraint_func[20],numpts);
   q_restraint_func[20].numpoints = numpts;
   for (i=0; i<numpts; i++)
   {
      q_restraint_func[20].x[i] = q_restraint_func21_data[i][0];
      q_restraint_func[20].y[i] = q_restraint_func21_data[i][1];
   }
   calc_spline_coefficients(&q_restraint_func[20]);

   numpts = sizeof(q_restraint_func22_data)/(sizeof(double)*2);
   (void)malloc_function(&q_restraint_func[21],numpts);
   q_restraint_func[21].numpoints = numpts;
   for (i=0; i<numpts; i++)
   {
      q_restraint_func[21].x[i] = q_restraint_func22_data[i][0];
      q_restraint_func[21].y[i] = q_restraint_func22_data[i][1];
   }
   calc_spline_coefficients(&q_restraint_func[21]);

   numpts = sizeof(q_restraint_func23_data)/(sizeof(double)*2);
   (void)malloc_function(&q_restraint_func[22],numpts);
   q_restraint_func[22].numpoints = numpts;
   for (i=0; i<numpts; i++)
   {
      q_restraint_func[22].x[i] = q_restraint_func23_data[i][0];
      q_restraint_func[22].y[i] = q_restraint_func23_data[i][1];
   }
   calc_spline_coefficients(&q_restraint_func[22]);

   numpts = sizeof(q_restraint_func24_data)/(sizeof(double)*2);
   (void)malloc_function(&q_restraint_func[23],numpts);
   q_restraint_func[23].numpoints = numpts;
   for (i=0; i<numpts; i++)
   {
      q_restraint_func[23].x[i] = q_restraint_func24_data[i][0];
      q_restraint_func[23].y[i] = q_restraint_func24_data[i][1];
   }
   calc_spline_coefficients(&q_restraint_func[23]);

   numpts = sizeof(q_restraint_func25_data)/(sizeof(double)*2);
   (void)malloc_function(&q_restraint_func[24],numpts);
   q_restraint_func[24].numpoints = numpts;
   for (i=0; i<numpts; i++)
   {
      q_restraint_func[24].x[i] = q_restraint_func25_data[i][0];
      q_restraint_func[24].y[i] = q_restraint_func25_data[i][1];
   }
   calc_spline_coefficients(&q_restraint_func[24]);

   numpts = sizeof(q_restraint_func26_data)/(sizeof(double)*2);
   (void)malloc_function(&q_restraint_func[25],numpts);
   q_restraint_func[25].numpoints = numpts;
   for (i=0; i<numpts; i++)
   {
      q_restraint_func[25].x[i] = q_restraint_func26_data[i][0];
      q_restraint_func[25].y[i] = q_restraint_func26_data[i][1];
   }
   calc_spline_coefficients(&q_restraint_func[25]);

   numpts = sizeof(q_restraint_func27_data)/(sizeof(double)*2);
   (void)malloc_function(&q_restraint_func[26],numpts);
   q_restraint_func[26].numpoints = numpts;
   for (i=0; i<numpts; i++)
   {
      q_restraint_func[26].x[i] = q_restraint_func27_data[i][0];
      q_restraint_func[26].y[i] = q_restraint_func27_data[i][1];
   }
   calc_spline_coefficients(&q_restraint_func[26]);

   numpts = sizeof(q_restraint_func28_data)/(sizeof(double)*2);
   (void)malloc_function(&q_restraint_func[27],numpts);
   q_restraint_func[27].numpoints = numpts;
   for (i=0; i<numpts; i++)
   {
      q_restraint_func[27].x[i] = q_restraint_func28_data[i][0];
      q_restraint_func[27].y[i] = q_restraint_func28_data[i][1];
   }
   calc_spline_coefficients(&q_restraint_func[27]);

   numpts = sizeof(q_restraint_func29_data)/(sizeof(double)*2);
   (void)malloc_function(&q_restraint_func[28],numpts);
   q_restraint_func[28].numpoints = numpts;
   for (i=0; i<numpts; i++)
   {
      q_restraint_func[28].x[i] = q_restraint_func29_data[i][0];
      q_restraint_func[28].y[i] = q_restraint_func29_data[i][1];
   }
   calc_spline_coefficients(&q_restraint_func[28]);

   numpts = sizeof(q_restraint_func30_data)/(sizeof(double)*2);
   (void)malloc_function(&q_restraint_func[29],numpts);
   q_restraint_func[29].numpoints = numpts;
   for (i=0; i<numpts; i++)
   {
      q_restraint_func[29].x[i] = q_restraint_func30_data[i][0];
      q_restraint_func[29].y[i] = q_restraint_func30_data[i][1];
   }
   calc_spline_coefficients(&q_restraint_func[29]);

   numpts = sizeof(q_restraint_func31_data)/(sizeof(double)*2);
   (void)malloc_function(&q_restraint_func[30],numpts);
   q_restraint_func[30].numpoints = numpts;
   for (i=0; i<numpts; i++)
   {
      q_restraint_func[30].x[i] = q_restraint_func31_data[i][0];
      q_restraint_func[30].y[i] = q_restraint_func31_data[i][1];
   }
   calc_spline_coefficients(&q_restraint_func[30]);

   numpts = sizeof(q_restraint_func32_data)/(sizeof(double)*2);
   (void)malloc_function(&q_restraint_func[31],numpts);
   q_restraint_func[31].numpoints = numpts;
   for (i=0; i<numpts; i++)
   {
      q_restraint_func[31].x[i] = q_restraint_func32_data[i][0];
      q_restraint_func[31].y[i] = q_restraint_func32_data[i][1];
   }
   calc_spline_coefficients(&q_restraint_func[31]);

   numpts = sizeof(q_restraint_func33_data)/(sizeof(double)*2);
   (void)malloc_function(&q_restraint_func[32],numpts);
   q_restraint_func[32].numpoints = numpts;
   for (i=0; i<numpts; i++)
   {
      q_restraint_func[32].x[i] = q_restraint_func33_data[i][0];
      q_restraint_func[32].y[i] = q_restraint_func33_data[i][1];
   }
   calc_spline_coefficients(&q_restraint_func[32]);

   numpts = sizeof(q_restraint_func34_data)/(sizeof(double)*2);
   (void)malloc_function(&q_restraint_func[33],numpts);
   q_restraint_func[33].numpoints = numpts;
   for (i=0; i<numpts; i++)
   {
      q_restraint_func[33].x[i] = q_restraint_func34_data[i][0];
      q_restraint_func[33].y[i] = q_restraint_func34_data[i][1];
   }
   calc_spline_coefficients(&q_restraint_func[33]);

   numpts = sizeof(q_restraint_func35_data)/(sizeof(double)*2);
   (void)malloc_function(&q_restraint_func[34],numpts);
   q_restraint_func[34].numpoints = numpts;
   for (i=0; i<numpts; i++)
   {
      q_restraint_func[34].x[i] = q_restraint_func35_data[i][0];
      q_restraint_func[34].y[i] = q_restraint_func35_data[i][1];
   }
   calc_spline_coefficients(&q_restraint_func[34]);

   numpts = sizeof(q_restraint_func36_data)/(sizeof(double)*2);
   (void)malloc_function(&q_restraint_func[35],numpts);
   q_restraint_func[35].numpoints = numpts;
   for (i=0; i<numpts; i++)
   {
      q_restraint_func[35].x[i] = q_restraint_func36_data[i][0];
      q_restraint_func[35].y[i] = q_restraint_func36_data[i][1];
   }
   calc_spline_coefficients(&q_restraint_func[35]);

   numpts = sizeof(q_restraint_func37_data)/(sizeof(double)*2);
   (void)malloc_function(&q_restraint_func[36],numpts);
   q_restraint_func[36].numpoints = numpts;
   for (i=0; i<numpts; i++)
   {
      q_restraint_func[36].x[i] = q_restraint_func37_data[i][0];
      q_restraint_func[36].y[i] = q_restraint_func37_data[i][1];
   }
   calc_spline_coefficients(&q_restraint_func[36]);

   numpts = sizeof(q_restraint_func38_data)/(sizeof(double)*2);
   (void)malloc_function(&q_restraint_func[37],numpts);
   q_restraint_func[37].numpoints = numpts;
   for (i=0; i<numpts; i++)
   {
      q_restraint_func[37].x[i] = q_restraint_func38_data[i][0];
      q_restraint_func[37].y[i] = q_restraint_func38_data[i][1];
   }
   calc_spline_coefficients(&q_restraint_func[37]);

   numpts = sizeof(q_restraint_func39_data)/(sizeof(double)*2);
   (void)malloc_function(&q_restraint_func[38],numpts);
   q_restraint_func[38].numpoints = numpts;
   for (i=0; i<numpts; i++)
   {
      q_restraint_func[38].x[i] = q_restraint_func39_data[i][0];
      q_restraint_func[38].y[i] = q_restraint_func39_data[i][1];
   }
   calc_spline_coefficients(&q_restraint_func[38]);

   numpts = sizeof(q_restraint_func40_data)/(sizeof(double)*2);
   (void)malloc_function(&q_restraint_func[39],numpts);
   q_restraint_func[39].numpoints = numpts;
   for (i=0; i<numpts; i++)
   {
      q_restraint_func[39].x[i] = q_restraint_func40_data[i][0];
      q_restraint_func[39].y[i] = q_restraint_func40_data[i][1];
   }
   calc_spline_coefficients(&q_restraint_func[39]);

   numpts = sizeof(q_restraint_func41_data)/(sizeof(double)*2);
   (void)malloc_function(&q_restraint_func[40],numpts);
   q_restraint_func[40].numpoints = numpts;
   for (i=0; i<numpts; i++)
   {
      q_restraint_func[40].x[i] = q_restraint_func41_data[i][0];
      q_restraint_func[40].y[i] = q_restraint_func41_data[i][1];
   }
   calc_spline_coefficients(&q_restraint_func[40]);

   numpts = sizeof(q_restraint_func42_data)/(sizeof(double)*2);
   (void)malloc_function(&q_restraint_func[41],numpts);
   q_restraint_func[41].numpoints = numpts;
   for (i=0; i<numpts; i++)
   {
      q_restraint_func[41].x[i] = q_restraint_func42_data[i][0];
      q_restraint_func[41].y[i] = q_restraint_func42_data[i][1];
   }
   calc_spline_coefficients(&q_restraint_func[41]);

   numpts = sizeof(q_restraint_func43_data)/(sizeof(double)*2);
   (void)malloc_function(&q_restraint_func[42],numpts);
   q_restraint_func[42].numpoints = numpts;
   for (i=0; i<numpts; i++)
   {
      q_restraint_func[42].x[i] = q_restraint_func43_data[i][0];
      q_restraint_func[42].y[i] = q_restraint_func43_data[i][1];
   }
   calc_spline_coefficients(&q_restraint_func[42]);

   numpts = sizeof(q_restraint_func44_data)/(sizeof(double)*2);
   (void)malloc_function(&q_restraint_func[43],numpts);
   q_restraint_func[43].numpoints = numpts;
   for (i=0; i<numpts; i++)
   {
      q_restraint_func[43].x[i] = q_restraint_func44_data[i][0];
      q_restraint_func[43].y[i] = q_restraint_func44_data[i][1];
   }
   calc_spline_coefficients(&q_restraint_func[43]);

   numpts = sizeof(q_restraint_func45_data)/(sizeof(double)*2);
   (void)malloc_function(&q_restraint_func[44],numpts);
   q_restraint_func[44].numpoints = numpts;
   for (i=0; i<numpts; i++)
   {
      q_restraint_func[44].x[i] = q_restraint_func45_data[i][0];
      q_restraint_func[44].y[i] = q_restraint_func45_data[i][1];
   }
   calc_spline_coefficients(&q_restraint_func[44]);

   numpts = sizeof(q_restraint_func46_data)/(sizeof(double)*2);
   (void)malloc_function(&q_restraint_func[45],numpts);
   q_restraint_func[45].numpoints = numpts;
   for (i=0; i<numpts; i++)
   {
      q_restraint_func[45].x[i] = q_restraint_func46_data[i][0];
      q_restraint_func[45].y[i] = q_restraint_func46_data[i][1];
   }
   calc_spline_coefficients(&q_restraint_func[45]);

   numpts = sizeof(q_restraint_func47_data)/(sizeof(double)*2);
   (void)malloc_function(&q_restraint_func[46],numpts);
   q_restraint_func[46].numpoints = numpts;
   for (i=0; i<numpts; i++)
   {
      q_restraint_func[46].x[i] = q_restraint_func47_data[i][0];
      q_restraint_func[46].y[i] = q_restraint_func47_data[i][1];
   }
   calc_spline_coefficients(&q_restraint_func[46]);

   numpts = sizeof(q_restraint_func48_data)/(sizeof(double)*2);
   (void)malloc_function(&q_restraint_func[47],numpts);
   q_restraint_func[47].numpoints = numpts;
   for (i=0; i<numpts; i++)
   {
      q_restraint_func[47].x[i] = q_restraint_func48_data[i][0];
      q_restraint_func[47].y[i] = q_restraint_func48_data[i][1];
   }
   calc_spline_coefficients(&q_restraint_func[47]);

   numpts = sizeof(q_restraint_func49_data)/(sizeof(double)*2);
   (void)malloc_function(&q_restraint_func[48],numpts);
   q_restraint_func[48].numpoints = numpts;
   for (i=0; i<numpts; i++)
   {
      q_restraint_func[48].x[i] = q_restraint_func49_data[i][0];
      q_restraint_func[48].y[i] = q_restraint_func49_data[i][1];
   }
   calc_spline_coefficients(&q_restraint_func[48]);

   numpts = sizeof(q_restraint_func50_data)/(sizeof(double)*2);
   (void)malloc_function(&q_restraint_func[49],numpts);
   q_restraint_func[49].numpoints = numpts;
   for (i=0; i<numpts; i++)
   {
      q_restraint_func[49].x[i] = q_restraint_func50_data[i][0];
      q_restraint_func[49].y[i] = q_restraint_func50_data[i][1];
   }
   calc_spline_coefficients(&q_restraint_func[49]);

   numpts = sizeof(q_restraint_func51_data)/(sizeof(double)*2);
   (void)malloc_function(&q_restraint_func[50],numpts);
   q_restraint_func[50].numpoints = numpts;
   for (i=0; i<numpts; i++)
   {
      q_restraint_func[50].x[i] = q_restraint_func51_data[i][0];
      q_restraint_func[50].y[i] = q_restraint_func51_data[i][1];
   }
   calc_spline_coefficients(&q_restraint_func[50]);

   numpts = sizeof(q_restraint_func52_data)/(sizeof(double)*2);
   (void)malloc_function(&q_restraint_func[51],numpts);
   q_restraint_func[51].numpoints = numpts;
   for (i=0; i<numpts; i++)
   {
      q_restraint_func[51].x[i] = q_restraint_func52_data[i][0];
      q_restraint_func[51].y[i] = q_restraint_func52_data[i][1];
   }
   calc_spline_coefficients(&q_restraint_func[51]);

   numpts = sizeof(q_restraint_func53_data)/(sizeof(double)*2);
   (void)malloc_function(&q_restraint_func[52],numpts);
   q_restraint_func[52].numpoints = numpts;
   for (i=0; i<numpts; i++)
   {
      q_restraint_func[52].x[i] = q_restraint_func53_data[i][0];
      q_restraint_func[52].y[i] = q_restraint_func53_data[i][1];
   }
   calc_spline_coefficients(&q_restraint_func[52]);

   numpts = sizeof(q_restraint_func54_data)/(sizeof(double)*2);
   (void)malloc_function(&q_restraint_func[53],numpts);
   q_restraint_func[53].numpoints = numpts;
   for (i=0; i<numpts; i++)
   {
      q_restraint_func[53].x[i] = q_restraint_func54_data[i][0];
      q_restraint_func[53].y[i] = q_restraint_func54_data[i][1];
   }
   calc_spline_coefficients(&q_restraint_func[53]);

   numpts = sizeof(q_restraint_func55_data)/(sizeof(double)*2);
   (void)malloc_function(&q_restraint_func[54],numpts);
   q_restraint_func[54].numpoints = numpts;
   for (i=0; i<numpts; i++)
   {
      q_restraint_func[54].x[i] = q_restraint_func55_data[i][0];
      q_restraint_func[54].y[i] = q_restraint_func55_data[i][1];
   }
   calc_spline_coefficients(&q_restraint_func[54]);

   numpts = sizeof(q_restraint_func56_data)/(sizeof(double)*2);
   (void)malloc_function(&q_restraint_func[55],numpts);
   q_restraint_func[55].numpoints = numpts;
   for (i=0; i<numpts; i++)
   {
      q_restraint_func[55].x[i] = q_restraint_func56_data[i][0];
      q_restraint_func[55].y[i] = q_restraint_func56_data[i][1];
   }
   calc_spline_coefficients(&q_restraint_func[55]);

   numpts = sizeof(q_restraint_func57_data)/(sizeof(double)*2);
   (void)malloc_function(&q_restraint_func[56],numpts);
   q_restraint_func[56].numpoints = numpts;
   for (i=0; i<numpts; i++)
   {
      q_restraint_func[56].x[i] = q_restraint_func57_data[i][0];
      q_restraint_func[56].y[i] = q_restraint_func57_data[i][1];
   }
   calc_spline_coefficients(&q_restraint_func[56]);

   numpts = sizeof(q_restraint_func58_data)/(sizeof(double)*2);
   (void)malloc_function(&q_restraint_func[57],numpts);
   q_restraint_func[57].numpoints = numpts;
   for (i=0; i<numpts; i++)
   {
      q_restraint_func[57].x[i] = q_restraint_func58_data[i][0];
      q_restraint_func[57].y[i] = q_restraint_func58_data[i][1];
   }
   calc_spline_coefficients(&q_restraint_func[57]);

   numpts = sizeof(q_restraint_func59_data)/(sizeof(double)*2);
   (void)malloc_function(&q_restraint_func[58],numpts);
   q_restraint_func[58].numpoints = numpts;
   for (i=0; i<numpts; i++)
   {
      q_restraint_func[58].x[i] = q_restraint_func59_data[i][0];
      q_restraint_func[58].y[i] = q_restraint_func59_data[i][1];
   }
   calc_spline_coefficients(&q_restraint_func[58]);

   numpts = sizeof(q_restraint_func60_data)/(sizeof(double)*2);
   (void)malloc_function(&q_restraint_func[59],numpts);
   q_restraint_func[59].numpoints = numpts;
   for (i=0; i<numpts; i++)
   {
      q_restraint_func[59].x[i] = q_restraint_func60_data[i][0];
      q_restraint_func[59].y[i] = q_restraint_func60_data[i][1];
   }
   calc_spline_coefficients(&q_restraint_func[59]);

   numpts = sizeof(q_restraint_func61_data)/(sizeof(double)*2);
   (void)malloc_function(&q_restraint_func[60],numpts);
   q_restraint_func[60].numpoints = numpts;
   for (i=0; i<numpts; i++)
   {
      q_restraint_func[60].x[i] = q_restraint_func61_data[i][0];
      q_restraint_func[60].y[i] = q_restraint_func61_data[i][1];
   }
   calc_spline_coefficients(&q_restraint_func[60]);

   numpts = sizeof(q_restraint_func62_data)/(sizeof(double)*2);
   (void)malloc_function(&q_restraint_func[61],numpts);
   q_restraint_func[61].numpoints = numpts;
   for (i=0; i<numpts; i++)
   {
      q_restraint_func[61].x[i] = q_restraint_func62_data[i][0];
      q_restraint_func[61].y[i] = q_restraint_func62_data[i][1];
   }
   calc_spline_coefficients(&q_restraint_func[61]);

   numpts = sizeof(q_restraint_func63_data)/(sizeof(double)*2);
   (void)malloc_function(&q_restraint_func[62],numpts);
   q_restraint_func[62].numpoints = numpts;
   for (i=0; i<numpts; i++)
   {
      q_restraint_func[62].x[i] = q_restraint_func63_data[i][0];
      q_restraint_func[62].y[i] = q_restraint_func63_data[i][1];
   }
   calc_spline_coefficients(&q_restraint_func[62]);

   numpts = sizeof(q_restraint_func64_data)/(sizeof(double)*2);
   (void)malloc_function(&q_restraint_func[63],numpts);
   q_restraint_func[63].numpoints = numpts;
   for (i=0; i<numpts; i++)
   {
      q_restraint_func[63].x[i] = q_restraint_func64_data[i][0];
      q_restraint_func[63].y[i] = q_restraint_func64_data[i][1];
   }
   calc_spline_coefficients(&q_restraint_func[63]);

   numpts = sizeof(q_restraint_func65_data)/(sizeof(double)*2);
   (void)malloc_function(&q_restraint_func[64],numpts);
   q_restraint_func[64].numpoints = numpts;
   for (i=0; i<numpts; i++)
   {
      q_restraint_func[64].x[i] = q_restraint_func65_data[i][0];
      q_restraint_func[64].y[i] = q_restraint_func65_data[i][1];
   }
   calc_spline_coefficients(&q_restraint_func[64]);

   numpts = sizeof(q_restraint_func66_data)/(sizeof(double)*2);
   (void)malloc_function(&q_restraint_func[65],numpts);
   q_restraint_func[65].numpoints = numpts;
   for (i=0; i<numpts; i++)
   {
      q_restraint_func[65].x[i] = q_restraint_func66_data[i][0];
      q_restraint_func[65].y[i] = q_restraint_func66_data[i][1];
   }
   calc_spline_coefficients(&q_restraint_func[65]);

   numpts = sizeof(q_restraint_func67_data)/(sizeof(double)*2);
   (void)malloc_function(&q_restraint_func[66],numpts);
   q_restraint_func[66].numpoints = numpts;
   for (i=0; i<numpts; i++)
   {
      q_restraint_func[66].x[i] = q_restraint_func67_data[i][0];
      q_restraint_func[66].y[i] = q_restraint_func67_data[i][1];
   }
   calc_spline_coefficients(&q_restraint_func[66]);

   numpts = sizeof(q_restraint_func68_data)/(sizeof(double)*2);
   (void)malloc_function(&q_restraint_func[67],numpts);
   q_restraint_func[67].numpoints = numpts;
   for (i=0; i<numpts; i++)
   {
      q_restraint_func[67].x[i] = q_restraint_func68_data[i][0];
      q_restraint_func[67].y[i] = q_restraint_func68_data[i][1];
   }
   calc_spline_coefficients(&q_restraint_func[67]);

}


void init_wrap_objects(void)
{

   /* Wrap objects are handled by the native OpenSim code, so */
   /* they are not exported to the Pipeline source code. */

  sdm.num_wrap_objects = 0;
  sdm.wrap_object = NULL;

}

void init_constraint_objects(void)
{

   /* There are no constraint objects in this model. */

  sdm.num_constraint_objects = 0;
  sdm.constraint_object = NULL;
}

