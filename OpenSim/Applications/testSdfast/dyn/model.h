/*******************************************************************************

  model.h

  08/07/2006 03:55:01 PM

*******************************************************************************/

/*********** Joints ************/
#define             gnd_pelvis  0
#define           pelvis_torso  1
#define         torso_neckhead  2
#define             acromial_r  3
#define                elbow_r  4
#define           radioulnar_r  5
#define          radius_hand_r  6
#define             acromial_l  7
#define                elbow_l  8
#define           radioulnar_l  9
#define          radius_hand_l 10
#define                  hip_r 11
#define         knee_flexion_r 12
#define              tib_pat_r 13
#define                ankle_r 14
#define             subtalar_r 15
#define                 toes_r 16
#define                  hip_l 17
#define         knee_flexion_l 18
#define              tib_pat_l 19
#define                ankle_l 20
#define             subtalar_l 21
#define                 toes_l 22

/************* Qs **************/
#define         lower_torso_TX  0   /* = sdindx(gnd_pelvis,0) */
#define         lower_torso_TY  1   /* = sdindx(gnd_pelvis,1) */
#define         lower_torso_TZ  2   /* = sdindx(gnd_pelvis,2) */
#define         lower_torso_RX  3   /* = sdindx(gnd_pelvis,3) */
#define         lower_torso_RY  4   /* = sdindx(gnd_pelvis,4) */
#define         lower_torso_RZ  5   /* = sdindx(gnd_pelvis,5) */
#define           lumbar_pitch  6   /* = sdindx(pelvis_torso,0) */
#define            lumbar_roll  7   /* = sdindx(pelvis_torso,1) */
#define             lumbar_yaw  8   /* = sdindx(pelvis_torso,2) */
#define             neck_pitch  9   /* = sdindx(torso_neckhead,0) */
#define              neck_roll 10   /* = sdindx(torso_neckhead,1) */
#define               neck_yaw 11   /* = sdindx(torso_neckhead,2) */
#define              arm_add_r 12   /* = sdindx(acromial_r,0) */
#define             arm_flex_r 13   /* = sdindx(acromial_r,1) */
#define              arm_rot_r 14   /* = sdindx(acromial_r,2) */
#define           elbow_flex_r 15   /* = sdindx(elbow_r,0) */
#define              pro_sup_r 16   /* = sdindx(radioulnar_r,0) */
#define           wrist_flex_r 17   /* = sdindx(radius_hand_r,0) */
#define            wrist_dev_r 18   /* = sdindx(radius_hand_r,1) */
#define              arm_add_l 19   /* = sdindx(acromial_l,0) */
#define             arm_flex_l 20   /* = sdindx(acromial_l,1) */
#define              arm_rot_l 21   /* = sdindx(acromial_l,2) */
#define           elbow_flex_l 22   /* = sdindx(elbow_l,0) */
#define              pro_sup_l 23   /* = sdindx(radioulnar_l,0) */
#define           wrist_flex_l 24   /* = sdindx(radius_hand_l,0) */
#define            wrist_dev_l 25   /* = sdindx(radius_hand_l,1) */
#define             hip_flex_r 26   /* = sdindx(hip_r,0) */
#define              hip_add_r 27   /* = sdindx(hip_r,1) */
#define              hip_rot_r 28   /* = sdindx(hip_r,2) */
#define      knee_flexion_r_tx 29   /* = sdindx(knee_flexion_r,0) */
#define      knee_flexion_r_ty 30   /* = sdindx(knee_flexion_r,1) */
#define            knee_flex_r 31   /* = sdindx(knee_flexion_r,2) */
#define           tib_pat_r_tx 32   /* = sdindx(tib_pat_r,0) */
#define           tib_pat_r_ty 33   /* = sdindx(tib_pat_r,1) */
#define           tib_pat_r_r1 34   /* = sdindx(tib_pat_r,2) */
#define           ankle_flex_r 35   /* = sdindx(ankle_r,0) */
#define           subt_angle_r 36   /* = sdindx(subtalar_r,0) */
#define            toe_angle_r 37   /* = sdindx(toes_r,0) */
#define             hip_flex_l 38   /* = sdindx(hip_l,0) */
#define              hip_add_l 39   /* = sdindx(hip_l,1) */
#define              hip_rot_l 40   /* = sdindx(hip_l,2) */
#define      knee_flexion_l_tx 41   /* = sdindx(knee_flexion_l,0) */
#define      knee_flexion_l_ty 42   /* = sdindx(knee_flexion_l,1) */
#define            knee_flex_l 43   /* = sdindx(knee_flexion_l,2) */
#define           tib_pat_l_tx 44   /* = sdindx(tib_pat_l,0) */
#define           tib_pat_l_ty 45   /* = sdindx(tib_pat_l,1) */
#define           tib_pat_l_r1 46   /* = sdindx(tib_pat_l,2) */
#define           ankle_flex_l 47   /* = sdindx(ankle_l,0) */
#define           subt_angle_l 48   /* = sdindx(subtalar_l,0) */
#define            toe_angle_l 49   /* = sdindx(toes_l,0) */

/******* Constrained Qs ********/
#define  knee_flexion_r_tx_con  0
#define  knee_flexion_r_ty_con  1
#define       tib_pat_r_tx_con  2
#define       tib_pat_r_ty_con  3
#define       tib_pat_r_r1_con  4
#define  knee_flexion_l_tx_con  5
#define  knee_flexion_l_ty_con  6
#define       tib_pat_l_tx_con  7
#define       tib_pat_l_ty_con  8
#define       tib_pat_l_r1_con  9

/******** Body Segments ********/
#define                 ground -1
#define                 pelvis  0
#define                  torso  1
#define               neckhead  2
#define              humerus_r  3
#define                 ulna_r  4
#define               radius_r  5
#define                 hand_r  6
#define              humerus_l  7
#define                 ulna_l  8
#define               radius_l  9
#define                 hand_l 10
#define                femur_r 11
#define                tibia_r 12
#define              patella_r 13
#define                talus_r 14
#define                 foot_r 15
#define                 toes_r 16
#define                femur_l 17
#define                tibia_l 18
#define              patella_l 19
#define                talus_l 20
#define                 foot_l 21
#define                 toes_l 22
