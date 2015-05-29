/*******************************************************************************

   LCSOLVETOOLS.C

   Author: Krystyne Blaikie, Peter Loan

   Date: 8-APR-03

   Copyright (c) 1992-5 MusculoGraphics, Inc.
   All rights reserved.
   Portions of this source code are copyrighted by MusculoGraphics, Inc.

   Description: This file contains a bunch of generic tool
      routines that are used in a wide range of circumstances.

   Routines:
   solve_initial_loops_and_constraints:
   solveAllLoopsAndConstraints:
   solveLCaffectedByGC:
   solveLCaffectedByJNT:
   calculateLCResids:
   calculateGCResiduals:
   calculateLoopResiduals:

*******************************************************************************/
#include "universal.h"
#include "globals.h"
#include "functions.h"

/*************** STATIC GLOBAL VARIABLES (for this file only) *****************/

/*************** EXTERNED VARIABLES (declared in another file) ****************/
extern int JACOBIAN;

/*************** GLOBAL VARIABLES (used in only a few files) ******************/
char gencoordResidualErrorMsg[] = "The IK solver found a loop configuration with large gencoord\n"
                                  "residuals. This usually means that either:\n"
                                  "(1) too many of the gencoords are clamped or locked\n"
                                  "    Solution: unclamp and unlock the gencoords, and turn the\n"
                                  "    IK solver off and then on again.\n"
                                  "(2) one or more of the gencoord restraint functions\n"
                                  "    are returning large errors.\n"
                                  "    Solution: look for poorly defined restraint functions\n"
                                  "    in the Gencoord Editor and fix them.\n\n"
                                  "Once you find a good configuration for your model, you should re-save\n"
                                  "the new gencoord values as defaults using the \"save gc values\"\n"
                                  "command in the Model Viewer.\n";
char badGencoordErrorMsg2[] = "Satisfying the constraints generated large gencoord\n"
                                  "residuals. This usually means that either:\n"
                                  "(1) too many of the gencoords are clamped or locked\n"
                                  "    Solution: unclamp and unlock the gencoords.\n"
                                  "(2) one or more of the gencoord restraint functions\n"
                                  "    are returning large errors.\n"
                                  "    Solution: look for poorly defined restraint functions\n"
                                  "    in the Gencoord Editor and fix them.\n\n"
                                  "Once you find a good configuration for your model, you should re-save\n"
                                  "the new gencoord values as defaults using the \"save gc values\"\n"
                                  "command in the Model Viewer.\n";
char badLoopErrorMsg[] = "The IK solver could not find a configuration that closed all the loops.\n"
                         "This could mean that either:\n"
                         "(1) the joints in the loop may be poorly defined\n"
                         "    Solution: check the joint definitions to see if a solution is possible.\n"
                         "    Try turning the IK solver off and moving gencoords manually.\n"
                         "(2) one or more of the gencoord restraint functions\n"
                         "    is returning large errors.\n"
                         "    Solution: look for poorly defined restraint functions\n"
                         "    in the Gencoord Editor and fix them.\n"
                         "Once you find a good configuration for your model, you should\n"
                         "save the gencoord values as defaults using the \"save gc values\"\n"
                         "command in the Model Viewer.\n"
                         "If none of these suggestions help, turn the IK solver off.\n\n";
char badConstraintErrorMsg[] = "Could not find a configuration that satisfied all the constraints.\n"
                         "This could mean that either:\n"
                         "(1) the constraints are poorly defined\n"
                         "    Solution: check the definitions to see if a solution is possible.\n"
                         "(2) one or more of the gencoord restraint functions\n"
                         "    is returning large errors.\n"
                         "    Solution: look for poorly defined restraint functions\n"
                         "    in the Gencoord Editor and fix them.\n"
                         "Once you find a good configuration for your model, you should\n"
                         "save the gencoord values as defaults using the \"save gc values\"\n"
                         "command in the Model Viewer.\n\n";

/*************** DEFINES (for this file only) *********************************/
/* max residual allowed for each gencoord */
double GENCOORD_TOLERANCE = 0.00001;
/* clamping tolerances used to keep gencoords within range when clamped.
 * linear function used to calculate residuals.  Residual is zero at
 * clamping tolerance and increases linearly towards limits. */
#define CLAMPING_TOLERANCE 0.0 /* JPL 08/05/04 was 10.0 */
#define CLAMPING_TOL_TRANS 0.001

/********************** FUNCTION PROTOTYPES ***********************************/
void calculateLCResids(int numResid, int numQ, double q[], double residuals[],
                       int *iflag, void *data);

/*************** PROTOTYPES for STATIC FUNCTIONS (for this file only) *********/
static void markAffectedLoopsAndConstraints(LCStruct *info, GeneralizedCoord* gencoord);
static void markLoopsAndConstraintsAffected(LCStruct *info, int jnt);
static void calculateGenCoordResiduals(void *data, int numQ, double q[], int numResid, double resid[], 
                                       int startIndex, int endIndex, double *weight, int *iflag);
static void calculateLoopResiduals(void *data, int numQ, double q[], int numResid, double resid[], 
                                   int startIndex, int endIndex, double *weight, int *iflag);
static void orderQs(ModelStruct *ms, int *nq, GeneralizedCoord* gc_list[], GeneralizedCoord* controlled_gc);
static void evaluateLCSolution(LCStruct *info, int nq, double q[], int nres, double resid[]);
static void updateModel(ModelStruct *ms, LCStruct *solveInfo, int nq, double q[], 
                        double saved_q[], int nres, double resid[], SBoolean *changed);
static void displayLCMessages(ModelStruct *ms, LoopStatus loopStatus, ConstraintStatus constraintStatus);
static double calculateRMS(int n, double fvec[]);
static int LeastSquaresSolver(LCStruct *solveInfo, int numQ, double q[], int numResid, double resid[]);
static SBoolean JNTUsedInConstraint(int jnt, ConstraintObject co);
static SBoolean GCUsedInConstraint(GeneralizedCoord* gencoord, ConstraintObject co);
static SBoolean GCUsedInLoop(GeneralizedCoord* gencoord, LoopStruct loop);
static SBoolean JNTUsedInLoop(int jnt, LoopStruct loop);

/* SOLVE_INITIAL_LOOPS_AND_CONSTRAINTS:
 * Solve any loops and constraints in the model and display any error
 * messages if necessary */
void solve_initial_loops_and_constraints(ModelStruct * ms)
{
   ConstraintStatus constraintStatus;
   LoopStatus loopStatus;

   evaluateLoopsAndConstraintsInCurrentConfiguration(ms, &loopStatus, &constraintStatus, yes);

   if (loopStatus != loopUnchanged || constraintStatus != constraintUnchanged)
      solveAllLoopsAndConstraints(ms, &loopStatus, &constraintStatus, yes);

#if ! ENGINE
   displayLCMessages(ms, loopStatus, constraintStatus);
#endif
}

void recalc_default_loops_and_constraints(ModelStruct *ms)
{
   int i, ans;
   LoopStatus loopStatus;
   ConstraintStatus constraintStatus;
   char gc_info[255];

   solveAllLoopsAndConstraints(ms, &loopStatus, &constraintStatus, yes);

   if ((loopStatus == loopChanged) || (constraintStatus == constraintChanged))
   {
      approveNewDefaultGCs(ms);
   }
   else if ((loopStatus == loopBroken) || (constraintStatus == constraintBroken))
   {
#if ! ENGINE
      glutMessageBox(GLUT_OK, 1, GLUT_ICON_INFORMATION,
         "Default Configuration: IK Solver could not close loops or constraints", badLoopErrorMsg);
#endif
   }
   else if ((loopStatus == largeResidinLoop) || (constraintStatus == gcOutOfRange))
   {
#if ! ENGINE
      glutMessageBox(GLUT_OK, 1, GLUT_ICON_INFORMATION,
         "Default Configuration: Check Restraint Functions", gencoordResidualErrorMsg);
#endif
      ms->GEFuncOK = yes;
   }
}

/* SOLVE_ALL_LOOPS_AND_CONSTRAINTS:
 * Solve all loops and constraints at once.  If enforce_constraints = yes,
 * solve constraints (if not, do not).  Return the status of loops and constraints
 */
void solveAllLoopsAndConstraints(ModelStruct *ms, LoopStatus *loopStatus, 
                                 ConstraintStatus *constraintStatus, 
                                 SBoolean enforce_constraints)
{
   int i, j, nq, nres, index;
   int numLoopQs, numLoopResids, numConstraintQs, numConstraintResids;
   double *q = NULL, *saved_q = NULL, *resid = NULL;
   LCStruct *solveInfo = NULL;
   ReturnCode rc;
   SBoolean changed = no;

   *loopStatus = loopUnchanged;
   *constraintStatus = constraintUnchanged;

   if ((!loopsToSolve(ms)) && (!constraintsToSolve(ms)))
      return;

   solveInfo = (LCStruct *)simm_malloc(sizeof(LCStruct));
   solveInfo->model = ms;
   solveInfo->first_iter = yes;
   solveInfo->largeGenCoordResids = no;
   solveInfo->loopClosed = yes;
   solveInfo->constraintClosed = yes;
   solveInfo->controlled_gc = NULL;
   solveInfo->controlled_value = 0.0;
   solveInfo->gencoord_list = NULL;

   /* create structure to hold info about loops being solved */
   solveInfo->loopInfo = (IKStruct *)simm_malloc(sizeof(IKStruct));
   solveInfo->loopInfo->ms = ms;
   solveInfo->loopInfo->loopUsed = (SBoolean *)simm_malloc(ms->numclosedloops * sizeof(SBoolean));
   solveInfo->loopInfo->gencoord_list = NULL;
   solveInfo->loopInfo->controlled_gc = NULL;
   solveInfo->loopInfo->controlled_value = 0.0;
   solveInfo->loopInfo->first_iter = yes;

   /* create structure to hold info about constraints being solved */
   solveInfo->constraintInfo = (ConstraintSolverStruct *)simm_malloc(sizeof(ConstraintSolverStruct));
   solveInfo->constraintInfo->model = ms;
   solveInfo->constraintInfo->controlled_gc = NULL;
   solveInfo->constraintInfo->controlled_value = 0.0;
   solveInfo->constraintInfo->gencoord_list = NULL;
   solveInfo->constraintInfo->first_iter = yes;
   solveInfo->constraintInfo->test = no;
//   solveInfo->constraintInfo->tolerance = ms->constraint_tolerance;
   solveInfo->constraintInfo->consUsed = (SBoolean *)simm_malloc(ms->num_constraint_objects * sizeof(SBoolean));

   nq = 0;
   nres = 0;
   numLoopQs = 0;
   numLoopResids = 0;
   numConstraintQs = 0;
   numConstraintResids = 0;

   /* Calculate the total number of qs and residuals in loops. */ 
   for (i = 0; i < ms->numclosedloops; i++)
   {
      numLoopQs += ms->loop[i].num_qs;
      numLoopResids += ms->loop[i].num_resids;
      solveInfo->loopInfo->loopUsed[i] = yes;
   }  

   /* Calculate the total number of qs and residuals in constraints. */ 
   for (i = 0; i < ms->num_constraint_objects; i++)
   {
      solveInfo->constraintInfo->consUsed[i] = no;
      if ((enforce_constraints == yes) && (ms->constraintobj[i].active == yes)
         && (ms->constraintobj[i].numPoints > 0))
      {
         numConstraintQs += ms->constraintobj[i].num_qs;
         numConstraintResids += ms->constraintobj[i].numPoints;
         solveInfo->constraintInfo->consUsed[i] = yes;
      }
   }
   
   if ((numLoopQs == 0) && (numConstraintQs == 0))
   {
      goto cleanup;
   }

   /* allocate space for list of gencoords in loops and constraints. */
   solveInfo->loopInfo->gencoord_list = (GeneralizedCoord**)simm_malloc(numLoopQs * sizeof(GeneralizedCoord*));
   solveInfo->constraintInfo->gencoord_list = (GeneralizedCoord**)simm_malloc(numConstraintQs * sizeof(GeneralizedCoord*));
   
   /* Make a list of all loop gencoords used. */
   index = 0;
   for (i = 0; i < ms->numclosedloops; i++)
   {
      if (solveInfo->loopInfo->loopUsed[i] == yes)
      {
         for (j = 0; j < ms->loop[i].num_qs; j++)
         {
            solveInfo->loopInfo->gencoord_list[index++] = ms->loop[i].qs[j];
         }
      }
   }
   /* Make a list of all constraint gencoords used. */
   index = 0;
   for (i = 0; i < ms->num_constraint_objects; i++)
   {
      if (solveInfo->constraintInfo->consUsed[i] == yes)
      {
         for (j = 0; j < ms->constraintobj[i].num_qs; j++)
         {
            solveInfo->constraintInfo->gencoord_list[index++] = ms->constraintobj[i].qs[j];
         }
      }
   }

   /* Rearrange the list of gencoords so each gencoord appears only once.
    * Locked gencoords are not included in the list.  
    * Update the number of Qs and residuals (add one residual for each Q).
    */   
   orderQs(ms, &numLoopQs, solveInfo->loopInfo->gencoord_list, NULL);
   orderQs(ms, &numConstraintQs, solveInfo->constraintInfo->gencoord_list, NULL);

   nq = numLoopQs + numConstraintQs;

   /* copy both gencoord lists into single list */
   solveInfo->gencoord_list = (GeneralizedCoord**)simm_malloc(nq * sizeof(GeneralizedCoord*));
   for (i = 0; i < numLoopQs; i++)
      solveInfo->gencoord_list[i] = solveInfo->loopInfo->gencoord_list[i];
   for (i = numLoopQs; i < nq; i++)
      solveInfo->gencoord_list[i] = solveInfo->constraintInfo->gencoord_list[i-numLoopQs];

   /* Rearrange the list of gencoords so each gencoord appears only once.
    * Locked gencoords are not included in the list.  
    * Update the number of Qs and residuals (add one residual for each Q).
    */   
   orderQs(ms, &nq, solveInfo->gencoord_list, NULL);
   solveInfo->numQs = nq;
//if nq or nres == 0???
   nres = numLoopResids + numConstraintResids + nq;

   /* reallocate space for arrays given new sizes */
   solveInfo->loopInfo->gencoord_list = (GeneralizedCoord**)simm_realloc(solveInfo->loopInfo->gencoord_list,
      numLoopQs * sizeof(GeneralizedCoord*), &rc);
   solveInfo->constraintInfo->gencoord_list = (GeneralizedCoord**)simm_realloc(solveInfo->constraintInfo->gencoord_list,
      numConstraintQs * sizeof(GeneralizedCoord*), &rc);
   solveInfo->gencoord_list = (GeneralizedCoord**)simm_realloc(solveInfo->gencoord_list, nq * sizeof(GeneralizedCoord*), &rc);

   /* allocate space for arrays now that sizes are known */
   q = (double *)simm_malloc(nq * sizeof(double));
   saved_q = (double *)simm_malloc(nq * sizeof(double));
   resid = (double *)simm_malloc(nres * sizeof(double));
   
   /* Copy gencoord values into q array, and save the original gencoord values. */
   for (i = 0; i < nq; i++)
      q[i] = saved_q[i] = solveInfo->gencoord_list[i]->value;

   solveInfo->numLoopQs = numLoopQs;
   solveInfo->numLoopResids = numLoopResids;
   solveInfo->numConstraintQs = numConstraintQs;
   solveInfo->numConstraintResids = numConstraintResids;

   /* solve for new qs using Levenberg-Marquard method of minimizing least squares. */
   if (!LeastSquaresSolver(solveInfo, nq, q, nres, resid))
   {
//      printf("lss problem\n");
   }
   evaluateLCSolution(solveInfo, nq, q, nres, resid);
   updateModel(solveInfo->model, solveInfo, nq, q, saved_q, nres, resid, &changed);

evaluate:
   calculateLCResids(nres, nq, q, resid, &i, solveInfo);
   evaluateLCSolution(solveInfo, nq, q, nres, resid);

cleanup:
   if (solveInfo)
   {
      if (solveInfo->loopClosed == no)
         *loopStatus = loopBroken;
      if (solveInfo->constraintClosed == no)
         *constraintStatus = constraintBroken;
      if (changed == yes)
      {
         *loopStatus = loopChanged;
         *constraintStatus = constraintChanged;
      }
      if (solveInfo->largeGenCoordResids == yes)
      {
         *loopStatus = largeResidinLoop;
         *constraintStatus = gcOutOfRange;
      }
      
      ms->loopsOK = solveInfo->loopClosed;
      ms->constraintsOK = solveInfo->constraintClosed;
      
      FREE_IFNOTNULL(solveInfo->loopInfo->gencoord_list);
      FREE_IFNOTNULL(solveInfo->loopInfo->loopUsed);
      FREE_IFNOTNULL(solveInfo->loopInfo);
      FREE_IFNOTNULL(solveInfo->constraintInfo->gencoord_list);
      FREE_IFNOTNULL(solveInfo->constraintInfo->consUsed);
      FREE_IFNOTNULL(solveInfo->constraintInfo);
      FREE_IFNOTNULL(solveInfo->gencoord_list);
      FREE_IFNOTNULL(solveInfo);
   }
   FREE_IFNOTNULL(q);
   FREE_IFNOTNULL(saved_q);
   FREE_IFNOTNULL(resid);
}

/* Solve any loops and/or constraints that are affected by the given gencoord */
//try to set to desired gc_value, if works OK, if doesn't work set to orig
SBoolean solveLCAffectedByGC(ModelStruct *ms, GeneralizedCoord* controlled_gc, double *gc_value)
{
   int i, j, index, nq, nres;
   int numLoopQs, numLoopResids, numConstraintQs, numConstraintResids;
   double orig_value;
   double *q = NULL, *saved_q = NULL, *resid = NULL;
   LCStruct *solveInfo;
   ReturnCode rc;
   SBoolean validSolution = yes;
    
   /* if there are no loops or constraints to solve */
   if ((!loopsToSolve(ms)) && (!constraintsToSolve(ms)))
      return validSolution;

   /* store the original value of the controlled gencoord */
   orig_value = controlled_gc->value;

   solveInfo = (LCStruct *)simm_malloc(sizeof(LCStruct));
   solveInfo->model = ms;
   solveInfo->first_iter = yes;
   solveInfo->largeGenCoordResids = no;
   solveInfo->loopClosed = yes;
   solveInfo->constraintClosed = yes;
   solveInfo->controlled_gc = controlled_gc;
   solveInfo->controlled_value = *gc_value;

   /* create structure to hold info about loops being solved */
   solveInfo->loopInfo = (IKStruct *)simm_malloc(sizeof(IKStruct));
   solveInfo->loopInfo->ms = ms;
   solveInfo->loopInfo->loopUsed = (SBoolean *)simm_malloc(ms->numclosedloops * sizeof(SBoolean));
   solveInfo->loopInfo->gencoord_list = NULL;
   solveInfo->loopInfo->controlled_gc = controlled_gc;
   solveInfo->loopInfo->controlled_value = *gc_value;
   solveInfo->loopInfo->first_iter = yes;

   /* create structure to hold info about constraints being solved */
   solveInfo->constraintInfo = (ConstraintSolverStruct *)simm_malloc(sizeof(ConstraintSolverStruct));
   solveInfo->constraintInfo->model = ms;
   solveInfo->constraintInfo->controlled_gc = controlled_gc;
   solveInfo->constraintInfo->controlled_value = *gc_value;
   solveInfo->constraintInfo->gencoord_list = NULL;
   solveInfo->constraintInfo->first_iter = yes;
   solveInfo->constraintInfo->test = no;
   solveInfo->constraintInfo->consUsed = (SBoolean *)simm_malloc(ms->num_constraint_objects * sizeof(SBoolean));

   nq = 0;
   nres = 0;
   numLoopQs = 0;
   numLoopResids = 0;
   numConstraintQs = 0;
   numConstraintResids = 0;

   /* mark all loops that are affected by a change in the controlled gencoord */
   markAffectedLoopsAndConstraints(solveInfo, controlled_gc);
   
   /* Calculate the total number of qs and residuals in loops. */ 
   for (i = 0; i < ms->numclosedloops; i++)
   {
      if (solveInfo->loopInfo->loopUsed[i])
      {
         numLoopQs += ms->loop[i].num_qs;
         numLoopResids += ms->loop[i].num_resids;
      }
   }  

   /* Calculate the total number of qs and residuals in constraints. */ 
   for (i = 0; i < ms->num_constraint_objects; i++)
   {
      if (solveInfo->constraintInfo->consUsed[i])
      {
         numConstraintQs += ms->constraintobj[i].num_qs;
         numConstraintResids += ms->constraintobj[i].numPoints;
      }
   }
   
   /* allocate space for list of gencoords in loops and constraints. */
   solveInfo->loopInfo->gencoord_list = (GeneralizedCoord**)simm_malloc(numLoopQs * sizeof(GeneralizedCoord*));
   solveInfo->constraintInfo->gencoord_list = (GeneralizedCoord**)simm_malloc(numConstraintQs * sizeof(GeneralizedCoord*));
   
   /* Make a list of all loop gencoords used. */
   index = 0;
   for (i = 0; i < ms->numclosedloops; i++)
   {
      if (solveInfo->loopInfo->loopUsed[i] == yes)
      {
         for (j = 0; j < ms->loop[i].num_qs; j++)
         {
            solveInfo->loopInfo->gencoord_list[index++] = ms->loop[i].qs[j];
         }
      }
   }

   /* Make a list of all constraint gencoords used. */
   index = 0;
   for (i = 0; i < ms->num_constraint_objects; i++)
   {
      if (solveInfo->constraintInfo->consUsed[i] == yes)
      {
         for (j = 0; j < ms->constraintobj[i].num_qs; j++)
         {
            solveInfo->constraintInfo->gencoord_list[index++] = ms->constraintobj[i].qs[j];
         }
      }
   }

   /* Rearrange the list of gencoords so each gencoord appears only once.
    * Locked gencoords and the controlled gencoord are not included in the
    * list.  Update the number of Qs and residuals (add one residual for each Q).
    */
   orderQs(ms, &numLoopQs, solveInfo->loopInfo->gencoord_list, controlled_gc);
   orderQs(ms, &numConstraintQs, solveInfo->constraintInfo->gencoord_list, controlled_gc);
   nq = numLoopQs + numConstraintQs;

   /* copy both gencoord lists into single list */
   solveInfo->gencoord_list = (GeneralizedCoord**)simm_malloc(nq * sizeof(GeneralizedCoord*));
   for (i = 0; i < numLoopQs; i++)
      solveInfo->gencoord_list[i] = solveInfo->loopInfo->gencoord_list[i];
   for (i = numLoopQs; i < nq; i++)
      solveInfo->gencoord_list[i] = solveInfo->constraintInfo->gencoord_list[i-numLoopQs];

   /* Rearrange the list of gencoords so each gencoord appears only once.
    * Locked gencoords are not included in the list.  
    * Update the number of Qs and residuals (add one residual for each Q).
    */   
   orderQs(ms, &nq, solveInfo->gencoord_list, NULL);
   solveInfo->numQs = nq;

   if (nq <= 0) //added dkb april 3, 2003
   {
      return validSolution;
   }

   nres = numLoopResids + numConstraintResids + nq;

   /* reallocate space for arrays given new sizes */
   solveInfo->loopInfo->gencoord_list = (GeneralizedCoord**)simm_realloc(solveInfo->loopInfo->gencoord_list,
      numLoopQs * sizeof(GeneralizedCoord*), &rc);
   solveInfo->constraintInfo->gencoord_list = (GeneralizedCoord**)simm_realloc(solveInfo->constraintInfo->gencoord_list,
      numConstraintQs * sizeof(GeneralizedCoord*), &rc);
   solveInfo->gencoord_list = (GeneralizedCoord**)simm_realloc(solveInfo->gencoord_list, nq * sizeof(GeneralizedCoord*), &rc);

   /* allocate space for arrays now that sizes are known */
   q = (double *)simm_malloc(nq * sizeof(double));
   saved_q = (double *)simm_malloc(nq * sizeof(double));
   resid = (double *)simm_malloc(nres * sizeof(double));
   
   /* Copy gencoord values into q array, and save the original gencoord values. */
   for (i = 0; i < nq; i++)
      q[i] = saved_q[i] = solveInfo->gencoord_list[i]->value;

   solveInfo->numLoopQs = numLoopQs;
   solveInfo->numLoopResids = numLoopResids;
   solveInfo->numConstraintQs = numConstraintQs;
   solveInfo->numConstraintResids = numConstraintResids;

   nres = numLoopResids + numConstraintResids + nq;

   /* Set the value of the controlled gencoord to its controlled value. */
   setGencoordValue2(solveInfo->model, solveInfo->controlled_gc, solveInfo->controlled_value);

   /* solve for new qs using Levenberg-Marquard method of minimizing least squares. */
   if (!LeastSquaresSolver(solveInfo, nq, q, nres, resid))
   {
//      printf("lss problem\n");
   }

   evaluateLCSolution(solveInfo, nq, q, nres, resid);

   if (numLoopQs > 0)
   {
      if (solveInfo->loopClosed == yes)
      {
         /* To ensure the display will update correctly, restore original gencoord values
          * in model structure.  When setting the gencoords based on the q values, all
          * the necessary joint matrices will be calculated.
          */
         for (i = 0; i < nq; i++)
         {
            solveInfo->gencoord_list[i]->value = saved_q[i];
         }
      }
      else
      {
         *gc_value = orig_value;
         for (i = 0; i < nq; i++)
            q[i] = saved_q[i];
      }
   }

   if (numConstraintQs > 0)
   {
      if (solveInfo->constraintClosed == yes)
      {
      /* To ensure the display will update correctly, restore original gencoord values
      * in model structure.  When setting the gencoords based on the q values, all
      * the necessary joint matrices will be calculated.
         */
         for (i = 0; i < nq; i++)
         {
            //        ms->gencoord[solveInfo->gencoord_list[i]].value = saved_q[i];
            set_gencoord_value(ms, solveInfo->gencoord_list[i], saved_q[i], no);
         }
         
      }
      else
      {
      /* Set the gencoord values for all gencoords used in constraints.  To prevent
         * recursion, set the flag so loops and constraints won't be resolved. */
         //      for (i = 0; i < nq; i++)
         //      {
         //        set_gencoord_value(ms, solveInfo->gencoord_list[i], saved_q[i], no);
         //     }
         //      valid = no;
         *gc_value = orig_value;
         for (i = 0; i < nq; i++)
            q[i] = saved_q[i];
      }
   }
   
   /* If the controlled q is clamped and the new solution is out of range,
    * don't reset any q's return with original values.
    */
   if ((controlled_gc->clamped == yes) && ((*gc_value < controlled_gc->range.start) || (*gc_value > controlled_gc->range.end)))
   {
      *gc_value = orig_value;
      validSolution = no;
   }
   else
   {
      /* Set the gencoord values for all gencoords used in loops.  To prevent
       * recursion, set the flag so loops won't be resolved. */
      for (i = 0; i < nq; i++)
      {
         set_gencoord_value(ms, solveInfo->gencoord_list[i], q[i], no);
      }
      validSolution = yes;
   }
//dkb may 15, 2003 - should these be set here?  we don't know about status
   //of other loops and constraints that weren't solved
   //ms->constraintsOK = validSolution;
   //ms->loopsOK = validSolution;
   FREE_IFNOTNULL(solveInfo->loopInfo->gencoord_list);
   FREE_IFNOTNULL(solveInfo->loopInfo->loopUsed);
   FREE_IFNOTNULL(solveInfo->loopInfo);
   FREE_IFNOTNULL(solveInfo->constraintInfo->gencoord_list);
   FREE_IFNOTNULL(solveInfo->constraintInfo);
   FREE_IFNOTNULL(solveInfo->gencoord_list);
   FREE_IFNOTNULL(solveInfo);
   FREE_IFNOTNULL(q);
   FREE_IFNOTNULL(saved_q);
   FREE_IFNOTNULL(resid);

   return validSolution;
}

SBoolean solveLCAffectedByJNT(ModelStruct *ms, int joint, LoopStatus *loopStatus, ConstraintStatus *constraintStatus)
{
   int i, j, index, nq, nres;
   int numLoopQs, numLoopResids, numConstraintQs, numConstraintResids;
   double *q = NULL, *saved_q = NULL, *resid = NULL;
   LCStruct *solveInfo;
   ReturnCode rc;
   SBoolean validSolution, changed;
   int controlled_gc;

   *loopStatus = loopUnchanged;
   *constraintStatus = constraintUnchanged;

   /* if there are no loops or constraints to solve */
   if ((!loopsToSolve(ms)) && (!constraintsToSolve(ms)))
      return yes;

   solveInfo = (LCStruct *)simm_malloc(sizeof(LCStruct));
   solveInfo->model = ms;
   solveInfo->first_iter = yes;
   solveInfo->largeGenCoordResids = no;
   solveInfo->loopClosed = yes;
   solveInfo->constraintClosed = yes;
   solveInfo->controlled_gc = NULL;
   solveInfo->controlled_value = 0.0;

   /* create structure to hold info about loops being solved */
   solveInfo->loopInfo = (IKStruct *)simm_malloc(sizeof(IKStruct));
   solveInfo->loopInfo->ms = ms;
   solveInfo->loopInfo->loopUsed = (SBoolean *)simm_malloc(ms->numclosedloops * sizeof(SBoolean));
   solveInfo->loopInfo->gencoord_list = NULL;
   solveInfo->loopInfo->controlled_gc = NULL;
   solveInfo->loopInfo->controlled_value = 0.0;
   solveInfo->loopInfo->first_iter = yes;

   /* create structure to hold info about constraints being solved */
   solveInfo->constraintInfo = (ConstraintSolverStruct *)simm_malloc(sizeof(ConstraintSolverStruct));
   solveInfo->constraintInfo->model = ms;
   solveInfo->constraintInfo->controlled_gc = NULL;
   solveInfo->constraintInfo->controlled_value = 0.0;
   solveInfo->constraintInfo->gencoord_list = NULL;
   solveInfo->constraintInfo->first_iter = yes;
   solveInfo->constraintInfo->test = no;
//   solveInfo->constraintInfo->tolerance = ms->constraint_tolerance;
   solveInfo->constraintInfo->consUsed = (SBoolean *)simm_malloc(ms->num_constraint_objects * sizeof(SBoolean));

   nq = 0;
   nres = 0;
   numLoopQs = 0;
   numLoopResids = 0;
   numConstraintQs = 0;
   numConstraintResids = 0;

    /* mark all loops that are affected by a change in the controlled gencoord */
   markLoopsAndConstraintsAffected(solveInfo, joint);
   
   /* Calculate the total number of qs and residuals in loops. */ 
   for (i = 0; i < ms->numclosedloops; i++)
   {
      if (solveInfo->loopInfo->loopUsed[i])
      {
         numLoopQs += ms->loop[i].num_qs;
         numLoopResids += ms->loop[i].num_resids;
      }
   }  

   /* Calculate the total number of qs and residuals in constraints. */ 
   for (i = 0; i < ms->num_constraint_objects; i++)
   {
      if (solveInfo->constraintInfo->consUsed[i])
      {
         numConstraintQs += ms->constraintobj[i].num_qs;
         numConstraintResids += ms->constraintobj[i].numPoints;
      }
   }
   
   /* allocate space for list of gencoords in loops and constraints. */
   solveInfo->loopInfo->gencoord_list = (GeneralizedCoord**)simm_malloc(numLoopQs * sizeof(GeneralizedCoord*));
   solveInfo->constraintInfo->gencoord_list = (GeneralizedCoord**)simm_malloc(numConstraintQs * sizeof(GeneralizedCoord*));
   
   /* Make a list of all loop gencoords used. */
   index = 0;
   for (i = 0; i < ms->numclosedloops; i++)
   {
      if (solveInfo->loopInfo->loopUsed[i] == yes)
      {
         for (j = 0; j < ms->loop[i].num_qs; j++)
         {
            solveInfo->loopInfo->gencoord_list[index++] = ms->loop[i].qs[j];
         }
      }
   }
   /* Make a list of all constraint gencoords used. */
   index = 0;
   for (i = 0; i < ms->num_constraint_objects; i++)
   {
      if (solveInfo->constraintInfo->consUsed[i] == yes)
      {
         for (j = 0; j < ms->constraintobj[i].num_qs; j++)
         {
            solveInfo->constraintInfo->gencoord_list[index++] = ms->constraintobj[i].qs[j];
         }
      }
   }

   /* Rearrange the list of gencoords so each gencoord appears only once.
    * Locked gencoords and the controlled gencoord are not included in the
    * list.  Update the number of Qs and residuals (add one residual for each Q).
    */
   orderQs(ms, &numLoopQs, solveInfo->loopInfo->gencoord_list, NULL);
   orderQs(ms, &numConstraintQs, solveInfo->constraintInfo->gencoord_list, NULL);
   nq = numLoopQs + numConstraintQs;

   /* copy both gencoord lists into single list */
   solveInfo->gencoord_list = (GeneralizedCoord**)simm_malloc(nq * sizeof(GeneralizedCoord*));
   for (i = 0; i < numLoopQs; i++)
      solveInfo->gencoord_list[i] = solveInfo->loopInfo->gencoord_list[i];
   for (i = numLoopQs; i < nq; i++)
      solveInfo->gencoord_list[i] = solveInfo->constraintInfo->gencoord_list[i-numLoopQs];

   /* Rearrange the list of gencoords so each gencoord appears only once.
    * Locked gencoords are not included in the list.  
    * Update the number of Qs and residuals (add one residual for each Q).
    */   
   orderQs(ms, &nq, solveInfo->gencoord_list, NULL);
   solveInfo->numQs = nq;

   if (nq <= 0) //added dkb april 3, 2003
   {
      FREE_IFNOTNULL(solveInfo->loopInfo->gencoord_list);
      FREE_IFNOTNULL(solveInfo->loopInfo->loopUsed);
      FREE_IFNOTNULL(solveInfo->loopInfo);
      FREE_IFNOTNULL(solveInfo->constraintInfo->gencoord_list);
      FREE_IFNOTNULL(solveInfo->constraintInfo);
      FREE_IFNOTNULL(solveInfo->gencoord_list);
      FREE_IFNOTNULL(solveInfo);
      FREE_IFNOTNULL(q);
      FREE_IFNOTNULL(saved_q);
      FREE_IFNOTNULL(resid);
      return yes;
   }

   nres = numLoopResids + numConstraintResids + nq;

   /* reallocate space for arrays given new sizes */
   solveInfo->loopInfo->gencoord_list = (GeneralizedCoord**)simm_realloc(solveInfo->loopInfo->gencoord_list,
      numLoopQs * sizeof(GeneralizedCoord*), &rc);
   solveInfo->constraintInfo->gencoord_list = (GeneralizedCoord**)simm_realloc(solveInfo->constraintInfo->gencoord_list,
      numConstraintQs * sizeof(GeneralizedCoord*), &rc);
   solveInfo->gencoord_list = (GeneralizedCoord**)simm_realloc(solveInfo->gencoord_list,
      nq * sizeof(GeneralizedCoord*), &rc);

   /* allocate space for arrays now that sizes are known */
   q = (double *)simm_malloc(nq * sizeof(double));
   saved_q = (double *)simm_malloc(nq * sizeof(double));
   resid = (double *)simm_malloc(nres * sizeof(double));
   
   /* Copy gencoord values into q array, and save the original gencoord values. */
   for (i = 0; i < nq; i++)
      q[i] = saved_q[i] = solveInfo->gencoord_list[i]->value;

   solveInfo->numLoopQs = numLoopQs;
   solveInfo->numLoopResids = numLoopResids;
   solveInfo->numConstraintQs = numConstraintQs;
   solveInfo->numConstraintResids = numConstraintResids;

   nres = numLoopResids + numConstraintResids + nq;

   /* solve for new qs using Levenberg-Marquard method of minimizing least squares. */
   if (!LeastSquaresSolver(solveInfo, nq, q, nres, resid))
   {
//      printf("lss problem\n");
   }
   evaluateLCSolution(solveInfo, nq, q, nres, resid);
   updateModel(solveInfo->model, solveInfo, nq, q, saved_q, nres, resid, &changed);
   
   if (solveInfo->loopClosed == no)
      *loopStatus = loopBroken;
   if (solveInfo->constraintClosed == no)
      *constraintStatus = constraintBroken;
   if (changed == yes)
   {
      if (solveInfo->loopClosed == yes)
         *loopStatus = loopChanged;
      if (solveInfo->constraintClosed == yes)
         *constraintStatus = constraintChanged;
   }
   if (solveInfo->largeGenCoordResids == yes)
   {
      *loopStatus = largeResidinLoop;
      *constraintStatus = gcOutOfRange;
   }
  
   if (solveInfo->loopClosed == yes && solveInfo->constraintClosed == yes)
      validSolution = yes;
   else
      validSolution = no;


   FREE_IFNOTNULL(solveInfo->loopInfo->gencoord_list);
   FREE_IFNOTNULL(solveInfo->loopInfo->loopUsed);
   FREE_IFNOTNULL(solveInfo->loopInfo);
   FREE_IFNOTNULL(solveInfo->constraintInfo->gencoord_list);
   FREE_IFNOTNULL(solveInfo->constraintInfo);
   FREE_IFNOTNULL(solveInfo->gencoord_list);
   FREE_IFNOTNULL(solveInfo);
   FREE_IFNOTNULL(q);
   FREE_IFNOTNULL(saved_q);
   FREE_IFNOTNULL(resid);

   return validSolution;
}

/* SOLVE_ALL_LOOPS_AND_CONSTRAINTS:
 * Solve all loops and constraints at once.  If enforce_constraints = yes,
 * solve constraints (if not, do not).  Return the status of loops and constraints
 */
void evaluateLoopsAndConstraintsInCurrentConfiguration(ModelStruct *ms, LoopStatus *loopStatus, 
                                 ConstraintStatus *constraintStatus, 
                                 SBoolean enforce_constraints)
{
   int i, j, nq, nres, index;
   int numLoopQs, numLoopResids, numConstraintQs, numConstraintResids;
   double *q = NULL, *saved_q = NULL, *resid = NULL;
   LCStruct *solveInfo = NULL;
   ReturnCode rc;
   SBoolean changed = no;

   if ((!loopsToSolve(ms)) && (!constraintsToSolve(ms)))
      goto cleanup;
   
   solveInfo = (LCStruct *)simm_malloc(sizeof(LCStruct));
   solveInfo->model = ms;
   solveInfo->first_iter = yes;
   solveInfo->largeGenCoordResids = no;
   solveInfo->loopClosed = yes;
   solveInfo->constraintClosed = yes;
   solveInfo->controlled_gc = NULL;
   solveInfo->controlled_value = 0.0;
   solveInfo->gencoord_list = NULL;

   /* create structure to hold info about loops being solved */
   solveInfo->loopInfo = (IKStruct *)simm_malloc(sizeof(IKStruct));
   solveInfo->loopInfo->ms = ms;
   solveInfo->loopInfo->loopUsed = (SBoolean *)simm_malloc(ms->numclosedloops * sizeof(SBoolean));
   solveInfo->loopInfo->gencoord_list = NULL;
   solveInfo->loopInfo->controlled_gc = NULL;
   solveInfo->loopInfo->controlled_value = 0.0;
   solveInfo->loopInfo->first_iter = yes;

   /* create structure to hold info about constraints being solved */
   solveInfo->constraintInfo = (ConstraintSolverStruct *)simm_malloc(sizeof(ConstraintSolverStruct));
   solveInfo->constraintInfo->model = ms;
   solveInfo->constraintInfo->controlled_gc = NULL;
   solveInfo->constraintInfo->controlled_value = 0.0;
   solveInfo->constraintInfo->gencoord_list = NULL;
   solveInfo->constraintInfo->first_iter = yes;
   solveInfo->constraintInfo->test = no;
//   solveInfo->constraintInfo->tolerance = ms->constraint_tolerance;
   solveInfo->constraintInfo->consUsed = (SBoolean *)simm_malloc(ms->num_constraint_objects * sizeof(SBoolean));

   nq = 0;
   nres = 0;
   numLoopQs = 0;
   numLoopResids = 0;
   numConstraintQs = 0;
   numConstraintResids = 0;

   /* Calculate the total number of qs and residuals in loops. */ 
   for (i = 0; i < ms->numclosedloops; i++)
   {
      numLoopQs += ms->loop[i].num_qs;
      numLoopResids += ms->loop[i].num_resids;
      solveInfo->loopInfo->loopUsed[i] = yes;
   }  

   /* Calculate the total number of qs and residuals in constraints. */ 
   for (i = 0; i < ms->num_constraint_objects; i++)
   {
      solveInfo->constraintInfo->consUsed[i] = no;
      if ((enforce_constraints == yes) && (ms->constraintobj[i].active == yes)
         && (ms->constraintobj[i].numPoints > 0))
      {
         numConstraintQs += ms->constraintobj[i].num_qs;
         numConstraintResids += ms->constraintobj[i].numPoints;
         solveInfo->constraintInfo->consUsed[i] = yes;
      }
   }
   
   if ((numLoopQs == 0) && (numConstraintQs == 0))
   {
      goto cleanup;
   }

   /* allocate space for list of gencoords in loops and constraints. */
   solveInfo->loopInfo->gencoord_list = (GeneralizedCoord**)simm_malloc(numLoopQs * sizeof(GeneralizedCoord*));
   solveInfo->constraintInfo->gencoord_list = (GeneralizedCoord**)simm_malloc(numConstraintQs * sizeof(GeneralizedCoord*));
   
   /* Make a list of all loop gencoords used. */
   index = 0;
   for (i = 0; i < ms->numclosedloops; i++)
   {
      if (solveInfo->loopInfo->loopUsed[i] == yes)
      {
         for (j = 0; j < ms->loop[i].num_qs; j++)
         {
            solveInfo->loopInfo->gencoord_list[index++] = ms->loop[i].qs[j];
         }
      }
   }
   /* Make a list of all constraint gencoords used. */
   index = 0;
   for (i = 0; i < ms->num_constraint_objects; i++)
   {
      if (solveInfo->constraintInfo->consUsed[i] == yes)
      {
         for (j = 0; j < ms->constraintobj[i].num_qs; j++)
         {
            solveInfo->constraintInfo->gencoord_list[index++] = ms->constraintobj[i].qs[j];
         }
      }
   }

   /* Rearrange the list of gencoords so each gencoord appears only once.
    * Locked gencoords are not included in the list.  
    * Update the number of Qs and residuals (add one residual for each Q).
    */   
   orderQs(ms, &numLoopQs, solveInfo->loopInfo->gencoord_list, NULL);
   orderQs(ms, &numConstraintQs, solveInfo->constraintInfo->gencoord_list, NULL);

   nq = numLoopQs + numConstraintQs;

   /* copy both gencoord lists into single list */
   solveInfo->gencoord_list = (GeneralizedCoord**)simm_malloc(nq * sizeof(GeneralizedCoord*));
   for (i = 0; i < numLoopQs; i++)
      solveInfo->gencoord_list[i] = solveInfo->loopInfo->gencoord_list[i];
   for (i = numLoopQs; i < nq; i++)
      solveInfo->gencoord_list[i] = solveInfo->constraintInfo->gencoord_list[i-numLoopQs];

   /* Rearrange the list of gencoords so each gencoord appears only once.
    * Locked gencoords are not included in the list.  
    * Update the number of Qs and residuals (add one residual for each Q).
    */   
   orderQs(ms, &nq, solveInfo->gencoord_list, NULL);
   solveInfo->numQs = nq;
//if nq or nres == 0???
   nres = numLoopResids + numConstraintResids + nq;

   /* reallocate space for arrays given new sizes */
   solveInfo->loopInfo->gencoord_list = (GeneralizedCoord**)simm_realloc(solveInfo->loopInfo->gencoord_list,
      numLoopQs * sizeof(GeneralizedCoord*), &rc);
   solveInfo->constraintInfo->gencoord_list = (GeneralizedCoord**)simm_realloc(solveInfo->constraintInfo->gencoord_list,
      numConstraintQs * sizeof(GeneralizedCoord*), &rc);
   solveInfo->gencoord_list = (GeneralizedCoord**)simm_realloc(solveInfo->gencoord_list, nq * sizeof(GeneralizedCoord*), &rc);

   /* allocate space for arrays now that sizes are known */
   q = (double *)simm_malloc(nq * sizeof(double));
   saved_q = (double *)simm_malloc(nq * sizeof(double));
   resid = (double *)simm_malloc(nres * sizeof(double));
   
   /* Copy gencoord values into q array, and save the original gencoord values. */
   for (i = 0; i < nq; i++)
      q[i] = saved_q[i] = solveInfo->gencoord_list[i]->value;

   solveInfo->numLoopQs = numLoopQs;
   solveInfo->numLoopResids = numLoopResids;
   solveInfo->numConstraintQs = numConstraintQs;
   solveInfo->numConstraintResids = numConstraintResids;

   calculateLCResids(nres, nq, q, resid, &i, solveInfo);
   evaluateLCSolution(solveInfo, nq, q, nres, resid);

cleanup:
   *loopStatus = loopUnchanged;
   *constraintStatus = constraintUnchanged;
   if (solveInfo)
   {
      if (solveInfo->loopClosed == no)
         *loopStatus = loopBroken;
      if (solveInfo->constraintClosed == no)
         *constraintStatus = constraintBroken;
      if (changed == yes)
      {
         *loopStatus = loopChanged;
         *constraintStatus = constraintChanged;
      }
      if (solveInfo->largeGenCoordResids == yes)
      {
         *loopStatus = largeResidinLoop;
         *constraintStatus = gcOutOfRange;
      }
      
      ms->loopsOK = solveInfo->loopClosed;
      ms->constraintsOK = solveInfo->constraintClosed;
      
      FREE_IFNOTNULL(solveInfo->loopInfo->gencoord_list);
      FREE_IFNOTNULL(solveInfo->loopInfo->loopUsed);
      FREE_IFNOTNULL(solveInfo->loopInfo);
      FREE_IFNOTNULL(solveInfo->constraintInfo->gencoord_list);
      FREE_IFNOTNULL(solveInfo->constraintInfo->consUsed);
      FREE_IFNOTNULL(solveInfo->constraintInfo);
      FREE_IFNOTNULL(solveInfo->gencoord_list);
      FREE_IFNOTNULL(solveInfo);
   }
   FREE_IFNOTNULL(q);
   FREE_IFNOTNULL(saved_q);
   FREE_IFNOTNULL(resid);
}


/* Set up the required work arrays and variables and call lmdif
 * to solve the least squares problem using the Levenberg-Marquart theory.
 * q: solution vector (initially contains estimate of soln in internal units) (x) 
 * fvec: functions evaluated at output q (final residuals)
 * num_resid: number of functions (m) (nres) 
 * nq: number of variables (n) (ndofinp)
 */
static int LeastSquaresSolver(LCStruct *solveInfo, int numQ, double q[], int numResid, 
                                  double resid[])
{   
   int info, callsToCalcResid, ldfjac = numResid;

   /* solution parameters */
   int mode = 1, nprint = 0, max_iter = 500;
   double ftol = 1e-4, xtol = 1e-4, gtol = 0.0;
   double epsfcn = 0.0, step_factor = 0.2;
   
   /* work arrays */
   int *ipvt;  
   double *diag, *qtf, *wa1, *wa2, *wa3, *wa4, *fjac;
      
   /* allocate space for necessary arrays */
   ipvt = (int *)simm_malloc(numQ * sizeof(int));
   diag = (double *)simm_malloc(numQ * sizeof(double));
   fjac = (double *)simm_malloc(numQ * numResid * sizeof(double));
   qtf = (double *)simm_malloc(numQ * sizeof(double));               /* wa[ndofinp + 1] */
   wa1 = (double *)simm_malloc(numQ * sizeof(double));               /* wa[2 * ndofinp + 1] */
   wa2 = (double *)simm_malloc(numQ * sizeof(double));               /* wa[3 * ndofinp + 1] */
   wa3 = (double *)simm_malloc(numQ * sizeof(double));               /* wa[4 * ndofinp + 1] */
   wa4 = (double *)simm_malloc(numResid * sizeof(double));           /* wa[5 * ndofinp + 1]*/

//   ftol = xtol = solveInfo->solver_accuracy;
//   max_iter = solveInfo->maxIter;
   /* if the number of qs is greater than the number of residuals,
    * LMDIF will not be able to find a solution and will return an error message
    */
   if (numResid < numQ)
   {
      (void)sprintf(errorbuffer, 
         "Least Squares Error: num residuals (%d) < num q (%d)\n", numResid, numQ);
      error(none, errorbuffer);
   }

   lmdif_C(calculateLCResids, numResid, numQ, q, resid,
       ftol, xtol, gtol, max_iter, epsfcn, diag, mode, step_factor,
       nprint, &info, &callsToCalcResid, fjac, ldfjac, ipvt, qtf,
       wa1, wa2, wa3, wa4, solveInfo);

/*   if (info == 0)
      printf("improper input parameters\n");
   if (info == 1)
      printf("actual and predicted relative reductions in ssq are at most ftol\n");
   if (info == 2)
      printf("relative error between two consecutive iterates is at most xtol.\n");
   if (info == 3)
      printf("conditions for info = 1 and info = 2 both hold.\n");
   if (info == 4)
      printf("the cosine of the angle between fvec and any column of the jacobian is at most gtol in absolute value.\n");
   if (info == 5)
      printf("max iter exceeded\n");
   if (info == 6)
      printf("ftol is too small. no further reduction in the sum of squares is possible.\n");
   if (info == 7)
      printf("xtol is too small. no further improvement in the approximate solution x is possible.\n");
   if (info == 8)
      printf("gtol is too small. fvec is orthogonal to the columns of the jacobian to machine precision.\n");
*/
   FREE_IFNOTNULL(ipvt);
   FREE_IFNOTNULL(diag);
   FREE_IFNOTNULL(fjac);
   FREE_IFNOTNULL(qtf);
   FREE_IFNOTNULL(wa1);
   FREE_IFNOTNULL(wa2);
   FREE_IFNOTNULL(wa3);
   FREE_IFNOTNULL(wa4);

   if (info == 0 || info == 5 || info== 6 || info == 7 || info == 8)
      return no;
   return yes;
}

/* calculate gencoord, loop and constraint residuals */
void calculateLCResids(int numResid, int numQ, double q[], double residuals[],
                       int *iflag, void *data)
{
   int startIndex, endIndex;
   double weight = 1.0;
   LCStruct *solveInfo = (LCStruct *)data;

   /* initialize residuals array */
   clear_vector(residuals, numResid);

   startIndex = 0;
   endIndex = numQ;
   calculateGenCoordResiduals(solveInfo, numQ, q, numResid, residuals, startIndex, endIndex, &weight, iflag);
   startIndex = endIndex;
   endIndex = startIndex + solveInfo->numLoopResids;
   calculateLoopResiduals(solveInfo, numQ, q, numResid, residuals, startIndex, endIndex, &weight, iflag);
   startIndex = endIndex;
   endIndex = startIndex + solveInfo->numConstraintResids;
   calculateConstraintResids(solveInfo, numQ, q, numResid, residuals, startIndex, endIndex, iflag);

   solveInfo->first_iter = no;
}

/* Determine whether loops and constraints are closed, and whether any
 * gencoords are out of range (have large residuals) */
static void evaluateLCSolution(LCStruct *info, int nq, double q[], int nres, double resid[])
{
   int i, j, index;
   double rms_total;
   double *loop_resid = NULL, *cons_resid = NULL, *q_resid = NULL;
   SBoolean good;

   loop_resid = (double *)simm_malloc(info->numLoopResids * sizeof(double));
   cons_resid = (double *)simm_malloc(info->numConstraintResids * sizeof(double));
   q_resid = (double *)simm_malloc(nq * sizeof(double));

   for (i = 0; i < nq; i++)
      q_resid[i] = resid[i];

   for (i = nq, j = 0; i < nq + info->numLoopResids; i++)
      loop_resid[j++] = resid[i];

   for (i = nq + info->numLoopResids, j = 0; i < nres; i++)
      cons_resid[j++] = resid[i];

   /* evaluate the loop solution */
   /* calculate the total rms from the 'joint' residuals (not the Q residuals).
    * If the total rms is above the threshold, the solution is not valid because
    * the loop will break apart.
    */
   info->loopClosed = yes;
   rms_total = calculateRMS(info->numLoopResids, loop_resid);
   if (rms_total > info->loopInfo->ms->loop_tolerance)
      info->loopClosed = no;
   
   /* evaluate the constraint solution */
   index = 0;
   good = yes;
   for (i = 0; i < info->model->num_constraint_objects; i++)
   {
      if (info->model->constraintobj[i].active == no)
         continue;
      if (info->constraintInfo->consUsed[i] == no)
         continue;

      for (j = 0; j < info->model->constraintobj[i].numPoints; j++)
      {
         info->model->constraintobj[i].points[j].broken = no;
         if (fabs(cons_resid[index++]) > info->model->constraintobj[i].points[j].tolerance)
         {
            info->model->constraintobj[i].points[j].broken = yes;
            good = no;
         }
      }
   }

   info->constraintClosed = yes;
   if (!good)
      info->constraintClosed = no;

   info->largeGenCoordResids = no;
   /* if any of the gencoord residuals are too high, the gencoord is out of range */
   for (i = 0; i < nq; i++)
      if (resid[i] >= GENCOORD_TOLERANCE)
         info->largeGenCoordResids = yes;
   
   FREE_IFNOTNULL(loop_resid);
   FREE_IFNOTNULL(cons_resid);
   FREE_IFNOTNULL(q_resid);

}

static void updateModel(ModelStruct *ms, LCStruct *solveInfo, int nq, double q[], 
                 double saved_q[], int nres, double resid[], SBoolean *changed)
{
   int i;
   GeneralizedCoord* gc;

   /* determine whether any gencoords have changed */
   *changed = no;
   for (i = 0; i < nq; i++)
   {
      gc = solveInfo->gencoord_list[i];
      if (DABS(saved_q[i] - q[i]) > GENCOORD_TOLERANCE) // ms->gencoord[gc]->tolerance)
      {
         *changed = yes;
         break;
      }
   }

   /* if there are no changes, set the gencoords back to their original values */
   if (!(*changed))
   {
      for (i = 0; i < nq; i++)
      {
         gc = solveInfo->gencoord_list[i];
         set_gencoord_value(ms, gc, saved_q[i], no);
      }
      return;
   }

   /* update the values in the model viewer.  Reset the original values so
    * the changes appear in the model viewer. */
   if ((solveInfo->loopClosed == no) || (solveInfo->constraintClosed == no))
   {
      /* If no solution was found, reset the gencoords to their original values. */
      for (i = 0; i < nq; i++)
      {
         q[i] = saved_q[i];
         *changed = no;
      }
   }
   else
   {
      for (i = 0; i < nq; i++)
      {
         solveInfo->gencoord_list[i]->value = saved_q[i];
      }
   }
   /* set gencoords to new values.  Set the flag so loops and constraints are 
    * not solved to prevent recursion. */
   for (i = 0; i < nq; i++)
   {
      gc = solveInfo->gencoord_list[i];
      set_gencoord_value(ms, gc, q[i], no);
   }
}

#if ! ENGINE
static void displayLCMessages(ModelStruct *ms, LoopStatus loopStatus, ConstraintStatus constraintStatus)
{
   int i, ans;
   char gc_info[255];

   if ((loopStatus == loopChanged) || (constraintStatus == constraintChanged))
   {
      approveNewDefaultGCs(ms);
   }
   else if ((loopStatus == loopBroken) || (constraintStatus == constraintBroken))
   {
      glutMessageBox(GLUT_OK, 1, GLUT_ICON_INFORMATION,
         "LOAD MODEL: Constraints and/or Loops not satisfied (closed)", badLoopErrorMsg);
      ms->defaultGCApproved = no;
      ms->defaultConstraintsOK = no;
      ms->constraintsOK = no;
   }
   else if ((loopStatus == largeResidinLoop) || (constraintStatus == gcOutOfRange))
   {
      glutMessageBox(GLUT_OK, 1, GLUT_ICON_INFORMATION,
         "LOAD MODEL: Gencoord Out of Range", gencoordResidualErrorMsg);
      ms->GEFuncOK = yes;
      
      /* store new values as defaults (even though loops not closed) */
      for (i = 0; i < ms->numgencoords; i++)
         ms->gencoord[i]->default_value = ms->gencoord[i]->value;
      ms->defaultGCApproved = yes;
      ms->defaultLoopsOK = no;
      ms->constraintsOK = no;
      sprintf(buffer, "Load Model: Default values do not close loops.\n");
      error(none, buffer);
   }
}

#endif

/* mark all loops that are affected by a change in the controlled q */
static void markAffectedLoopsAndConstraints(LCStruct *info, GeneralizedCoord* gencoord)
{
   int i, j, k, numLoopsUsed = 0, numConsUsed = 0;
   SBoolean changed;

   for (i = 0; i < info->model->numclosedloops; i++)
      info->loopInfo->loopUsed[i] = no;

   for (i = 0; i < info->model->num_constraint_objects; i++)
      info->constraintInfo->consUsed[i] = no;

   /* find all loops that use the 'locked' gencoord */
   for (i = 0; i < info->model->numclosedloops; i++)
   {
      if ((GCUsedInLoop(gencoord, info->model->loop[i]) == yes) && (info->model->useIK == yes))
      {
         info->loopInfo->loopUsed[i] = yes;
         numLoopsUsed++;
      }
   }

   /* find all constraints that use the 'locked' gencoord */
   for (i = 0; i < info->model->num_constraint_objects; i++)
   {
      if (GCUsedInConstraint(gencoord, info->model->constraintobj[i]) == yes)
      {
         info->constraintInfo->consUsed[i] = yes;
         numConsUsed++;
      }
   }
   if ((numLoopsUsed == info->model->numclosedloops) && (numConsUsed == info->model->num_constraint_objects))
      return;

   /* find all loops that are affected by a change in the 'locked' q
    * A change in the locked q will cause changes in all the qs in that
    * loop.  These qs may be part of other loops which will in turn be
    * affected.  The q's in these affected loops may in turn affect
    * other loops.
    * Go through all qs in all loops until all affected loops have been
    * found.  Check all gencoords in each used loop. to see if they belong
    * to other loops.  If they are, mark those other loops if they haven't
    * been marked already. Keep doing this until no changes are made.
    */
   while (1)
   {
      changed = no;
      /* see if gencoords are used in any other loops */
      for (i = 0; i < info->model->numclosedloops; i++)
      {
         if ((info->loopInfo->loopUsed[i] == yes) && (info->model->useIK == yes))
         {
            for (j = 0; j < info->model->loop[i].num_qs; j++)
            {
               for (k = 0; k < info->model->numclosedloops; k++)
               {
                  if (GCUsedInLoop(info->model->loop[i].qs[j], info->model->loop[k]) == yes)
                  {
                     if (info->loopInfo->loopUsed[k] == no)
                     {
                        info->loopInfo->loopUsed[k] = yes;
                        changed = yes;
                     }
                  }
               }
            }
         }
      }
      for (i = 0; i < info->model->numclosedloops; i++)
      {
         if ((info->loopInfo->loopUsed[i] == yes) && (info->model->useIK == yes))
         {
            for (j = 0; j < info->model->loop[i].num_qs; j++)
            {
               for (k = 0; k < info->model->num_constraint_objects; k++)
               {
                  if (GCUsedInConstraint(info->model->loop[i].qs[j], info->model->constraintobj[k]) == yes)
                  {
                     if (info->constraintInfo->consUsed[k] == no)
                     {
                        info->constraintInfo->consUsed[k] = yes;
                        changed = yes;
                     }
                  }
               }
            }
         }
      }

      
      /* see if gencoords are used in any other constraints */
      for (i = 0; i < info->model->num_constraint_objects; i++)
      {
         if (info->constraintInfo->consUsed[i] == yes)
         {
            for (j = 0; j < info->model->constraintobj[i].num_qs; j++)
            {
               for (k = 0; k < info->model->num_constraint_objects; k++)
               {
                  if (GCUsedInConstraint(info->model->constraintobj[i].qs[j], info->model->constraintobj[k]) == yes)
                  {
                     if (info->constraintInfo->consUsed[k] == no)
                     {
                        info->constraintInfo->consUsed[k] = yes;
                        changed = yes;
                     }
                  }
               }
            }
         }
      }
      /* see if gencoords are used in any other loops */
      for (i = 0; i < info->model->num_constraint_objects; i++)
      {
         if (info->constraintInfo->consUsed[i] == yes)
         {
            for (j = 0; j < info->model->constraintobj[i].num_qs; j++)
            {
               for (k = 0; k < info->model->numclosedloops; k++)
               {
                  if (GCUsedInLoop(info->model->constraintobj[i].qs[j], info->model->loop[k]) == yes)
                  {
                     if (info->model->useIK == yes)
                     {
                        if (info->loopInfo->loopUsed[k] == no)
                        {
                           info->loopInfo->loopUsed[k] = yes;
                           changed = yes;
                        }
                     }
                  }
               }
            }
         }
      }
      if (changed == no)
         break;
   }
}

/* mark all loops that are affected by a change in the controlled q */
static void markLoopsAndConstraintsAffected(LCStruct *info, int jnt)
{
   int i, j, k, numLoopsUsed = 0, numConsUsed = 0, jnt_index;
   SBoolean changed;

   for (i = 0; i < info->model->numclosedloops; i++)
      info->loopInfo->loopUsed[i] = no;

   for (i = 0; i < info->model->num_constraint_objects; i++)
      info->constraintInfo->consUsed[i] = no;

   /* find all loops that use the 'locked' gencoord */
   for (i = 0; i < info->model->numclosedloops; i++)
   {
      if ((JNTUsedInLoop(jnt, info->model->loop[i])) && (info->model->useIK == yes))
      {
         info->loopInfo->loopUsed[i] = yes;
         numLoopsUsed++;
      }
   }
   /* find all constraints that use the 'locked' gencoord */
   for (i = 0; i < info->model->num_constraint_objects; i++)
   {
      if (JNTUsedInConstraint(jnt, info->model->constraintobj[i]))
      {
         info->constraintInfo->consUsed[i] = yes;
         numConsUsed++;
      }
   }
   if ((numLoopsUsed == info->model->numclosedloops) && (numConsUsed == info->model->num_constraint_objects))
      return;

   /* find all loops that are affected by a change in the 'locked' q
    * A change in the locked q will cause changes in all the qs in that
    * loop.  These qs may be part of other loops which will in turn be
    * affected.  The q's in these affected loops may in turn affect
    * other loops.
    * Go through all qs in all loops until all affected loops have been
    * found.  Check all gencoords in each used loop. to see if they belong
    * to other loops.  If they are, mark those other loops if they haven't
    * been marked already. Keep doing this until no changes are made.
    */
   while (1)
   {
      changed = no;
      /* see if gencoords are used in any other loops */
      for (i = 0; i < info->model->numclosedloops; i++)
      {
         if ((info->loopInfo->loopUsed[i] == yes) && (info->model->useIK == yes))
         {
            for (j = 0; j < info->model->loop[i].num_jnts; j++)
            {
               for (k = 0; k < info->model->numclosedloops; k++)
               {
                  jnt_index = fabs(info->model->loop[i].joints[j]) - 1;
                  if (JNTUsedInLoop(jnt_index, info->model->loop[k]) == yes)
                  {
                     if (info->loopInfo->loopUsed[k] == no)
                     {
                        info->loopInfo->loopUsed[k] = yes;
                        changed = yes;
                     }
                  }
               }
            }
         }
      }
      for (i = 0; i < info->model->numclosedloops; i++)
      {
         if ((info->loopInfo->loopUsed[i] == yes) && (info->model->useIK == yes))
         {
            for (j = 0; j < info->model->loop[i].num_jnts; j++)
            {
               jnt_index = fabs(info->model->loop[i].joints[j]) - 1;
               for (k = 0; k < info->model->num_constraint_objects; k++)
               {
                  if (JNTUsedInConstraint(jnt_index, info->model->constraintobj[k]) == yes)
                  {
                     if (info->constraintInfo->consUsed[k] == no)
                     {
                        info->constraintInfo->consUsed[k] = yes;
                        changed = yes;
                     }
                  }
               }
            }
         }
      }

      
      /* see if gencoords are used in any other constraints */
      for (i = 0; i < info->model->num_constraint_objects; i++)
      {
         if (info->constraintInfo->consUsed[i] == yes)
         {
            for (j = 0; j < info->model->constraintobj[i].num_jnts; j++)
            {
               for (k = 0; k < info->model->num_constraint_objects; k++)
               {
                  if (JNTUsedInConstraint(info->model->constraintobj[i].joints[j], info->model->constraintobj[k]) == yes)
                  {
                     if (info->constraintInfo->consUsed[k] == no)
                     {
                        info->constraintInfo->consUsed[k] = yes;
                        changed = yes;
                     }
                  }
               }
            }
         }
      }
      /* see if gencoords are used in any other loops */
      for (i = 0; i < info->model->num_constraint_objects; i++)
      {
         if (info->constraintInfo->consUsed[i] == yes)
         {
            for (j = 0; j < info->model->constraintobj[i].num_jnts; j++)
            {
               for (k = 0; k < info->model->numclosedloops; k++)
               {
                  if (JNTUsedInLoop(info->model->constraintobj[i].joints[j], info->model->loop[k]) == yes)
                  {
                     if ((info->loopInfo->loopUsed[k] == no) && (info->model->useIK == yes))
                     {
                        info->loopInfo->loopUsed[k] = yes;
                        changed = yes;
                     }
                  }
               }
            }
         }
      }
      if (changed == no)
         break;
   }
}


/* CALCULATEGENCOORDRESIDUALS:
 * Calculate the residuals for all the gencoords solved (in the q array).
 * Set the gencoords to the values in the q array and invalidate the joint
 * matrices if the values have changed.
 * If a gencoord is clamped and out of range, increase the weight, and set
 * the residual.
 */
static void calculateGenCoordResiduals(void *data, int numQ, double q[], int numResid,
                                       double resid[], int startIndex, int endIndex,
                                       double *weight, int *iflag)
{
   int i, index, gc_error;
   double rangeError, toMax, toMin, rangeStart, rangeEnd;
   LCStruct *solveInfo = (LCStruct *)data;
   GeneralizedCoord* gc = NULL;

   *weight = 1.0;
   index = startIndex;
//if (!JACOBIAN) printf("GC resids\n");

   /* Update the gencoord records in the model structure to the new values stored in
    * the q array (using setGencoordValue2).  Set the values and invalidate the transform matrices so that these
    * will be recalculated when necessary.  If a clamped gencoord is out of range, 
    * increase the weight so the loop residuals will be increased.
    */
   for (i = 0; i < numQ; i++)
   {
      if (index > endIndex)
      {
         *iflag = 10;
         break;
      }
      gc = solveInfo->gencoord_list[i];
      gc_error = setGencoordValue2(solveInfo->model, gc, q[i]);
      rangeStart = gc->range.start;
      rangeEnd = gc->range.end;
      toMax = rangeEnd - q[i];
      toMin = q[i] - rangeStart;
      
      if (gc->clamped == yes)
      {
         if (gc->type == rotation_gencoord)
         {
            if (q[i] > rangeEnd)
            {
               rangeError = q[i] - rangeEnd + CLAMPING_TOLERANCE;
            }
            else if (q[i] < rangeStart)
            {
               rangeError = rangeStart - q[i] + CLAMPING_TOLERANCE;
            }
            else 
            {
               if ((toMax > CLAMPING_TOLERANCE) && (fabs(toMin) > CLAMPING_TOLERANCE))
                  rangeError = 0.0;
               else if (toMax < toMin)
                  rangeError = CLAMPING_TOLERANCE - toMax;
               else
                  rangeError = -1 * (CLAMPING_TOLERANCE - toMin);
            }
            rangeError *= 0.01;
            *weight += rangeError/100.0;
            resid[index] = SQR(rangeError)/1000.0;
         }
         else
         {
            if (q[i] > rangeEnd)
            {
               rangeError = q[i] - rangeEnd + CLAMPING_TOL_TRANS;
            }
            else if (q[i] < rangeStart)
            {
               rangeError = rangeStart - q[i] + CLAMPING_TOL_TRANS;
            }
            else 
            {
               if ((toMax >= CLAMPING_TOL_TRANS) && 
                  (fabs(toMin) >= CLAMPING_TOL_TRANS))
                  rangeError = 0.0;
               else if (toMax < toMin)
                  rangeError = CLAMPING_TOL_TRANS - toMax;
               else
                  rangeError = -1 * (CLAMPING_TOL_TRANS - toMin);
            }
            *weight += rangeError;
            resid[index] = rangeError;
         }
      }
      else
      {
         /* if the gencoord is unclamped and has an active function,
          * use the restraint function to calculate the weight and
          * residual */
         if (gc->restraint_function && gc->restraintFuncActive == yes)
         {
            rangeError = fabs(interpolate_function(q[i], gc->restraint_function, zeroth, 0.0, 0.0));
            rangeError *= 0.1;
            *weight += rangeError / 100.0;
            resid[index] = rangeError / 1000.0;
         }
         else
         {
#if 0 // JPL 01/23/06: if gc is unclamped and no restraint function, residual should be 0.0, right?
            /* original method */
            if (q[i] < rangeStart)
               rangeError = (rangeStart - q[i]);
            else if (q[i] > rangeEnd)
               rangeError = (q[i] - rangeEnd);
            else
               rangeError = 0.0;            
            resid[index] = rangeError;
#endif
                resid[index] = 0.0;
         }  //added dkb apr 2003
      }
//if (!JACOBIAN)   printf("resid[%d] (gc%s) = %f\n", index, gc->name, resid[index]);
      index++;
   }
   solveInfo->first_iter = no;
}


/* CALCULATELOOPRESIDUALS:
 * Calculate the residuals for the loops solved. Assume the gencoords have been
 * set to the new values and the joint matrices have been invalidated.
 * The loop residuals are calculated by breaking each joint in the loop and
 * determining the coordinates of the point in the ground frame using two paths
 * from the "first" segment to the joint.  The coordinates of the points are
 * then subtracted and scaled by the given weight to find the residual.
 */
static void calculateLoopResiduals(void *data, int numQ, double q[], int numResid, double resid[], 
                            int startIndex, int endIndex, double *weight, int *iflag)
{
   int i, j, loop, seg, index;
   int *pathA= NULL, *pathB = NULL, pathLengthA, pathLengthB;
   double coordsA[3], coordsB[3];
   LCStruct *solveInfo = (LCStruct *)data;
//if (!JACOBIAN) printf("loop resid\n");
   index = startIndex;
   for (loop = 0; loop < solveInfo->model->numclosedloops; loop++)
   {
      if (index > endIndex)
      {
//         printf("problem\n");
         break;
      }
      if (solveInfo->loopInfo->loopUsed[loop] == yes)
      {
         /* allocate paths */
         pathA = (int *)simm_malloc(solveInfo->model->loop[loop].num_jnts * sizeof(int));
         pathB = (int *)simm_malloc(solveInfo->model->loop[loop].num_jnts * sizeof(int));
         
         /* calculate residuals 
         * Find residuals for each joint in the loop, as though that joint were the
         * loop joint.  For each joint, "break" the loop and determine forward and inverse
         * paths from each segment to the "first" segment in the loop.  Transform the coords
         * of the two points to the "first" segment frame then to the ground frame.
         * The residual is the difference in the coordinates multiplied by the weight factor.
         */
         for (seg = 1; seg < solveInfo->model->loop[loop].num_segs - 1; seg++)
         {
            /* initially, coordinates in each frame are 0.0 0.0 0.0 */
            clear_vector(coordsA, 3);
            clear_vector(coordsB, 3);
            
            /* create paths going both ways from the segment to the "first" segment */
            pathLengthA = getLoopPath(seg, pathA, INVERSE, &solveInfo->model->loop[loop]);
            pathLengthB = getLoopPath(seg, pathB, FORWARD, &solveInfo->model->loop[loop]);
            
            /* convert point to "first" segment in loop via INVERSE path, then to ground */
            convertNEW(solveInfo->model, coordsA, pathA, pathLengthA);
            convert(solveInfo->model, coordsA, solveInfo->model->loop[loop].segs[0], 
               solveInfo->model->ground_segment);
            
            /* convert point to "first" segment in loop via FORWARD path, then to ground */
            convertNEW(solveInfo->model, coordsB, pathB, pathLengthB);
            convert(solveInfo->model, coordsB, solveInfo->model->loop[loop].segs[0], 
               solveInfo->model->ground_segment);
            
            for (i = 0; i < 3; i++)
            {
               resid[index] = (coordsA[i] - coordsB[i]) * *weight;
               index++;
            }
         }
         FREE_IFNOTNULL(pathA);
         FREE_IFNOTNULL(pathB);
      }
   }
   solveInfo->first_iter = no;

}

/* Reorganize the gencoord list so that a gencoord index appears only once.  Do not
 * include the controlled gencoord or any locked gencoords in the list.  Update the 
 * number of qs needed (number of gencoords in the new list)
 */
static void orderQs(ModelStruct *ms, int *nq, GeneralizedCoord* gc_list[], GeneralizedCoord* controlled_gc)
{
   SBoolean *gc_used = NULL;
   int i;

   gc_used = (SBoolean *)simm_malloc(ms->numgencoords * sizeof(SBoolean));

   for (i = 0; i < ms->numgencoords; i++)
      gc_used[i] = no;

   /* mark which gencoords are used in the affected loops */
   for (i = 0; i < *nq; i++)
      gc_used[getGencoordIndex(ms, gc_list[i])] = yes;

   if (controlled_gc)
      gc_used[getGencoordIndex(ms, controlled_gc)] = no;
   
   for (i = 0; i < ms->numgencoords; i++)
      if (ms->gencoord[i]->locked == yes)
         gc_used[i] = no;

   /* create a new q array with each gencoord appearing only once, and count the number
    * of qs used. */
   *nq = 0;
   for (i = 0; i < ms->numgencoords; i++)
   {
      if (gc_used[i] == yes)
         gc_list[(*nq)++] = ms->gencoord[i];
   }

   FREE_IFNOTNULL(gc_used);
}

/* calculate RMS values for loop joint residuals as well as total rms */
static double calculateRMS(int n, double fvec[])
{
   int i, j;
   double sum, *rms = NULL, temp, rms_all;

   sum = 0.0;
   rms = (double *)simm_malloc(n * sizeof(double));
   for (i = 0; i < n/3; i++)
   {
      temp = 0.0;
      for (j = 0; j < 3; j++)
         temp += SQR(fvec[3*i + j]);
      rms[i] = sqrt(temp/3);
      sum += temp;
   }
   rms_all = sqrt(sum/n);

   FREE_IFNOTNULL(rms);
   return rms_all;
}


/* determine whether the given gencoord is used in the given constraint object */
static SBoolean GCUsedInConstraint(GeneralizedCoord* gencoord, ConstraintObject co)
{
   int i;

   for (i = 0; i < co.num_qs; i++)
   {
      if (gencoord == co.qs[i])
         return yes;
   }
   return no;
}

/* determine whether the given joint is used in the given constraint object */
static SBoolean JNTUsedInConstraint(int jnt, ConstraintObject co)
{
   int i;

   for (i = 0; i < co.num_jnts; i++)
   {
      if (jnt == co.joints[i])
         return yes;
   }
   return no;
}

static SBoolean GCUsedInLoop(GeneralizedCoord* gencoord, LoopStruct loop)
{
   int i;

   for (i = 0; i < loop.num_qs; i++)
   {
      if (gencoord == loop.qs[i])
         return yes;
   }
   return no;
}

static SBoolean JNTUsedInLoop(int jnt, LoopStruct loop)
{
   int i;

   for (i = 0; i < loop.num_jnts; i++)
   {
      if (jnt == fabs(loop.joints[i]) - 1)
         return(yes);
   }
   return (no);
}


SBoolean loopsToSolve(ModelStruct *model)
{
   if ((model->numclosedloops > 0) && (model->useIK == yes))
      return yes;
   return no;
}

SBoolean constraintsToSolve(ModelStruct *model)
{
   int i;

   for (i = 0; i < model->num_constraint_objects; i++)
      if ((model->constraintobj[i].active == yes) && (model->constraintobj[i].numPoints > 0))
         return yes;
   return no;
}

void approveNewDefaultGCs(ModelStruct *ms)
{
   int i, ans;
   char gc_info[255];

#if ! ENGINE
   /* in all cases, defaults have been approved (whether or not they're accepted) */
   ms->defaultGCApproved = yes;
   sprintf(buffer, 
         "The set of default values for the gencoords did not\n"
         "satisfy the constraints and/or loops.  SIMM calculated\n"
         "a new solution closing all loops/satisfying all constraints.\n"
         "Do you want to accept the new default values?\n\n");
   for (i = 0; i < ms->numgencoords; i++)
   {
      if ((ms->gencoord[i]->used_in_model == yes)
         && (NOT_EQUAL_WITHIN_ERROR(ms->gencoord[i]->value,
         ms->gencoord[i]->default_value)))            
      {
         sprintf(gc_info, "   %-20s: % 10.5f -> % 10.5f\n", 
            ms->gencoord[i]->name, ms->gencoord[i]->default_value,
            ms->gencoord[i]->value);
         strcat(buffer, gc_info);
      }
   }
   strcat(buffer, "\nClick \"YES\" to display and save new values.\n\n");
   strcat(buffer, "Click \"NO\" to display new values without saving\n"
      "them as default values\n\n");
   strcat(buffer, "Click \"Cancel\" to ignore new values.");

   ans = glutMessageBox(GLUT_YES_NO_CANCEL, 1, GLUT_ICON_INFORMATION,
      "LOAD MODEL: Default Configuration", buffer);
   if (ans == GLUT_MSG_YES)
   {
      /* accept new default values and display them*/
      for (i = 0; i < ms->numgencoords; i++)
      {
         ms->gencoord[i]->default_value = ms->gencoord[i]->value;
         ms->gencoord[i]->default_value_specified = yes;
      }
      ms->defaultLoopsOK = yes;
      ms->defaultConstraintsOK = yes;
      sprintf(buffer, "New default gencoord values saved for %s.\n", ms->name);
      message(buffer, 0, DEFAULT_MESSAGE_X_OFFSET);
   }
   else if (ans == GLUT_MSG_CANCEL)
   {
      LoopStatus loopStatus;
      ConstraintStatus constraintStatus;

      /* keep and display original default values.  Set values without solving 
       * loops and constraints */
      //evaluate current situation to set values
      ms->defaultLoopsOK = no;
      ms->defaultConstraintsOK = no;
      ms->loopsOK = no;
      ms->constraintsOK = no;
      for (i = 0; i < ms->numgencoords; i++)
         set_gencoord_value(ms, ms->gencoord[i], ms->gencoord[i]->default_value, no);
      evaluateLoopsAndConstraintsInCurrentConfiguration(ms, &loopStatus, &constraintStatus,
         yes);
      if (!ms->defaultLoopsOK || !ms->defaultConstraintsOK)
      {
         sprintf(buffer, "Default values may not close loops or satisfy constraints.\n");
         error(none, buffer);
      }
   }
   else
   {
      /* display new values, but do not change defaults */
      ms->defaultLoopsOK = no;
      ms->defaultConstraintsOK = no;
      sprintf(buffer, "Default values may not close loops or satisfy constraints.\n");
      error(none, buffer);
   }
#else
   for (i = 0; i < ms->numgencoords; i++)
   {
      ms->gencoord[i]->default_value = ms->gencoord[i]->value;
      ms->gencoord[i]->default_value_specified = yes;
   }
   ms->defaultLoopsOK = yes;
   ms->defaultConstraintsOK = yes;
#endif
}
