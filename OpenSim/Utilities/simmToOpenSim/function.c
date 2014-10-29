/*******************************************************************************

   FUNCTION.C

   Author: Peter Loan

   Date: 5-NOV-2009

   Copyright (c) 2009 MusculoGraphics, Inc.
   All rights reserved.

   Description:

   Routines:

*******************************************************************************/

#include "universal.h"
#include "globals.h"
#include "functions.h"
#include "gcvspl.h"


/*************** DEFINES (for this file only) *********************************/


/*************** STATIC GLOBAL VARIABLES (for this file only) *****************/


/*************** GLOBAL VARIABLES (used in only a few files) ******************/


/*************** EXTERNED VARIABLES (declared in another file) ****************/


/*************** PROTOTYPES for STATIC FUNCTIONS (for this file only) *********/
static void calc_natural_cubic_coefficients(dpFunction* f);


ReturnCode read_function(ModelStruct* ms, FILE* fp, SBoolean muscle_function, dpFunctionType funcType, const char endTag[])
{
   dpFunction func;
   char fname[CHARBUFFER];

   func.type = funcType;
   func.cutoff_frequency = 0.0;
   func.source = (muscle_function == yes) ? dpMuscleFile : dpJointFile;

   // Malloc space for the points and coefficients of this function.
   (void)malloc_function(&func, FUNCTION_ARRAY_INCREMENT);

   if (read_string(fp, buffer) == EOF)
   {
      (void)sprintf(errorbuffer, "Unexpected EOF found reading a function");
      error(abort_action,errorbuffer);
      return code_bad;
   }

   // Read the function name, which is usually a number (e.g. "f19").
   if (sscanf(buffer, "f%d", &func.usernum) != 1)
   {
      error(abort_action, "Error reading a function name");
      return code_bad;
   }

   if (func.usernum < 0)
   {
      (void)sprintf(errorbuffer, "Function numbers must be non-negative (%d not allowed)", func.usernum);
      error(abort_action,errorbuffer);
      return code_bad;
   }

   if (muscle_function)
      (void)sprintf(fname, "muscle function f%d", func.usernum);
   else
      (void)sprintf(fname, "f%d", func.usernum);

   // Get the array of x-y pairs which comprises the function.
   if (read_double_array(fp, endTag, fname, &func) == code_bad)
   {
      free(func.x);
      free(func.y);
      free(func.b);
      free(func.c);
      free(func.d);
      return code_bad;
   }

   // If you made it to here, a valid function with two or more points has been
   // read-in. You want to leave the arrays large so that the user can add more
   // points to the function. So just calculate the coefficients, and then load
   // it into the model's function list.
   calc_function_coefficients(&func);

   if (load_user_function(ms, &func) == NULL)
      return code_bad;

   return code_fine;
}


/* ENTER_FUNCTION: whenever a function number is read from a file, the
 * user-specified number is compared to the existing array of functions to
 * see if that one has already been defined. If it has, the routine just
 * returns the internal number of that function. If it has not yet been
 * defined, it adds the number to a new element in the function list.
 */
int enter_function(ModelStruct* model, int usernum, SBoolean permission_to_add)
{
   int i, new_func, old_count;
   ReturnCode rc1 = code_fine, rc2 = code_fine;

   if (usernum != UNDEFINED_USERFUNCNUM)
   {
      for (i=0; i<model->func_array_size; i++)
      {
         if (model->function[i] && usernum == model->function[i]->usernum)
         {
            if (model->function[i]->status == dpFunctionSimmDefined)
            {
               // If the usernum is already being used by a SIMM-defined function,
               // change the usernum of the SIMM-defined function.
               if (permission_to_add == yes)
               {
                  model->function[i]->usernum = findHighestUserFuncNum(model) + 1;
                  break;
               }
               else // You can't match a user-defined function with a SIMM-defined one.
               {
                  return INVALID_FUNCTION;
               }
            }
            else
            {
               return i;
            }
         }
      }
   }

   if (permission_to_add == no)
      return INVALID_FUNCTION;

   for (i=0; i<model->func_array_size; i++)
      if (model->function[i] == NULL)
         break;
   new_func = i;

   if (new_func == model->func_array_size)
   {
      old_count = model->func_array_size;
      model->func_array_size += FUNC_ARRAY_INCREMENT;
      model->function = (dpFunction**)simm_realloc(model->function, model->func_array_size*sizeof(dpFunction*), &rc1);
      model->save.function = (dpFunction**)simm_realloc(model->save.function, model->func_array_size*sizeof(dpFunction*), &rc2);
      if (rc1 == code_bad || rc2 == code_bad)
      {
         model->func_array_size = old_count;
         return INVALID_FUNCTION;
      }

      for (i=old_count; i<model->func_array_size; i++)
      {
         model->function[i] = NULL;
         model->save.function[i] = NULL;
      }
   }

   model->function[new_func] = (dpFunction*)simm_calloc(1, sizeof(dpFunction));
   model->function[new_func]->used = dpYes;
   model->function[new_func]->status = dpFunctionUndefined;
   model->function[new_func]->usernum = usernum;
   model->function[new_func]->numpoints = 0;
   model->function[new_func]->coefficient_array_size = 0;
   model->function[new_func]->cutoff_frequency = 0.0;

   return new_func;
}

/* This function passes ownership of the x, y, b, c, d arrays in 'func' to the model. */
dpFunction* load_user_function(ModelStruct* model, dpFunction* func)
{
   // Get a slot to put the function in. enter_function() will allocate space for
   // the dpFunction structure, but not for the x, y, b, c, d arrays, which are
   // taken from 'func.'
   int funcIndex = enter_function(model, func->usernum, yes);

   // This should happen only when the function array can't be expanded
   // to hold the new function.
   if (funcIndex == INVALID_FUNCTION)
      return NULL;

   if (model->function[funcIndex]->status != dpFunctionUndefined)
   {
      (void)sprintf(errorbuffer, "Warning: redefinition of function f%d. Replacing earlier definition.", func->usernum);
      error(none, errorbuffer);
   }

   model->function[funcIndex]->type = func->type;
   model->function[funcIndex]->numpoints = func->numpoints;
   model->function[funcIndex]->coefficient_array_size = func->coefficient_array_size;
   model->function[funcIndex]->cutoff_frequency = func->cutoff_frequency;
   model->function[funcIndex]->x = func->x;
   model->function[funcIndex]->y = func->y;
   model->function[funcIndex]->b = func->b;
   model->function[funcIndex]->c = func->c;
   model->function[funcIndex]->d = func->d;
   model->function[funcIndex]->status = dpFunctionUserDefined;
   model->function[funcIndex]->usernum = func->usernum;
   model->function[funcIndex]->used = dpYes;
   model->function[funcIndex]->source = func->source;

   //Put a copy of the function in model->save.function.
   free_function(model->save.function[funcIndex], no);
   if (model->save.function[funcIndex] == NULL)
      model->save.function[funcIndex] = (dpFunction*)simm_calloc(1, sizeof(dpFunction));
   malloc_function(model->save.function[funcIndex], func->coefficient_array_size);
   copy_function(model->function[funcIndex], model->save.function[funcIndex]);

   return model->function[funcIndex];
}


//TODO5.0: backup the function in model.save.function?
/* This function passes ownership of the x, y, b, c, d arrays in 'func' to the model. */
dpFunction* load_simm_function(ModelStruct* model, dpFunction* func, SBoolean isMuscleFunc)
{
   int i, funcIndex;

   // Find an unused slot in the function array.
   for (i=0; i<model->func_array_size; i++)
      if (model->function[i] == NULL)
         break;
   funcIndex = i;

   if (funcIndex == model->func_array_size)
   {
      ReturnCode rc1, rc2;
      int old_count = model->func_array_size;
      model->func_array_size += FUNC_ARRAY_INCREMENT;
      model->function = (dpFunction**)simm_realloc(model->function, model->func_array_size*sizeof(dpFunction*), &rc1);
      model->save.function = (dpFunction**)simm_realloc(model->save.function, model->func_array_size*sizeof(dpFunction*), &rc2);
      if (rc1 == code_bad || rc2 == code_bad)
      {
         model->func_array_size = old_count;
         return NULL;
      }

      for (i=old_count; i<model->func_array_size; i++)
      {
         model->function[i] = NULL;
         model->save.function[i] = NULL;
      }
   }

   model->function[funcIndex] = (dpFunction*)simm_calloc(1, sizeof(dpFunction));
   model->function[funcIndex]->usernum = findHighestUserFuncNum(model) + 1;
   model->function[funcIndex]->source = (isMuscleFunc) ? dpMuscleFile : dpJointFile;
   model->function[funcIndex]->type = func->type;
   model->function[funcIndex]->numpoints = func->numpoints;
   model->function[funcIndex]->coefficient_array_size = func->coefficient_array_size;
   model->function[funcIndex]->cutoff_frequency = func->cutoff_frequency;
   model->function[funcIndex]->x = func->x;
   model->function[funcIndex]->y = func->y;
   model->function[funcIndex]->b = func->b;
   model->function[funcIndex]->c = func->c;
   model->function[funcIndex]->d = func->d;
   model->function[funcIndex]->status = dpFunctionSimmDefined;
   model->function[funcIndex]->used = dpYes;

   //Put a copy of the function in model->save.function.
   free_function(model->save.function[funcIndex], no);
   if (model->save.function[funcIndex] == NULL)
      model->save.function[funcIndex] = (dpFunction*)simm_calloc(1, sizeof(dpFunction));
   malloc_function(model->save.function[funcIndex], func->coefficient_array_size);
   copy_function(model->function[funcIndex], model->save.function[funcIndex]);

   return model->function[funcIndex];
}


ReturnCode malloc_function(dpFunction* func, int size)
{
   func->numpoints = 0;

   if ((func->x = (double*)simm_malloc(size*sizeof(double))) == NULL)
      return code_bad;
   if ((func->y = (double*)simm_malloc(size*sizeof(double))) == NULL)
      return code_bad;
   if ((func->b = (double*)simm_malloc(size*sizeof(double))) == NULL)
      return code_bad;
   if ((func->c = (double*)simm_malloc(size*sizeof(double))) == NULL)
      return code_bad;
   if ((func->d = (double*)simm_malloc(size*sizeof(double))) == NULL)
      return code_bad;

   func->coefficient_array_size = size;

   return code_fine;
}


ReturnCode realloc_function(dpFunction* func, int size)
{
   ReturnCode rc1 = code_fine, rc2 = code_fine, rc3 = code_fine, rc4 = code_fine, rc5 = code_fine;

   func->x = (double*)simm_realloc(func->x,size*sizeof(double),&rc1);
   func->y = (double*)simm_realloc(func->y,size*sizeof(double),&rc2);
   func->b = (double*)simm_realloc(func->b,size*sizeof(double),&rc3);
   func->c = (double*)simm_realloc(func->c,size*sizeof(double),&rc4);
   func->d = (double*)simm_realloc(func->d,size*sizeof(double),&rc5);

   if (rc1 == code_bad || rc2 == code_bad || rc3 == code_bad || rc4 == code_bad ||
       rc5 == code_bad)
      return code_bad;

   func->coefficient_array_size = size;

   return code_fine;
}


/* CALC_NATURAL_CUBIC_COEFFICIENTS: this function computes the coefficients
 * of a natural cubic spline defined by the XY control points. The code
 * is translated from a Fortran version printed in "Computer Methods
 * for Mathematical Computations" by Forsythe, Malcolm, and Moler,
 * pp 77-8. To better handle splines which have two or more control points
 * with the same X values, checks were added to make sure that the code never
 * divides by zero.
 */
static void calc_natural_cubic_coefficients(dpFunction* f)
{
   int nm1, nm2, i, j;
   double t;

   if (f->numpoints < 2 || f->used == dpNo || f->type != dpNaturalCubicSpline)
      return;

   if (f->numpoints == 2)
   {
      t = _MAX(TINY_NUMBER,f->x[1]-f->x[0]);
      f->b[0] = f->b[1] = (f->y[1]-f->y[0])/t;
      f->c[0] = f->c[1] = 0.0;
      f->d[0] = f->d[1] = 0.0;
      return;
   }

   nm1 = f->numpoints - 1;
   nm2 = f->numpoints - 2;

   /* Set up tridiagonal system:
    * b = diagonal, d = offdiagonal, c = right-hand side
    */
   f->d[0] = _MAX(TINY_NUMBER,f->x[1] - f->x[0]);
   f->c[1] = (f->y[1]-f->y[0])/f->d[0];
   for (i=1; i<nm1; i++)
   {
      f->d[i] = _MAX(TINY_NUMBER,f->x[i+1] - f->x[i]);
      f->b[i] = 2.0*(f->d[i-1]+f->d[i]);
      f->c[i+1] = (f->y[i+1]-f->y[i])/f->d[i];
      f->c[i] = f->c[i+1] - f->c[i];
   }

   /* End conditions. Third derivatives at x[0] and x[n-1]
    * are obtained from divided differences.
    */
   f->b[0] = -f->d[0];
   f->b[nm1] = -f->d[nm2];
   f->c[0] = 0.0;
   f->c[nm1] = 0.0;

   if (f->numpoints > 3)
   {
      double d1, d2, d3, d20, d30, d31;

      d31 = _MAX(TINY_NUMBER,f->x[3] - f->x[1]);
      d20 = _MAX(TINY_NUMBER,f->x[2] - f->x[0]);
      d1 = _MAX(TINY_NUMBER,f->x[nm1]-f->x[f->numpoints-3]);
      d2 = _MAX(TINY_NUMBER,f->x[nm2]-f->x[f->numpoints-4]);
      d30 = _MAX(TINY_NUMBER,f->x[3] - f->x[0]);
      d3 = _MAX(TINY_NUMBER,f->x[nm1]-f->x[f->numpoints-4]);
      f->c[0] = f->c[2]/d31 - f->c[1]/d20;
      f->c[nm1] = f->c[nm2]/d1 - f->c[f->numpoints-3]/d2;
      f->c[0] = f->c[0]*f->d[0]*f->d[0]/d30;
      f->c[nm1] = -f->c[nm1]*f->d[nm2]*f->d[nm2]/d3;
   }

   /* Forward elimination */
   for (i=1; i<f->numpoints; i++)
   {
      t = f->d[i-1]/f->b[i-1];
      f->b[i] -= t*f->d[i-1];
      f->c[i] -= t*f->c[i-1];
   }

   /* Back substitution */
   f->c[nm1] /= f->b[nm1];
   for (j=0; j<nm1; j++)
   {
      i = nm2 - j;
      f->c[i] = (f->c[i]-f->d[i]*f->c[i+1])/f->b[i];
   }

   /* compute polynomial coefficients */
   f->b[nm1] = (f->y[nm1]-f->y[nm2])/f->d[nm2] +
               f->d[nm2]*(f->c[nm2]+2.0*f->c[nm1]);
   for (i=0; i<nm1; i++)
   {
      f->b[i] = (f->y[i+1]-f->y[i])/f->d[i] - f->d[i]*(f->c[i+1]+2.0*f->c[i]);
      f->d[i] = (f->c[i+1]-f->c[i])/f->d[i];
      f->c[i] *= 3.0;
   }
   f->c[nm1] *= 3.0;
   f->d[nm1] = f->d[nm2];
}


/* CALC_FUNCTION_COEFFICIENTS: this function takes an array of x and y values,
 * and computes the coefficients to interpolate the data. It handles linear,
 * natural cubic, and GCV spline functions (step functions do not need
 * coefficients).
 */
void calc_function_coefficients(dpFunction* sf)
{
   if (sf == NULL)
      return;

   if (sf->status == dpFunctionUndefined)
      return;

   memset(sf->b, 0.0, sf->coefficient_array_size * sizeof(double));
   memset(sf->c, 0.0, sf->coefficient_array_size * sizeof(double));
   memset(sf->d, 0.0, sf->coefficient_array_size * sizeof(double));

    if (sf->type == dpLinearFunction)
    {
        int i;

        if (sf->numpoints < 2)
        {
            error(abort_action, "Error: linear function must have at least 2 points.");
            return;
        }

        /* For a linear function, the slope of each segment is stored in the b array. */
        for (i = 0; i < sf->numpoints - 1; i++)
            sf->b[i] = (sf->y[i+1] - sf->y[i]) / (sf->x[i+1] - sf->x[i]);
        sf->b[sf->numpoints - 1] = sf->b[sf->numpoints - 2];
    }
   else if (sf->type == dpNaturalCubicSpline)
   {
      calc_natural_cubic_coefficients(sf);
   }
   else if (sf->type == dpGCVSpline)
   {
      int i, ier = 0;
      double val, sampling_rate, *weight_x = NULL, *weight_y = NULL, *work = NULL;
      int mode = 1; // Mode for GCVSPL function chosen to specify smoothing parameter directly
      int order = 5; //5th order for quintic spline
      int half_order = (order + 1) / 2; // GCVSPL works with the half order
      int k = 1; // Only one colum of y data is splined at a time

      sampling_rate = 1.0 / (sf->x[1] - sf->x[0]);

      // Allocate memory as needed
      weight_x = (double*)simm_malloc(sf->numpoints*sizeof(double));
      weight_y = (double*)simm_malloc(1*sizeof(double));
      work = (double*)simm_malloc((6*(sf->numpoints*half_order+1)+sf->numpoints)*sizeof(double));

      // Assign weights
      for (i = 0; i < sf->numpoints; i++)
         weight_x[i] = 1.0;
      weight_y[0] = 1.0;

      // Calculate GCVSPL version of cut-off frequency. If cut-off frequency is <= 0.0,
      // set val = 0.0 so that no smoothing is performed.
      if (sf->cutoff_frequency <= 0.0)
         val = 0.0;
      else
         val = sampling_rate / pow(2.0*M_PI*sf->cutoff_frequency/pow(sqrt(2.0)-1.0,0.5/half_order), 2.0*half_order);

      gcvspl(sf->x, sf->y, &sf->numpoints, weight_x, weight_y, &half_order, 
             &sf->numpoints, &k, &mode, &val, sf->c, &sf->numpoints, work, &ier);

        free(weight_x);
      free(weight_y);
      free(work);

      if (ier > 0)
      {
         char buf[CHARBUFFER];
         if (ier == 1)
         {
            sprintf(buf, "GCV spline error: Only %d coordinates specified (%d or more are required).",
                    sf->numpoints, order + 1);
            error(abort_action, buf);
         }
         else if (ier == 2)
         {
            error(abort_action, "GCV spline error: X coordinates do not consistently increase in value.");
         }
         else
         {
            sprintf(buf, "Error code returned by gcvspl() = %d.", ier);
            error(abort_action, buf);
         }
         return;
      }
   }
}


/* INTERPOLATE_FUNCTION: given a function and an x-value, this routine
 * finds the corresponding y-value by interpolating the spline. It
 * can return the zeroth, first, or second derivative of the function
 * at that x-value.
 */
double interpolate_function(double abscissa, dpFunction* func, Derivative deriv, double velocity, double acceleration)
{
   int i, j, k, n;
   double dx;

   if (func->status == dpFunctionUndefined)
      return ERROR_DOUBLE;

   if (func->type == dpStepFunction)
   {
      if (deriv != zeroth)
         return 0.0;

      for (i = func->numpoints - 1; i >= 0; i--)
      {
         if (abscissa >= func->x[i] - ROUNDOFF_ERROR)
            return func->y[i];
      }

      /* If the abscissa is less than x[0], return y[0]. */
      return func->y[0];
   }
    else if (func->type == dpLinearFunction)
    {
      if (abscissa < func->x[0])
      {
         if (deriv == zeroth)
            return func->y[0] - (func->x[0] - abscissa) * func->b[0];
         else if (deriv == first)
            return func->b[0] * velocity;
         else
            return 0.0;
      }
      else
      {
         for (i = func->numpoints - 1; i >= 0; i--)
         {
            if (abscissa >= func->x[i] - ROUNDOFF_ERROR)
            {
               if (deriv == zeroth)
                  return func->y[i] + (abscissa - func->x[i]) * func->b[i];
               else if (deriv == first)
                  return func->b[i] * velocity;
               else
                  return 0.0;
            }
         }
      }
    }
   else if (func->type == dpGCVSpline)
   {
      int l = 1;
      int order = 5; //5th order for quintic spline
      int half_order = (order + 1) / 2; // GCVSPL works with the half order
      double work[20]; // size of work array must be >= order

      // TODO5.0: derivatives > 0 seem to be wrong (and they don't take into
      // account velocity or acceleration).
      return splder((int*)&deriv, &half_order, &func->numpoints, &abscissa, func->x, func->c, &l, work);
   }
   else // dpNaturalCubicSpline
   {
      n = func->numpoints;

      // Check if the abscissa is out of range of the function. If it is,
      // then use the slope of the function at the appropriate end point to
      // extrapolate. You do this rather than printing an error because the
      // assumption is that this will only occur in relatively harmless
      // situations (like a motion file that contains an out-of-range gencoord
      // value). The rest of the SIMM code has many checks to clamp a gencoord
      // value within its range of motion, so if you make it to this function
      // and the gencoord is still out of range, deal with it quietly.
      if (abscissa < func->x[0] - ROUNDOFF_ERROR)
      {
         if (deriv == zeroth)
            return func->y[0] + (abscissa - func->x[0])*func->b[0];
         if (deriv == first)
            return func->b[0]*velocity;
         if (deriv == second)
            return func->b[0]*acceleration;
      }
      else if (abscissa > func->x[n-1] + ROUNDOFF_ERROR)
      {
         if (deriv == zeroth)
            return func->y[n-1] + (abscissa - func->x[n-1])*func->b[n-1];
         if (deriv == first)
            return func->b[n-1]*velocity;
         if (deriv == second)
            return func->b[n-2]*acceleration;
      }

      // Check to see if the abscissa is close to one of the end points
      // (the binary search method doesn't work well if you are at one of the
      // end points.
      if (EQUAL_WITHIN_ERROR(abscissa, func->x[0]))
      {
         if (deriv == zeroth)
            return func->y[0];
         if (deriv == first)
            return func->b[0]*velocity;
         if (deriv == second)
            return func->b[0]*acceleration + 2.0*func->c[0]*velocity*velocity;
      }
      else if (EQUAL_WITHIN_ERROR(abscissa, func->x[n-1]))
      {
         if (deriv == zeroth)
            return func->y[n-1];
         if (deriv == first)
            return func->b[n-1]*velocity;
         if (deriv == second)
            return func->b[n-1]*acceleration + 2.0*func->c[n-1]*velocity*velocity;
      }

      if (n < 3)
      {
         // If there are only 2 function points, then set k to zero
         // (you've already checked to see if the abscissa is out of
         // range or equal to one of the endpoints).
         k = 0;
      }
      else
      {
         // Do a binary search to find which two points the abscissa is between.
         i = 0;
         j = n;
         while (1)
         {
            k = (i+j)/2;
            if (abscissa < func->x[k])
               j = k;
            else if (abscissa > func->x[k+1])
               i = k;
            else
               break;
         }
      }

      dx = abscissa - func->x[k];

      if (deriv == zeroth)
         return func->y[k] + dx*(func->b[k] + dx*(func->c[k] + dx*func->d[k]));

      if (deriv == first)
         return (func->b[k] + dx*(2.0*func->c[k] + 3.0*dx*func->d[k]))*velocity;

      if (deriv == second)
         return (func->b[k] + dx*(2.0*func->c[k] + 3.0*dx*func->d[k]))*acceleration +
         (2.0*func->c[k] + 6.0*dx*func->d[k])*velocity*velocity;
   }

   return ERROR_DOUBLE;
}


const char* get_function_type_name(dpFunctionType funcType)
{
   switch (funcType)
   {
      case dpStepFunction:
         return "step_function";
      case dpLinearFunction:
         return "linear_function";
      case dpNaturalCubicSpline:
         return "natural_cubic";
      case dpGCVSpline:
         return "gcv_spline";
      case dpFunctionTypeUndefined:
      default:
         return "function_type_undefined"; // should probably be an error
   }
}


const char* get_function_tag(dpFunctionType funcType, int beginEnd)
{
   // beginEnd = 0 for "begin", beginEnd = 1 for "end"
   switch (funcType)
   {
      case dpStepFunction:
         return (beginEnd) ? "endstepfunction" : "beginstepfunction";
      case dpLinearFunction:
         return (beginEnd) ? "endlinearfunction" : "beginlinearfunction";
      case dpNaturalCubicSpline:
         return (beginEnd) ? "endnaturalcubicspline" : "beginnaturalcubicspline";
      case dpGCVSpline:
         return (beginEnd) ? "endgcvspline" : "begingcvspline";
      case dpFunctionTypeUndefined:
      default:
         return (beginEnd) ? "endfunction" : "beginfunction"; // old-style function definitions
   }
}

dpFunction* clone_function(ModelStruct* model, dpFunction* oldFunction, SBoolean isMuscleFunc)
{
   dpFunction f;

   malloc_function(&f, oldFunction->coefficient_array_size);
   copy_function(oldFunction, &f);

   return load_simm_function(model, &f, isMuscleFunc);
}

#if ! ENGINE
void draw_function(dpFunction* func, float lineWidth, int color, double x1, double x2, int extrapolated_color)
{
   int i, j, numSteps;
   float currentLineWidth;
   double stepSize, pnt[2];

   glGetFloatv(GL_LINE_WIDTH, &currentLineWidth);

   if (func->type == dpNaturalCubicSpline || func->type == dpGCVSpline)
      numSteps = 48;
   else
      numSteps = 1;

   simm_color(color);

   for (i=0; i<func->numpoints-1; i++)
   {
      stepSize = (func->x[i+1] - func->x[i]) / numSteps;

      glLineWidth(lineWidth);
      glBegin(GL_LINE_STRIP);

      if (func->type == dpStepFunction)
      {
         pnt[0] = func->x[i];
         pnt[1] = func->y[i];
         glVertex2dv(pnt);
         pnt[0] = func->x[i+1];
         pnt[1] = func->y[i];
         glVertex2dv(pnt);
         pnt[0] = func->x[i+1];
         pnt[1] = func->y[i+1];
         glVertex2dv(pnt);
      }
      else
      {
         for (j=0; j<=numSteps; j++)
         {
            pnt[0] = func->x[i] + j * stepSize;
            pnt[1] = interpolate_function(pnt[0], func, zeroth, 0.0, 0.0);
            glVertex2dv(pnt);
         }
      }

      glEnd();
   }

   simm_color(extrapolated_color);
   glBegin(GL_LINES);
   pnt[0] = x1;
   pnt[1] = interpolate_function(pnt[0], func, zeroth, 0.0, 0.0);
   glVertex2dv(pnt);
   pnt[0] = func->x[0];
   pnt[1] = func->y[0];
   glVertex2dv(pnt);
   pnt[0] = func->x[func->numpoints-1];
   pnt[1] = func->y[func->numpoints-1];
   glVertex2dv(pnt);
   pnt[0] = x2;
   pnt[1] = interpolate_function(pnt[0], func, zeroth, 0.0, 0.0);
   glVertex2dv(pnt);
   glEnd();

   glLineWidth(currentLineWidth);
}
#endif


/* return the highest user function number */
int findHighestUserFuncNum(ModelStruct* ms)
{
   int i, highest_user_num = -1;

   for (i=0; i<ms->func_array_size; i++)
   {
       if (ms->function[i] && ms->function[i]->used == dpYes && ms->function[i]->usernum > highest_user_num)
          highest_user_num = ms->function[i]->usernum;
   }

   return highest_user_num;
}


int countUsedFunctions(ModelStruct* ms)
{
   int i, count=0;

   for (i=0; i<ms->func_array_size; i++)
       if (ms->function[i] && ms->function[i]->used == dpYes)
          count++;

   return count;
}


int getFunctionIndex(ModelStruct* model, dpFunction* function)
{
   int i;

   if (function)
   {
      for (i=0; i<model->func_array_size; i++)
         if (model->function[i] == function)
            return i;
   }

   return INVALID_FUNCTION;
}


dpFunction* getFunctionByUserNumber(ModelStruct* model, int userNumber)
{
   int i;

    for (i=0; i<model->func_array_size; i++)
        if (model->function[i] && model->function[i]->used == yes && model->function[i]->usernum == userNumber)
            return model->function[i];

   return NULL;
}


void save_function(ModelStruct* model, dpFunction* function)
{
   if (model && function)
   {
      int funcIndex = getFunctionIndex(model, function);
      if (funcIndex != INVALID_FUNCTION)
      {
         free_function(model->save.function[funcIndex], no);
         malloc_function(model->save.function[funcIndex], model->function[funcIndex]->coefficient_array_size);
         copy_function(model->function[funcIndex], model->save.function[funcIndex]);
      }
   }
}


void restore_function(ModelStruct* model, dpFunction* function)
{
   if (model && function)
   {
      int funcIndex = getFunctionIndex(model, function);
      if (funcIndex != INVALID_FUNCTION)
      {
         free_function(model->function[funcIndex], no);
         malloc_function(model->function[funcIndex], model->save.function[funcIndex]->coefficient_array_size);
         copy_function(model->save.function[funcIndex], model->function[funcIndex]);
      }
   }
}


void free_function(dpFunction* func, SBoolean freeTheFuncToo)
{
   if (func == NULL)
      return;

   FREE_IFNOTNULL(func->x);
   FREE_IFNOTNULL(func->y);
   FREE_IFNOTNULL(func->b);
   FREE_IFNOTNULL(func->c);
   FREE_IFNOTNULL(func->d);

   if (freeTheFuncToo)
   {
      free(func);
   }
   else
   {
      func->status = dpFunctionUndefined;
      func->used = dpNo;
   }
}
