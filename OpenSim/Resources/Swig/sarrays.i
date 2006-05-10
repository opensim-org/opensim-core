%inline %{
/* Note: double[n] is equivalent to a pointer to a double */
double *new_doubleArray(int n) {
   return (double *) malloc(n*sizeof(double));
}
void free_doubleArray(double *x) {
   free(x);
}
void doubleArray_set(double* x, int i, double v) {
   x[i] = v;
}
double doubleArray_get(double *x, int i) {
   return x[i];
}


/* parallel code for float arrays */

float *new_floatArray(int n) {
   return (float *) malloc(n*sizeof(float));
}
void free_floatArray(float *x) {
   free(x);
}
void floatArray_set(float* x, int i, float v) {
   x[i] = v;
}
float floatArray_get(float *x, int i) {
   return x[i];
}


/* repeat for integer arrays */

int *new_intArray(int n) {
   return (int *) malloc(n*sizeof(int));
}
void free_intArray(int *x) {
   free(x);
}
void intArray_set(int* x, int i, int v) {
   x[i] = v;
}
int intArray_get(int *x, int i) {
   return x[i];
}
%}
