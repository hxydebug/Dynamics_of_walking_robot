
#include <math.h>
#include "mex.h"
#include <stdio.h>
#include <stdlib.h>
#include "get_eom.c"

 
/* 
 *  Gateway routine
 */

void mexFunction(
  int nlhs, mxArray *plhs[],
  int nrhs, const mxArray *prhs[])
{
  int i;
  unsigned int ms, ns, ns2;
  double t;
  double *Z;
  double *params;
  double *A, *b;
  double *J_l, *J_r, *Jdot_l, *Jdot_r;

  // mexPrintf("Hello\n"); 
  
  /* Get the input argument, x, which is a vector */
  
  //[A,b] = dynamicsMEX(t,z0,params);
     t = mxGetScalar(prhs[0]);
     //mexPrintf("t = %f\n",t); 
     
     ms = mxGetM(prhs[1]); //Get Rows of prhs, prhs[1] is z0
     ns = mxGetN(prhs[1]); //Get Columns of prhs
     Z = mxGetPr(prhs[1]); // Get data elements of prhs 
     //mexPrintf("%d %d \n",ms,ns);
     
     params = mxGetPr(prhs[2]); // Get data elements of prhs
     
//  /* Create a matrix for the return argument */
//  // Return is A, b 
   //ms2 = 2; //Get Rows of prhs, prhs[1] is z0
   ns2 = ns/2; //Get Columns of prhs
   plhs[0] = mxCreateDoubleMatrix(ns2, ns2, mxREAL); //A
   plhs[1] = mxCreateDoubleMatrix(ns2, 1, mxREAL); //b
   //Although J and Jdots are 3 x ns2, we assign it as a transpose because MATLAB 
   // does column wise assignment (like Fortran) while C does row wise assignment
   // After call from mex, simply transport J and Jdot to get the correct term.
   plhs[2] = mxCreateDoubleMatrix(ns2, 3, mxREAL); //J_l 
   plhs[3] = mxCreateDoubleMatrix(ns2, 3, mxREAL); //J_r
   plhs[4] = mxCreateDoubleMatrix(ns2, 3, mxREAL); //Jdot_l
   plhs[5] = mxCreateDoubleMatrix(ns2, 3, mxREAL); //Jdot_r
//         
//   /* Dereference arguments */
   A = mxGetPr(plhs[0]);
   b = mxGetPr(plhs[1]);
   J_l = mxGetPr(plhs[2]);
   J_r = mxGetPr(plhs[3]);
   Jdot_l = mxGetPr(plhs[4]);
   Jdot_r = mxGetPr(plhs[5]);
  
   //get_dynamics(A,b,Z,params);
   get_dynamics(A,b,J_l,J_r,Jdot_l,Jdot_r,Z,params);
   //void void get_Ab(double *A, double *b, double *Z, double *params)(double *A, double *b, double *Z, double *params)
  /* call the computational routine */
 
}
