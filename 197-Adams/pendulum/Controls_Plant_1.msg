
-------------------------------------------------------------------------------
|        ..          ..                                                       |
|       ####*.      ####.                                                     |
|         `###*.       `#.              MSC Software                          |
|            `###.       #.             ------------                          |
|               `##.     ##                                                   |
|                 `##.   ##                                                   |
|                   `## ##'             A d a m s   C++   S o l v e r         |
|                     ####              Build:   2017.2.0-CL501464            |
|                     ###.              Version: Adams 2017.2                 |
|                    ###`#.                                                   |
|                  .###  `#                                                   |
|                 ####    `#                                                  |
|                ####      #.                                                 |
|              #####       `#           http://www.mscsoftware.com            |
|            .#####                                                           |
|           #####'                                                            |
|           ####                                                              |
|                                                                             |
|    *********************************************************************    |
|    *                                                                   *    |
|    *                      MSC Software Corporation                     *    |
|    *                                                                   *    |
|    *                             A d a m s                             *    |
|    *                                                                   *    |
|    *          Automatic Dynamic Analysis of Mechanical Systems         *    |
|    *                                                                   *    |
|    *********************************************************************    |
|    *                                                                   *    |
|    *                         LEGAL INFORMATION                         *    |
|    *                                                                   *    |
|    *  Warning:  This computer  program is protected  by copyright law  *    |
|    *  and  international  treaties.    Unauthorized  reproduction  or  *    |
|    *  distribution  of this  computer program, or any  portion of it,  *    |
|    *  may result in severe civil and criminal penalties.               *    |
|    *                                                                   *    |
|    *  Copyright (C) 2017 MSC Software Corporation and  its licensors.  *    |
|    *                        All rights reserved.                       *    |
|    *                                                                   *    |
|    *********************************************************************    |
|                                                                             |
|-----------------------------------------------------------------------------|
|                                                                             |
|    *********************************************************************    |
|    *                                                                   *    |
|    *   Local Time: 2018-03-03 22:33:21                                 *    |
|    *    Host Name: Power-LJR                                           *    |
|    *        Uname: MS Windows NT                                       *    |
|    *      OS Info: 6.2                                                 *    |
|    *         User: xlh                                                 *    |
|    *  Customer ID: DBD4EE4B-163E1A76                                   *    |
|    *  Current dir: F:\adam\pendulum                                    *    |
|    *                                                                   *    |
|    *********************************************************************    |
|                                                                             |
-------------------------------------------------------------------------------


---- START: WARNING ----
   Planar Part pendulum.rod_up is Constrained to another Part.
   This may cause Planar Part to move out of Global XY Plane.


---- END: WARNING ----

-------------------------------------------------------------------------------
|                                 Model Title                                 |
|=============================================================================|
|                                                                             |
|                                                                             |
|                                                                             |
-------------------------------------------------------------------------------

 command: simulate/initial_conditions


  Begin Displacement Initial Conditions

  Time T=0.000000000E+00.
  Degree-of-freedom analysis identified redundant constraints in the model:
  -------------------------------------------------------------------------
   -  deactivating constraint equation  Zi.Xj in pendulum.JOINT_4
   -  deactivating constraint equation  Zi.Yj in pendulum.JOINT_5

  End Displacement Initial Conditions


  Begin Velocity Initial Conditions


  End Velocity Initial Conditions


  Begin Acceleration Initial Conditions


---- START: WARNING ----

  The Reconcile Acceleration-Force has detected a singular Jacobian matrix.
  This problem may be caused by:
     (a) A constraint equations that became almost redundant, e.g., the model
         is at a singular configuration or at a  bifurcation point.
     (b) A massless PARTS in the model.

  The Sparse Solver has detected row 1 has zero length (during the symbolic pivot selection).
  The corresponding equation is: Part/1 X Force

  A minimization algorithm will be used to find a solution on the
  projected constraint manifold.

---- END: WARNING ----

  End Acceleration Initial Conditions
 command: var/7, fun=  1.00000000000000000e+00
 command: var/7, fun=  1.00000000000000000e+00


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   0.00000E+00  2.50000E-04            0            0       1            0.09 

  The dynamic solver is being restarted with the modified corrector. 


---- START: WARNING ----

  The Integrator has detected a singular Jacobian matrix.
  This problem may be caused by:
     (a) A constraint equations that became almost redundant, e.g., the model
         is at a singular configuration or at a  bifurcation point.
     (b) A massless PARTS in the model.

  The Calahan Sparse Solver has detected an unexpected singularity at row 54.
  The corresponding equation is: TrnJnt/3 Zi.Xj

  A minimization algorithm will be used to find a solution on the
  projected constraint manifold.

---- END: WARNING ----
   5.00000E-03  1.66667E-03           16            3       1            0.11 

  End Simulation
 command: var/7, fun= -7.23528202987625991e+04


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   5.00000E-03  1.66667E-03           16            3       1            0.11 
   1.00000E-02  1.89156E-03           74           23       2            0.13 

  End Simulation
 command: var/7, fun= -7.23528202987625991e+04


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   1.00000E-02  1.89156E-03           74           23       2            0.13 
   1.50000E-02  5.00000E-03           80           26       2            0.14 

  End Simulation
 command: var/7, fun=  2.04889048298496418e+04


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   1.50000E-02  5.00000E-03           80           26       2            0.14 
   2.00000E-02  5.00000E-03          114           36       2            0.14 

  End Simulation
 command: var/7, fun=  2.04889048298496418e+04


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   2.00000E-02  5.00000E-03          114           36       2            0.14 
   2.50000E-02  5.00000E-03          116           37       2            0.16 

  End Simulation
 command: var/7, fun=  1.81375318043565812e+02


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   2.50000E-02  5.00000E-03          116           37       2            0.16 
   3.00000E-02  5.00000E-03          140           44       2            0.17 

  End Simulation
 command: var/7, fun=  1.81375318043565812e+02


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   3.00000E-02  5.00000E-03          140           44       2            0.17 
   3.50000E-02  5.00000E-03          142           45       2            0.19 

  End Simulation
 command: var/7, fun= -9.54756634427567042e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   3.50000E-02  5.00000E-03          142           45       2            0.19 
   4.00000E-02  5.00000E-03          170           52       2            0.20 

  End Simulation
 command: var/7, fun= -9.54756634427567042e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   4.00000E-02  5.00000E-03          170           52       2            0.20 
   4.50000E-02  5.00000E-03          172           53       2            0.20 

  End Simulation
 command: var/7, fun= -2.14403823407860864e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   4.50000E-02  5.00000E-03          172           53       2            0.20 
   5.00000E-02  5.00000E-03          198           59       2            0.20 

  End Simulation
 command: var/7, fun= -2.14403823407860864e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   5.00000E-02  5.00000E-03          198           59       2            0.20 
   5.50000E-02  5.00000E-03          200           60       2            0.22 

  End Simulation
 command: var/7, fun= -3.08174144671109298e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   5.50000E-02  5.00000E-03          200           60       2            0.22 
   6.00000E-02  2.50000E-03          206           62       2            0.23 

  End Simulation
 command: var/7, fun= -3.08174144671109298e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   6.00000E-02  2.50000E-03          206           62       2            0.23 
   6.50000E-02  5.00000E-03          210           64       2            0.25 

  End Simulation
 command: var/7, fun= -4.11375220039750002e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   6.50000E-02  5.00000E-03          210           64       2            0.25 
   7.00000E-02  2.50000E-03          216           66       2            0.25 

  End Simulation
 command: var/7, fun= -4.11375220039750002e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   7.00000E-02  2.50000E-03          216           66       2            0.25 
   7.50000E-02  5.00000E-03          220           68       2            0.25 

  End Simulation
 command: var/7, fun= -3.55529593487210332e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   7.50000E-02  5.00000E-03          220           68       2            0.25 
   8.00000E-02  5.00000E-03          222           69       2            0.27 

  End Simulation
 command: var/7, fun= -3.55529593487210332e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   8.00000E-02  5.00000E-03          222           69       2            0.27 
   8.50000E-02  5.00000E-03          224           70       2            0.28 

  End Simulation
 command: var/7, fun= -3.57043307091509269e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   8.50000E-02  5.00000E-03          224           70       2            0.28 
   9.00000E-02  5.00000E-03          226           71       2            0.30 

  End Simulation
 command: var/7, fun= -3.57043307091509269e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   9.00000E-02  5.00000E-03          226           71       2            0.30 
   9.50000E-02  5.00000E-03          228           72       2            0.31 

  End Simulation
 command: var/7, fun= -3.66994349499910004e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   9.50000E-02  5.00000E-03          228           72       2            0.31 
   1.00000E-01  5.00000E-03          230           73       2            0.33 

  End Simulation
 command: var/7, fun= -3.66994349499910004e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   1.00000E-01  5.00000E-03          230           73       2            0.33 
   1.05000E-01  5.00000E-03          232           74       2            0.34 

  End Simulation
 command: var/7, fun= -3.63017221611648665e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   1.05000E-01  5.00000E-03          232           74       2            0.34 
   1.10000E-01  5.00000E-03          234           75       2            0.34 

  End Simulation
 command: var/7, fun= -3.63017221611648665e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   1.10000E-01  5.00000E-03          234           75       2            0.34 
   1.15000E-01  5.00000E-03          236           76       2            0.36 

  End Simulation
 command: var/7, fun= -3.62640066406005371e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   1.15000E-01  5.00000E-03          236           76       2            0.36 
   1.20000E-01  5.00000E-03          238           77       2            0.38 

  End Simulation
 command: var/7, fun= -3.62640066406005371e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   1.20000E-01  5.00000E-03          238           77       2            0.38 
   1.25000E-01  5.00000E-03          240           78       2            0.39 

  End Simulation
 command: var/7, fun= -3.63521885128458189e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   1.25000E-01  5.00000E-03          240           78       2            0.39 
   1.30000E-01  5.00000E-03          242           79       2            0.39 

  End Simulation
 command: var/7, fun= -3.63521885128458189e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   1.30000E-01  5.00000E-03          242           79       2            0.39 
   1.35000E-01  5.00000E-03          244           80       2            0.39 

  End Simulation
 command: var/7, fun= -3.63264440950156268e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   1.35000E-01  5.00000E-03          244           80       2            0.39 
   1.40000E-01  5.00000E-03          246           81       2            0.41 

  End Simulation
 command: var/7, fun= -3.63264440950156268e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   1.40000E-01  5.00000E-03          246           81       2            0.41 
   1.45000E-01  5.00000E-03          248           82       2            0.42 

  End Simulation
 command: var/7, fun= -3.63195698137468889e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   1.45000E-01  5.00000E-03          248           82       2            0.42 
   1.50000E-01  5.00000E-03          250           83       2            0.44 

  End Simulation
 command: var/7, fun= -3.63195698137468889e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   1.50000E-01  5.00000E-03          250           83       2            0.44 
   1.55000E-01  5.00000E-03          252           84       2            0.45 

  End Simulation
 command: var/7, fun= -3.63269256298652545e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   1.55000E-01  5.00000E-03          252           84       2            0.45 
   1.60000E-01  5.00000E-03          254           85       2            0.45 

  End Simulation
 command: var/7, fun= -3.63269256298652545e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   1.60000E-01  5.00000E-03          254           85       2            0.45 
   1.65000E-01  5.00000E-03          256           86       2            0.47 

  End Simulation
 command: var/7, fun= -3.63254733277105333e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   1.65000E-01  5.00000E-03          256           86       2            0.47 
   1.70000E-01  5.00000E-03          258           87       2            0.48 

  End Simulation
 command: var/7, fun= -3.63254733277105333e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   1.70000E-01  5.00000E-03          258           87       2            0.48 
   1.75000E-01  5.00000E-03          260           88       2            0.50 

  End Simulation
 command: var/7, fun= -3.63246414909178657e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   1.75000E-01  5.00000E-03          260           88       2            0.50 
   1.80000E-01  5.00000E-03          262           89       2            0.50 

  End Simulation
 command: var/7, fun= -3.63246414909178657e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   1.80000E-01  5.00000E-03          262           89       2            0.50 
   1.85000E-01  5.00000E-03          264           90       2            0.52 

  End Simulation
 command: var/7, fun= -3.63252215009099064e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   1.85000E-01  5.00000E-03          264           90       2            0.52 
   1.90000E-01  5.00000E-03          266           91       2            0.53 

  End Simulation
 command: var/7, fun= -3.63252215009099064e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   1.90000E-01  5.00000E-03          266           91       2            0.53 
   1.95000E-01  5.00000E-03          268           92       2            0.55 

  End Simulation
 command: var/7, fun= -3.63251610252124919e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   1.95000E-01  5.00000E-03          268           92       2            0.55 
   2.00000E-01  5.00000E-03          270           93       2            0.56 

  End Simulation
 command: var/7, fun= -3.63251610252124919e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   2.00000E-01  5.00000E-03          270           93       2            0.56 
   2.05000E-01  5.00000E-03          272           94       2            0.58 

  End Simulation
 command: var/7, fun= -3.63250753480214144e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   2.05000E-01  5.00000E-03          272           94       2            0.58 
   2.10000E-01  5.00000E-03          274           95       2            0.58 

  End Simulation
 command: var/7, fun= -3.63250753480214144e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   2.10000E-01  5.00000E-03          274           95       2            0.58 
   2.15000E-01  5.00000E-03          276           96       2            0.59 

  End Simulation
 command: var/7, fun= -3.63251184714477995e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   2.15000E-01  5.00000E-03          276           96       2            0.59 
   2.20000E-01  5.00000E-03          278           97       2            0.61 

  End Simulation
 command: var/7, fun= -3.63251184714477995e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   2.20000E-01  5.00000E-03          278           97       2            0.61 
   2.25000E-01  5.00000E-03          280           98       2            0.61 

  End Simulation
 command: var/7, fun= -3.63251184188683783e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   2.25000E-01  5.00000E-03          280           98       2            0.61 
   2.30000E-01  5.00000E-03          282           99       2            0.61 

  End Simulation
 command: var/7, fun= -3.63251184188683783e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   2.30000E-01  5.00000E-03          282           99       2            0.61 
   2.35000E-01  5.00000E-03          284          100       2            0.63 

  End Simulation
 command: var/7, fun= -3.63251103970648774e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   2.35000E-01  5.00000E-03          284          100       2            0.63 
   2.40000E-01  5.00000E-03          286          101       2            0.64 

  End Simulation
 command: var/7, fun= -3.63251103970648774e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   2.40000E-01  5.00000E-03          286          101       2            0.64 
   2.45000E-01  5.00000E-03          288          102       2            0.66 

  End Simulation
 command: var/7, fun= -3.63251133885867512e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   2.45000E-01  5.00000E-03          288          102       2            0.66 
   2.50000E-01  5.00000E-03          290          103       2            0.66 

  End Simulation
 command: var/7, fun= -3.63251133885867512e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   2.50000E-01  5.00000E-03          290          103       2            0.66 
   2.55000E-01  5.00000E-03          292          104       2            0.67 

  End Simulation
 command: var/7, fun= -3.63251137724816363e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   2.55000E-01  5.00000E-03          292          104       2            0.67 
   2.60000E-01  5.00000E-03          294          105       2            0.69 

  End Simulation
 command: var/7, fun= -3.63251137724816363e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   2.60000E-01  5.00000E-03          294          105       2            0.69 
   2.65000E-01  5.00000E-03          296          106       2            0.70 

  End Simulation
 command: var/7, fun= -3.63251130719498178e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   2.65000E-01  5.00000E-03          296          106       2            0.70 
   2.70000E-01  5.00000E-03          298          107       2            0.72 

  End Simulation
 command: var/7, fun= -3.63251130719498178e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   2.70000E-01  5.00000E-03          298          107       2            0.72 
   2.75000E-01  5.00000E-03          300          108       2            0.72 

  End Simulation
 command: var/7, fun= -3.63251132607525778e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   2.75000E-01  5.00000E-03          300          108       2            0.72 
   2.80000E-01  5.00000E-03          302          109       2            0.73 

  End Simulation
 command: var/7, fun= -3.63251132607525778e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   2.80000E-01  5.00000E-03          302          109       2            0.73 
   2.85000E-01  5.00000E-03          304          110       2            0.75 

  End Simulation
 command: var/7, fun= -3.63251133212045443e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   2.85000E-01  5.00000E-03          304          110       2            0.75 
   2.90000E-01  5.00000E-03          306          111       2            0.77 

  End Simulation
 command: var/7, fun= -3.63251133212045443e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   2.90000E-01  5.00000E-03          306          111       2            0.77 
   2.95000E-01  5.00000E-03          308          112       2            0.77 

  End Simulation
 command: var/7, fun= -3.63251132635278464e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   2.95000E-01  5.00000E-03          308          112       2            0.77 
   3.00000E-01  5.00000E-03          311          113       2            0.78 

  End Simulation
 command: var/7, fun= -3.63251132635278464e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   3.00000E-01  5.00000E-03          311          113       2            0.78 
   3.05000E-01  5.00000E-03          314          114       2            0.78 

  End Simulation
 command: var/7, fun= -3.63251132736937825e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   3.05000E-01  5.00000E-03          314          114       2            0.78 
   3.10000E-01  5.00000E-03          317          115       2            0.81 

  End Simulation
 command: var/7, fun= -3.63251132736937825e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   3.10000E-01  5.00000E-03          317          115       2            0.81 
   3.15000E-01  5.00000E-03          320          116       2            0.83 

  End Simulation
 command: var/7, fun= -3.63251132806700207e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   3.15000E-01  5.00000E-03          320          116       2            0.83 
   3.20000E-01  5.00000E-03          323          117       2            0.83 

  End Simulation
 command: var/7, fun= -3.63251132806700207e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   3.20000E-01  5.00000E-03          323          117       2            0.83 
   3.25000E-01  5.00000E-03          326          118       2            0.84 

  End Simulation
 command: var/7, fun= -3.63251132761814415e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   3.25000E-01  5.00000E-03          326          118       2            0.84 
   3.30000E-01  5.00000E-03          329          119       2            0.86 

  End Simulation
 command: var/7, fun= -3.63251132761814415e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   3.30000E-01  5.00000E-03          329          119       2            0.86 
   3.35000E-01  5.00000E-03          331          120       2            0.88 

  End Simulation
 command: var/7, fun= -3.63251132765488228e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   3.35000E-01  5.00000E-03          331          120       2            0.88 
   3.40000E-01  5.00000E-03          333          121       2            0.88 

  End Simulation
 command: var/7, fun= -3.63251132765488228e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   3.40000E-01  5.00000E-03          333          121       2            0.88 
   3.45000E-01  5.00000E-03          335          122       2            0.89 

  End Simulation
 command: var/7, fun= -3.63251132772492474e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   3.45000E-01  5.00000E-03          335          122       2            0.89 
   3.50000E-01  5.00000E-03          337          123       2            0.89 

  End Simulation
 command: var/7, fun= -3.63251132772492474e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   3.50000E-01  5.00000E-03          337          123       2            0.89 
   3.55000E-01  5.00000E-03          339          124       2            0.91 

  End Simulation
 command: var/7, fun= -3.63251132769205560e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   3.55000E-01  5.00000E-03          339          124       2            0.91 
   3.60000E-01  5.00000E-03          341          125       2            0.92 

  End Simulation
 command: var/7, fun= -3.63251132769205560e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   3.60000E-01  5.00000E-03          341          125       2            0.92 
   3.65000E-01  5.00000E-03          343          126       2            0.94 

  End Simulation
 command: var/7, fun= -3.63251132769120250e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   3.65000E-01  5.00000E-03          343          126       2            0.94 
   3.70000E-01  5.00000E-03          345          127       2            0.94 

  End Simulation
 command: var/7, fun= -3.63251132769120250e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   3.70000E-01  5.00000E-03          345          127       2            0.94 
   3.75000E-01  5.00000E-03          347          128       2            0.94 

  End Simulation
 command: var/7, fun= -3.63251132769767082e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   3.75000E-01  5.00000E-03          347          128       2            0.94 
   3.80000E-01  5.00000E-03          349          129       2            0.95 

  End Simulation
 command: var/7, fun= -3.63251132769767082e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   3.80000E-01  5.00000E-03          349          129       2            0.95 
   3.85000E-01  5.00000E-03          351          130       2            0.97 

  End Simulation
 command: var/7, fun= -3.63251132769542392e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   3.85000E-01  5.00000E-03          351          130       2            0.97 
   3.90000E-01  5.00000E-03          353          131       2            0.98 

  End Simulation
 command: var/7, fun= -3.63251132769542392e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   3.90000E-01  5.00000E-03          353          131       2            0.98 
   3.95000E-01  5.00000E-03          355          132       2            0.98 

  End Simulation
 command: var/7, fun= -3.63251132769504466e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   3.95000E-01  5.00000E-03          355          132       2            0.98 
   4.00000E-01  5.00000E-03          357          133       2            0.98 

  End Simulation
 command: var/7, fun= -3.63251132769504466e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   4.00000E-01  5.00000E-03          357          133       2            0.98 
   4.05000E-01  5.00000E-03          359          134       2            1.00 

  End Simulation
 command: var/7, fun= -3.63251132769560400e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   4.05000E-01  5.00000E-03          359          134       2            1.00 
   4.10000E-01  5.00000E-03          361          135       2            1.02 

  End Simulation
 command: var/7, fun= -3.63251132769560400e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   4.10000E-01  5.00000E-03          361          135       2            1.02 
   4.15000E-01  5.00000E-03          364          136       2            1.03 

  End Simulation
 command: var/7, fun= -3.63251132769546030e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   4.15000E-01  5.00000E-03          364          136       2            1.03 
   4.20000E-01  5.00000E-03          367          137       2            1.03 

  End Simulation
 command: var/7, fun= -3.63251132769546030e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   4.20000E-01  5.00000E-03          367          137       2            1.03 
   4.25000E-01  5.00000E-03          370          138       2            1.05 

  End Simulation
 command: var/7, fun= -3.63251132769541982e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   4.25000E-01  5.00000E-03          370          138       2            1.05 
   4.30000E-01  5.00000E-03          373          139       2            1.06 

  End Simulation
 command: var/7, fun= -3.63251132769541982e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   4.30000E-01  5.00000E-03          373          139       2            1.06 
   4.35000E-01  5.00000E-03          376          140       2            1.08 

  End Simulation
 command: var/7, fun= -3.63251132769545802e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   4.35000E-01  5.00000E-03          376          140       2            1.08 
   4.40000E-01  5.00000E-03          379          141       2            1.08 

  End Simulation
 command: var/7, fun= -3.63251132769545802e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   4.40000E-01  5.00000E-03          379          141       2            1.08 
   4.45000E-01  5.00000E-03          382          142       2            1.08 

  End Simulation
 command: var/7, fun= -3.63251132769544984e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   4.45000E-01  5.00000E-03          382          142       2            1.08 
   4.50000E-01  5.00000E-03          385          143       2            1.09 

  End Simulation
 command: var/7, fun= -3.63251132769544984e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   4.50000E-01  5.00000E-03          385          143       2            1.09 
   4.55000E-01  5.00000E-03          388          144       2            1.11 

  End Simulation
 command: var/7, fun= -3.63251132769545438e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   4.55000E-01  5.00000E-03          388          144       2            1.11 
   4.60000E-01  5.00000E-03          391          145       2            1.13 

  End Simulation
 command: var/7, fun= -3.63251132769545438e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   4.60000E-01  5.00000E-03          391          145       2            1.13 
   4.65000E-01  5.00000E-03          394          146       2            1.13 

  End Simulation
 command: var/7, fun= -3.63251132769543938e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   4.65000E-01  5.00000E-03          394          146       2            1.13 
   4.70000E-01  5.00000E-03          397          147       2            1.13 

  End Simulation
 command: var/7, fun= -3.63251132769543938e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   4.70000E-01  5.00000E-03          397          147       2            1.13 
   4.75000E-01  5.00000E-03          400          148       2            1.14 

  End Simulation
 command: var/7, fun= -3.63251132769545802e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   4.75000E-01  5.00000E-03          400          148       2            1.14 
   4.80000E-01  5.00000E-03          403          149       2            1.16 

  End Simulation
 command: var/7, fun= -3.63251132769545802e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   4.80000E-01  5.00000E-03          403          149       2            1.16 
   4.85000E-01  5.00000E-03          406          150       2            1.17 

  End Simulation
 command: var/7, fun= -3.63251132769544165e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   4.85000E-01  5.00000E-03          406          150       2            1.17 
   4.90000E-01  5.00000E-03          409          151       2            1.19 

  End Simulation
 command: var/7, fun= -3.63251132769544165e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   4.90000E-01  5.00000E-03          409          151       2            1.19 
   4.95000E-01  5.00000E-03          412          152       2            1.19 

  End Simulation
 command: var/7, fun= -3.63251132769545029e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   4.95000E-01  5.00000E-03          412          152       2            1.19 
   5.00000E-01  5.00000E-03          415          153       2            1.19 

  End Simulation
 command: var/7, fun= -3.63251132769545029e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   5.00000E-01  5.00000E-03          415          153       2            1.19 
   5.05000E-01  5.00000E-03          418          154       2            1.20 

  End Simulation
 command: var/7, fun= -3.63251132769544529e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   5.05000E-01  5.00000E-03          418          154       2            1.20 
   5.10000E-01  5.00000E-03          421          155       2            1.22 

  End Simulation
 command: var/7, fun= -3.63251132769544529e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   5.10000E-01  5.00000E-03          421          155       2            1.22 
   5.15000E-01  5.00000E-03          424          156       2            1.23 

  End Simulation
 command: var/7, fun= -3.63251132769544256e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   5.15000E-01  5.00000E-03          424          156       2            1.23 
   5.20000E-01  5.00000E-03          427          157       2            1.23 

  End Simulation
 command: var/7, fun= -3.63251132769544256e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   5.20000E-01  5.00000E-03          427          157       2            1.23 
   5.25000E-01  5.00000E-03          430          158       2            1.25 

  End Simulation
 command: var/7, fun= -3.63251132769544984e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   5.25000E-01  5.00000E-03          430          158       2            1.25 
   5.30000E-01  5.00000E-03          433          159       2            1.27 

  End Simulation
 command: var/7, fun= -3.63251132769544984e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   5.30000E-01  5.00000E-03          433          159       2            1.27 
   5.35000E-01  5.00000E-03          436          160       2            1.27 

  End Simulation
 command: var/7, fun= -3.63251132769544483e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   5.35000E-01  5.00000E-03          436          160       2            1.27 
   5.40000E-01  5.00000E-03          439          161       2            1.28 

  End Simulation
 command: var/7, fun= -3.63251132769544483e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   5.40000E-01  5.00000E-03          439          161       2            1.28 
   5.45000E-01  5.00000E-03          441          162       2            1.30 

  End Simulation
 command: var/7, fun= -3.63251132769544165e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   5.45000E-01  5.00000E-03          441          162       2            1.30 
   5.50000E-01  5.00000E-03          443          163       2            1.31 

  End Simulation
 command: var/7, fun= -3.63251132769544165e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   5.50000E-01  5.00000E-03          443          163       2            1.31 
   5.55000E-01  5.00000E-03          445          164       2            1.31 

  End Simulation
 command: var/7, fun= -3.63251132769544938e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   5.55000E-01  5.00000E-03          445          164       2            1.31 
   5.60000E-01  5.00000E-03          447          165       2            1.33 

  End Simulation
 command: var/7, fun= -3.63251132769544938e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   5.60000E-01  5.00000E-03          447          165       2            1.33 
   5.65000E-01  5.00000E-03          449          166       2            1.34 

  End Simulation
 command: var/7, fun= -3.63251132769544529e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   5.65000E-01  5.00000E-03          449          166       2            1.34 
   5.70000E-01  5.00000E-03          451          167       2            1.36 

  End Simulation
 command: var/7, fun= -3.63251132769544529e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   5.70000E-01  5.00000E-03          451          167       2            1.36 
   5.75000E-01  5.00000E-03          453          168       2            1.38 

  End Simulation
 command: var/7, fun= -3.63251132769544165e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   5.75000E-01  5.00000E-03          453          168       2            1.38 
   5.80000E-01  5.00000E-03          455          169       2            1.38 

  End Simulation
 command: var/7, fun= -3.63251132769544165e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   5.80000E-01  5.00000E-03          455          169       2            1.38 
   5.85000E-01  5.00000E-03          457          170       2            1.38 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   5.85000E-01  5.00000E-03          457          170       2            1.38 
   5.90000E-01  5.00000E-03          459          171       2            1.39 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   5.90000E-01  5.00000E-03          459          171       2            1.39 
   5.95000E-01  5.00000E-03          461          172       2            1.39 

  End Simulation
 command: var/7, fun= -3.63251132769544529e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   5.95000E-01  5.00000E-03          461          172       2            1.39 
   6.00000E-01  5.00000E-03          463          173       2            1.41 

  End Simulation
 command: var/7, fun= -3.63251132769544529e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   6.00000E-01  5.00000E-03          463          173       2            1.41 
   6.05000E-01  5.00000E-03          465          174       2            1.42 

  End Simulation
 command: var/7, fun= -3.63251132769544029e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   6.05000E-01  5.00000E-03          465          174       2            1.42 
   6.10000E-01  5.00000E-03          467          175       2            1.44 

  End Simulation
 command: var/7, fun= -3.63251132769544029e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   6.10000E-01  5.00000E-03          467          175       2            1.44 
   6.15000E-01  5.00000E-03          469          176       2            1.44 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   6.15000E-01  5.00000E-03          469          176       2            1.44 
   6.20000E-01  5.00000E-03          471          177       2            1.44 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   6.20000E-01  5.00000E-03          471          177       2            1.44 
   6.25000E-01  5.00000E-03          473          178       2            1.45 

  End Simulation
 command: var/7, fun= -3.63251132769544483e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   6.25000E-01  5.00000E-03          473          178       2            1.45 
   6.30000E-01  5.00000E-03          475          179       2            1.47 

  End Simulation
 command: var/7, fun= -3.63251132769544483e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   6.30000E-01  5.00000E-03          475          179       2            1.47 
   6.35000E-01  5.00000E-03          477          180       2            1.48 

  End Simulation
 command: var/7, fun= -3.63251132769544120e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   6.35000E-01  5.00000E-03          477          180       2            1.48 
   6.40000E-01  5.00000E-03          479          181       2            1.48 

  End Simulation
 command: var/7, fun= -3.63251132769544120e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   6.40000E-01  5.00000E-03          479          181       2            1.48 
   6.45000E-01  5.00000E-03          481          182       2            1.48 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   6.45000E-01  5.00000E-03          481          182       2            1.48 
   6.50000E-01  5.00000E-03          483          183       2            1.50 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   6.50000E-01  5.00000E-03          483          183       2            1.50 
   6.55000E-01  5.00000E-03          485          184       2            1.52 

  End Simulation
 command: var/7, fun= -3.63251132769544483e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   6.55000E-01  5.00000E-03          485          184       2            1.52 
   6.60000E-01  5.00000E-03          487          185       2            1.52 

  End Simulation
 command: var/7, fun= -3.63251132769544483e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   6.60000E-01  5.00000E-03          487          185       2            1.52 
   6.65000E-01  5.00000E-03          489          186       2            1.55 

  End Simulation
 command: var/7, fun= -3.63251132769544074e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   6.65000E-01  5.00000E-03          489          186       2            1.55 
   6.70000E-01  5.00000E-03          491          187       2            1.55 

  End Simulation
 command: var/7, fun= -3.63251132769544074e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   6.70000E-01  5.00000E-03          491          187       2            1.55 
   6.75000E-01  5.00000E-03          493          188       2            1.56 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   6.75000E-01  5.00000E-03          493          188       2            1.56 
   6.80000E-01  5.00000E-03          495          189       2            1.58 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   6.80000E-01  5.00000E-03          495          189       2            1.58 
   6.85000E-01  5.00000E-03          497          190       2            1.59 

  End Simulation
 command: var/7, fun= -3.63251132769544483e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   6.85000E-01  5.00000E-03          497          190       2            1.59 
   6.90000E-01  5.00000E-03          499          191       2            1.61 

  End Simulation
 command: var/7, fun= -3.63251132769544483e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   6.90000E-01  5.00000E-03          499          191       2            1.61 
   6.95000E-01  5.00000E-03          501          192       2            1.63 

  End Simulation
 command: var/7, fun= -3.63251132769544074e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   6.95000E-01  5.00000E-03          501          192       2            1.63 
   7.00000E-01  5.00000E-03          503          193       2            1.63 

  End Simulation
 command: var/7, fun= -3.63251132769544074e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   7.00000E-01  5.00000E-03          503          193       2            1.63 
   7.05000E-01  5.00000E-03          505          194       2            1.64 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   7.05000E-01  5.00000E-03          505          194       2            1.64 
   7.10000E-01  5.00000E-03          507          195       2            1.66 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   7.10000E-01  5.00000E-03          507          195       2            1.66 
   7.15000E-01  5.00000E-03          509          196       2            1.67 

  End Simulation
 command: var/7, fun= -3.63251132769544483e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   7.15000E-01  5.00000E-03          509          196       2            1.67 
   7.20000E-01  5.00000E-03          511          197       2            1.69 

  End Simulation
 command: var/7, fun= -3.63251132769544483e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   7.20000E-01  5.00000E-03          511          197       2            1.69 
   7.25000E-01  5.00000E-03          513          198       2            1.69 

  End Simulation
 command: var/7, fun= -3.63251132769544074e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   7.25000E-01  5.00000E-03          513          198       2            1.69 
   7.30000E-01  5.00000E-03          515          199       2            1.70 

  End Simulation
 command: var/7, fun= -3.63251132769544074e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   7.30000E-01  5.00000E-03          515          199       2            1.70 
   7.35000E-01  5.00000E-03          517          200       2            1.72 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   7.35000E-01  5.00000E-03          517          200       2            1.72 
   7.40000E-01  5.00000E-03          519          201       2            1.73 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   7.40000E-01  5.00000E-03          519          201       2            1.73 
   7.45000E-01  5.00000E-03          521          202       2            1.73 

  End Simulation
 command: var/7, fun= -3.63251132769544483e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   7.45000E-01  5.00000E-03          521          202       2            1.73 
   7.50000E-01  5.00000E-03          523          203       2            1.75 

  End Simulation
 command: var/7, fun= -3.63251132769544483e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   7.50000E-01  5.00000E-03          523          203       2            1.75 
   7.55000E-01  5.00000E-03          525          204       2            1.77 

  End Simulation
 command: var/7, fun= -3.63251132769544074e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   7.55000E-01  5.00000E-03          525          204       2            1.77 
   7.60000E-01  5.00000E-03          527          205       2            1.78 

  End Simulation
 command: var/7, fun= -3.63251132769544074e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   7.60000E-01  5.00000E-03          527          205       2            1.78 
   7.65000E-01  5.00000E-03          529          206       2            1.78 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   7.65000E-01  5.00000E-03          529          206       2            1.78 
   7.70000E-01  5.00000E-03          531          207       2            1.80 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   7.70000E-01  5.00000E-03          531          207       2            1.80 
   7.75000E-01  5.00000E-03          533          208       2            1.81 

  End Simulation
 command: var/7, fun= -3.63251132769544438e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   7.75000E-01  5.00000E-03          533          208       2            1.81 
   7.80000E-01  5.00000E-03          535          209       2            1.83 

  End Simulation
 command: var/7, fun= -3.63251132769544438e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   7.80000E-01  5.00000E-03          535          209       2            1.83 
   7.85000E-01  5.00000E-03          537          210       2            1.84 

  End Simulation
 command: var/7, fun= -3.63251132769544074e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   7.85000E-01  5.00000E-03          537          210       2            1.84 
   7.90000E-01  5.00000E-03          539          211       2            1.84 

  End Simulation
 command: var/7, fun= -3.63251132769544074e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   7.90000E-01  5.00000E-03          539          211       2            1.84 
   7.95000E-01  5.00000E-03          541          212       2            1.86 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   7.95000E-01  5.00000E-03          541          212       2            1.86 
   8.00000E-01  5.00000E-03          543          213       2            1.88 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   8.00000E-01  5.00000E-03          543          213       2            1.88 
   8.05000E-01  5.00000E-03          545          214       2            1.89 

  End Simulation
 command: var/7, fun= -3.63251132769544483e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   8.05000E-01  5.00000E-03          545          214       2            1.89 
   8.10000E-01  5.00000E-03          547          215       2            1.91 

  End Simulation
 command: var/7, fun= -3.63251132769544483e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   8.10000E-01  5.00000E-03          547          215       2            1.91 
   8.15000E-01  5.00000E-03          549          216       2            1.91 

  End Simulation
 command: var/7, fun= -3.63251132769543983e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   8.15000E-01  5.00000E-03          549          216       2            1.91 
   8.20000E-01  5.00000E-03          551          217       2            1.92 

  End Simulation
 command: var/7, fun= -3.63251132769543983e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   8.20000E-01  5.00000E-03          551          217       2            1.92 
   8.25000E-01  5.00000E-03          553          218       2            1.94 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   8.25000E-01  5.00000E-03          553          218       2            1.94 
   8.30000E-01  5.00000E-03          555          219       2            1.95 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   8.30000E-01  5.00000E-03          555          219       2            1.95 
   8.35000E-01  5.00000E-03          557          220       2            1.95 

  End Simulation
 command: var/7, fun= -3.63251132769544483e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   8.35000E-01  5.00000E-03          557          220       2            1.95 
   8.40000E-01  5.00000E-03          559          221       2            1.97 

  End Simulation
 command: var/7, fun= -3.63251132769544483e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   8.40000E-01  5.00000E-03          559          221       2            1.97 
   8.45000E-01  5.00000E-03          561          222       2            1.98 

  End Simulation
 command: var/7, fun= -3.63251132769544074e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   8.45000E-01  5.00000E-03          561          222       2            1.98 
   8.50000E-01  5.00000E-03          563          223       2            2.00 

  End Simulation
 command: var/7, fun= -3.63251132769544074e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   8.50000E-01  5.00000E-03          563          223       2            2.00 
   8.55000E-01  5.00000E-03          565          224       2            2.02 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   8.55000E-01  5.00000E-03          565          224       2            2.02 
   8.60000E-01  5.00000E-03          567          225       2            2.02 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   8.60000E-01  5.00000E-03          567          225       2            2.02 
   8.65000E-01  5.00000E-03          569          226       2            2.03 

  End Simulation
 command: var/7, fun= -3.63251132769544483e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   8.65000E-01  5.00000E-03          569          226       2            2.03 
   8.70000E-01  5.00000E-03          571          227       2            2.05 

  End Simulation
 command: var/7, fun= -3.63251132769544483e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   8.70000E-01  5.00000E-03          571          227       2            2.05 
   8.75000E-01  5.00000E-03          573          228       2            2.06 

  End Simulation
 command: var/7, fun= -3.63251132769544074e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   8.75000E-01  5.00000E-03          573          228       2            2.06 
   8.80000E-01  5.00000E-03          575          229       2            2.06 

  End Simulation
 command: var/7, fun= -3.63251132769544074e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   8.80000E-01  5.00000E-03          575          229       2            2.06 
   8.85000E-01  5.00000E-03          577          230       2            2.06 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   8.85000E-01  5.00000E-03          577          230       2            2.06 
   8.90000E-01  5.00000E-03          579          231       2            2.08 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   8.90000E-01  5.00000E-03          579          231       2            2.08 
   8.95000E-01  5.00000E-03          581          232       2            2.09 

  End Simulation
 command: var/7, fun= -3.63251132769544483e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   8.95000E-01  5.00000E-03          581          232       2            2.09 
   9.00000E-01  5.00000E-03          583          233       2            2.11 

  End Simulation
 command: var/7, fun= -3.63251132769544483e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   9.00000E-01  5.00000E-03          583          233       2            2.11 
   9.05000E-01  5.00000E-03          585          234       2            2.11 

  End Simulation
 command: var/7, fun= -3.63251132769544074e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   9.05000E-01  5.00000E-03          585          234       2            2.11 
   9.10000E-01  5.00000E-03          587          235       2            2.13 

  End Simulation
 command: var/7, fun= -3.63251132769544074e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   9.10000E-01  5.00000E-03          587          235       2            2.13 
   9.15000E-01  5.00000E-03          589          236       2            2.14 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   9.15000E-01  5.00000E-03          589          236       2            2.14 
   9.20000E-01  5.00000E-03          591          237       2            2.16 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   9.20000E-01  5.00000E-03          591          237       2            2.16 
   9.25000E-01  5.00000E-03          593          238       2            2.17 

  End Simulation
 command: var/7, fun= -3.63251132769544483e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   9.25000E-01  5.00000E-03          593          238       2            2.17 
   9.30000E-01  5.00000E-03          595          239       2            2.17 

  End Simulation
 command: var/7, fun= -3.63251132769544483e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   9.30000E-01  5.00000E-03          595          239       2            2.17 
   9.35000E-01  5.00000E-03          597          240       2            2.19 

  End Simulation
 command: var/7, fun= -3.63251132769544074e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   9.35000E-01  5.00000E-03          597          240       2            2.19 
   9.40000E-01  5.00000E-03          599          241       2            2.20 

  End Simulation
 command: var/7, fun= -3.63251132769544074e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   9.40000E-01  5.00000E-03          599          241       2            2.20 
   9.45000E-01  5.00000E-03          601          242       2            2.22 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   9.45000E-01  5.00000E-03          601          242       2            2.22 
   9.50000E-01  5.00000E-03          603          243       2            2.22 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   9.50000E-01  5.00000E-03          603          243       2            2.22 
   9.55000E-01  5.00000E-03          605          244       2            2.23 

  End Simulation
 command: var/7, fun= -3.63251132769544438e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   9.55000E-01  5.00000E-03          605          244       2            2.23 
   9.60000E-01  5.00000E-03          607          245       2            2.25 

  End Simulation
 command: var/7, fun= -3.63251132769544438e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   9.60000E-01  5.00000E-03          607          245       2            2.25 
   9.65000E-01  5.00000E-03          609          246       2            2.27 

  End Simulation
 command: var/7, fun= -3.63251132769544074e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   9.65000E-01  5.00000E-03          609          246       2            2.27 
   9.70000E-01  5.00000E-03          611          247       2            2.27 

  End Simulation
 command: var/7, fun= -3.63251132769544074e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   9.70000E-01  5.00000E-03          611          247       2            2.27 
   9.75000E-01  5.00000E-03          613          248       2            2.28 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   9.75000E-01  5.00000E-03          613          248       2            2.28 
   9.80000E-01  5.00000E-03          615          249       2            2.30 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   9.80000E-01  5.00000E-03          615          249       2            2.30 
   9.85000E-01  5.00000E-03          617          250       2            2.31 

  End Simulation
 command: var/7, fun= -3.63251132769544483e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   9.85000E-01  5.00000E-03          617          250       2            2.31 
   9.90000E-01  5.00000E-03          619          251       2            2.33 

  End Simulation
 command: var/7, fun= -3.63251132769544483e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   9.90000E-01  5.00000E-03          619          251       2            2.33 
   9.95000E-01  5.00000E-03          621          252       2            2.33 

  End Simulation
 command: var/7, fun= -3.63251132769543983e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   9.95000E-01  5.00000E-03          621          252       2            2.33 
   1.00000E+00  5.00000E-03          623          253       2            2.33 

  End Simulation
 command: var/7, fun= -3.63251132769543983e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   1.00000E+00  5.00000E-03          623          253       2            2.33 
   1.00500E+00  5.00000E-03          625          254       2            2.34 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   1.00500E+00  5.00000E-03          625          254       2            2.34 
   1.01000E+00  5.00000E-03          627          255       2            2.36 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   1.01000E+00  5.00000E-03          627          255       2            2.36 
   1.01500E+00  5.00000E-03          629          256       2            2.36 

  End Simulation
 command: var/7, fun= -3.63251132769544483e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   1.01500E+00  5.00000E-03          629          256       2            2.36 
   1.02000E+00  5.00000E-03          631          257       2            2.38 

  End Simulation
 command: var/7, fun= -3.63251132769544483e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   1.02000E+00  5.00000E-03          631          257       2            2.38 
   1.02500E+00  5.00000E-03          633          258       2            2.38 

  End Simulation
 command: var/7, fun= -3.63251132769544074e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   1.02500E+00  5.00000E-03          633          258       2            2.38 
   1.03000E+00  5.00000E-03          635          259       2            2.39 

  End Simulation
 command: var/7, fun= -3.63251132769544074e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   1.03000E+00  5.00000E-03          635          259       2            2.39 
   1.03500E+00  5.00000E-03          637          260       2            2.41 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   1.03500E+00  5.00000E-03          637          260       2            2.41 
   1.04000E+00  5.00000E-03          639          261       2            2.42 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   1.04000E+00  5.00000E-03          639          261       2            2.42 
   1.04500E+00  5.00000E-03          641          262       2            2.44 

  End Simulation
 command: var/7, fun= -3.63251132769544483e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   1.04500E+00  5.00000E-03          641          262       2            2.44 
   1.05000E+00  5.00000E-03          643          263       2            2.44 

  End Simulation
 command: var/7, fun= -3.63251132769544483e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   1.05000E+00  5.00000E-03          643          263       2            2.44 
   1.05500E+00  5.00000E-03          645          264       2            2.45 

  End Simulation
 command: var/7, fun= -3.63251132769544074e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   1.05500E+00  5.00000E-03          645          264       2            2.45 
   1.06000E+00  5.00000E-03          647          265       2            2.45 

  End Simulation
 command: var/7, fun= -3.63251132769544074e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   1.06000E+00  5.00000E-03          647          265       2            2.45 
   1.06500E+00  5.00000E-03          649          266       2            2.45 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   1.06500E+00  5.00000E-03          649          266       2            2.45 
   1.07000E+00  5.00000E-03          651          267       2            2.47 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   1.07000E+00  5.00000E-03          651          267       2            2.47 
   1.07500E+00  5.00000E-03          653          268       2            2.48 

  End Simulation
 command: var/7, fun= -3.63251132769544483e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   1.07500E+00  5.00000E-03          653          268       2            2.48 
   1.08000E+00  5.00000E-03          655          269       2            2.48 

  End Simulation
 command: var/7, fun= -3.63251132769544483e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   1.08000E+00  5.00000E-03          655          269       2            2.48 
   1.08500E+00  5.00000E-03          657          270       2            2.52 

  End Simulation
 command: var/7, fun= -3.63251132769544074e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   1.08500E+00  5.00000E-03          657          270       2            2.52 
   1.09000E+00  5.00000E-03          659          271       2            2.53 

  End Simulation
 command: var/7, fun= -3.63251132769544074e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   1.09000E+00  5.00000E-03          659          271       2            2.53 
   1.09500E+00  5.00000E-03          661          272       2            2.53 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   1.09500E+00  5.00000E-03          661          272       2            2.53 
   1.10000E+00  5.00000E-03          663          273       2            2.53 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   1.10000E+00  5.00000E-03          663          273       2            2.53 
   1.10500E+00  5.00000E-03          665          274       2            2.55 

  End Simulation
 command: var/7, fun= -3.63251132769544483e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   1.10500E+00  5.00000E-03          665          274       2            2.55 
   1.11000E+00  5.00000E-03          667          275       2            2.56 

  End Simulation
 command: var/7, fun= -3.63251132769544483e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   1.11000E+00  5.00000E-03          667          275       2            2.56 
   1.11500E+00  5.00000E-03          669          276       2            2.58 

  End Simulation
 command: var/7, fun= -3.63251132769544074e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   1.11500E+00  5.00000E-03          669          276       2            2.58 
   1.12000E+00  5.00000E-03          671          277       2            2.58 

  End Simulation
 command: var/7, fun= -3.63251132769544074e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   1.12000E+00  5.00000E-03          671          277       2            2.58 
   1.12500E+00  5.00000E-03          673          278       2            2.58 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   1.12500E+00  5.00000E-03          673          278       2            2.58 
   1.13000E+00  5.00000E-03          675          279       2            2.59 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   1.13000E+00  5.00000E-03          675          279       2            2.59 
   1.13500E+00  5.00000E-03          677          280       2            2.61 

  End Simulation
 command: var/7, fun= -3.63251132769544438e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   1.13500E+00  5.00000E-03          677          280       2            2.61 
   1.14000E+00  5.00000E-03          679          281       2            2.63 

  End Simulation
 command: var/7, fun= -3.63251132769544438e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   1.14000E+00  5.00000E-03          679          281       2            2.63 
   1.14500E+00  5.00000E-03          681          282       2            2.63 

  End Simulation
 command: var/7, fun= -3.63251132769544074e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   1.14500E+00  5.00000E-03          681          282       2            2.63 
   1.15000E+00  5.00000E-03          683          283       2            2.64 

  End Simulation
 command: var/7, fun= -3.63251132769544074e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   1.15000E+00  5.00000E-03          683          283       2            2.64 
   1.15500E+00  5.00000E-03          685          284       2            2.66 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   1.15500E+00  5.00000E-03          685          284       2            2.66 
   1.16000E+00  5.00000E-03          687          285       2            2.67 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   1.16000E+00  5.00000E-03          687          285       2            2.67 
   1.16500E+00  5.00000E-03          689          286       2            2.69 

  End Simulation
 command: var/7, fun= -3.63251132769544483e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   1.16500E+00  5.00000E-03          689          286       2            2.69 
   1.17000E+00  5.00000E-03          691          287       2            2.69 

  End Simulation
 command: var/7, fun= -3.63251132769544483e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   1.17000E+00  5.00000E-03          691          287       2            2.69 
   1.17500E+00  5.00000E-03          693          288       2            2.69 

  End Simulation
 command: var/7, fun= -3.63251132769543983e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   1.17500E+00  5.00000E-03          693          288       2            2.69 
   1.18000E+00  5.00000E-03          695          289       2            2.70 

  End Simulation
 command: var/7, fun= -3.63251132769543983e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   1.18000E+00  5.00000E-03          695          289       2            2.70 
   1.18500E+00  5.00000E-03          697          290       2            2.72 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   1.18500E+00  5.00000E-03          697          290       2            2.72 
   1.19000E+00  5.00000E-03          699          291       2            2.72 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   1.19000E+00  5.00000E-03          699          291       2            2.72 
   1.19500E+00  5.00000E-03          701          292       2            2.73 

  End Simulation
 command: var/7, fun= -3.63251132769544483e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   1.19500E+00  5.00000E-03          701          292       2            2.73 
   1.20000E+00  5.00000E-03          703          293       2            2.75 

  End Simulation
 command: var/7, fun= -3.63251132769544483e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   1.20000E+00  5.00000E-03          703          293       2            2.75 
   1.20500E+00  5.00000E-03          705          294       2            2.77 

  End Simulation
 command: var/7, fun= -3.63251132769544074e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   1.20500E+00  5.00000E-03          705          294       2            2.77 
   1.21000E+00  5.00000E-03          707          295       2            2.78 

  End Simulation
 command: var/7, fun= -3.63251132769544074e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   1.21000E+00  5.00000E-03          707          295       2            2.78 
   1.21500E+00  5.00000E-03          709          296       2            2.80 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   1.21500E+00  5.00000E-03          709          296       2            2.80 
   1.22000E+00  5.00000E-03          711          297       2            2.80 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   1.22000E+00  5.00000E-03          711          297       2            2.80 
   1.22500E+00  5.00000E-03          713          298       2            2.81 

  End Simulation
 command: var/7, fun= -3.63251132769544483e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   1.22500E+00  5.00000E-03          713          298       2            2.81 
   1.23000E+00  5.00000E-03          715          299       2            2.83 

  End Simulation
 command: var/7, fun= -3.63251132769544483e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   1.23000E+00  5.00000E-03          715          299       2            2.83 
   1.23500E+00  5.00000E-03          717          300       2            2.84 

  End Simulation
 command: var/7, fun= -3.63251132769544074e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   1.23500E+00  5.00000E-03          717          300       2            2.84 
   1.24000E+00  5.00000E-03          719          301       2            2.86 

  End Simulation
 command: var/7, fun= -3.63251132769544074e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   1.24000E+00  5.00000E-03          719          301       2            2.86 
   1.24500E+00  5.00000E-03          721          302       2            2.88 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   1.24500E+00  5.00000E-03          721          302       2            2.88 
   1.25000E+00  5.00000E-03          723          303       2            2.88 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   1.25000E+00  5.00000E-03          723          303       2            2.88 
   1.25500E+00  5.00000E-03          725          304       2            2.88 

  End Simulation
 command: var/7, fun= -3.63251132769544483e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   1.25500E+00  5.00000E-03          725          304       2            2.88 
   1.26000E+00  5.00000E-03          727          305       2            2.89 

  End Simulation
 command: var/7, fun= -3.63251132769544483e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   1.26000E+00  5.00000E-03          727          305       2            2.89 
   1.26500E+00  5.00000E-03          729          306       2            2.91 

  End Simulation
 command: var/7, fun= -3.63251132769544074e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   1.26500E+00  5.00000E-03          729          306       2            2.91 
   1.27000E+00  5.00000E-03          731          307       2            2.91 

  End Simulation
 command: var/7, fun= -3.63251132769544074e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   1.27000E+00  5.00000E-03          731          307       2            2.91 
   1.27500E+00  5.00000E-03          733          308       2            2.91 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   1.27500E+00  5.00000E-03          733          308       2            2.91 
   1.28000E+00  5.00000E-03          735          309       2            2.92 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   1.28000E+00  5.00000E-03          735          309       2            2.92 
   1.28500E+00  5.00000E-03          737          310       2            2.94 

  End Simulation
 command: var/7, fun= -3.63251132769544483e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   1.28500E+00  5.00000E-03          737          310       2            2.94 
   1.29000E+00  5.00000E-03          739          311       2            2.95 

  End Simulation
 command: var/7, fun= -3.63251132769544483e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   1.29000E+00  5.00000E-03          739          311       2            2.95 
   1.29500E+00  5.00000E-03          741          312       2            2.97 

  End Simulation
 command: var/7, fun= -3.63251132769544120e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   1.29500E+00  5.00000E-03          741          312       2            2.97 
   1.30000E+00  5.00000E-03          743          313       2            2.98 

  End Simulation
 command: var/7, fun= -3.63251132769544120e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   1.30000E+00  5.00000E-03          743          313       2            2.98 
   1.30500E+00  5.00000E-03          745          314       2            2.98 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   1.30500E+00  5.00000E-03          745          314       2            2.98 
   1.31000E+00  5.00000E-03          747          315       2            3.00 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   1.31000E+00  5.00000E-03          747          315       2            3.00 
   1.31500E+00  5.00000E-03          749          316       2            3.02 

  End Simulation
 command: var/7, fun= -3.63251132769544483e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   1.31500E+00  5.00000E-03          749          316       2            3.02 
   1.32000E+00  5.00000E-03          751          317       2            3.03 

  End Simulation
 command: var/7, fun= -3.63251132769544483e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   1.32000E+00  5.00000E-03          751          317       2            3.03 
   1.32500E+00  5.00000E-03          754          318       2            3.05 

  End Simulation
 command: var/7, fun= -3.63251132769543983e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   1.32500E+00  5.00000E-03          754          318       2            3.05 
   1.33000E+00  5.00000E-03          757          319       2            3.05 

  End Simulation
 command: var/7, fun= -3.63251132769543983e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   1.33000E+00  5.00000E-03          757          319       2            3.05 
   1.33500E+00  5.00000E-03          760          320       2            3.06 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   1.33500E+00  5.00000E-03          760          320       2            3.06 
   1.34000E+00  5.00000E-03          763          321       2            3.08 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   1.34000E+00  5.00000E-03          763          321       2            3.08 
   1.34500E+00  5.00000E-03          766          322       2            3.09 

  End Simulation
 command: var/7, fun= -3.63251132769544483e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   1.34500E+00  5.00000E-03          766          322       2            3.09 
   1.35000E+00  5.00000E-03          769          323       2            3.09 

  End Simulation
 command: var/7, fun= -3.63251132769544483e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   1.35000E+00  5.00000E-03          769          323       2            3.09 
   1.35500E+00  5.00000E-03          772          324       2            3.11 

  End Simulation
 command: var/7, fun= -3.63251132769544120e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   1.35500E+00  5.00000E-03          772          324       2            3.11 
   1.36000E+00  5.00000E-03          775          325       2            3.11 

  End Simulation
 command: var/7, fun= -3.63251132769544120e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   1.36000E+00  5.00000E-03          775          325       2            3.11 
   1.36500E+00  5.00000E-03          778          326       2            3.13 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   1.36500E+00  5.00000E-03          778          326       2            3.13 
   1.37000E+00  5.00000E-03          781          327       2            3.14 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   1.37000E+00  5.00000E-03          781          327       2            3.14 
   1.37500E+00  5.00000E-03          784          328       2            3.14 

  End Simulation
 command: var/7, fun= -3.63251132769544483e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   1.37500E+00  5.00000E-03          784          328       2            3.14 
   1.38000E+00  5.00000E-03          787          329       2            3.14 

  End Simulation
 command: var/7, fun= -3.63251132769544483e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   1.38000E+00  5.00000E-03          787          329       2            3.14 
   1.38500E+00  5.00000E-03          790          330       2            3.16 

  End Simulation
 command: var/7, fun= -3.63251132769544074e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   1.38500E+00  5.00000E-03          790          330       2            3.16 
   1.39000E+00  5.00000E-03          793          331       2            3.17 

  End Simulation
 command: var/7, fun= -3.63251132769544074e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   1.39000E+00  5.00000E-03          793          331       2            3.17 
   1.39500E+00  5.00000E-03          796          332       2            3.19 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   1.39500E+00  5.00000E-03          796          332       2            3.19 
   1.40000E+00  5.00000E-03          799          333       2            3.20 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   1.40000E+00  5.00000E-03          799          333       2            3.20 
   1.40500E+00  5.00000E-03          802          334       2            3.20 

  End Simulation
 command: var/7, fun= -3.63251132769544483e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   1.40500E+00  5.00000E-03          802          334       2            3.20 
   1.41000E+00  5.00000E-03          805          335       2            3.22 

  End Simulation
 command: var/7, fun= -3.63251132769544483e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   1.41000E+00  5.00000E-03          805          335       2            3.22 
   1.41500E+00  5.00000E-03          808          336       2            3.23 

  End Simulation
 command: var/7, fun= -3.63251132769544074e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   1.41500E+00  5.00000E-03          808          336       2            3.23 
   1.42000E+00  5.00000E-03          811          337       2            3.25 

  End Simulation
 command: var/7, fun= -3.63251132769544074e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   1.42000E+00  5.00000E-03          811          337       2            3.25 
   1.42500E+00  5.00000E-03          814          338       2            3.27 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   1.42500E+00  5.00000E-03          814          338       2            3.27 
   1.43000E+00  5.00000E-03          817          339       2            3.27 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   1.43000E+00  5.00000E-03          817          339       2            3.27 
   1.43500E+00  5.00000E-03          820          340       2            3.28 

  End Simulation
 command: var/7, fun= -3.63251132769544483e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   1.43500E+00  5.00000E-03          820          340       2            3.28 
   1.44000E+00  5.00000E-03          822          341       2            3.30 

  End Simulation
 command: var/7, fun= -3.63251132769544483e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   1.44000E+00  5.00000E-03          822          341       2            3.30 
   1.44500E+00  5.00000E-03          824          342       2            3.31 

  End Simulation
 command: var/7, fun= -3.63251132769544074e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   1.44500E+00  5.00000E-03          824          342       2            3.31 
   1.45000E+00  5.00000E-03          826          343       2            3.33 

  End Simulation
 command: var/7, fun= -3.63251132769544074e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   1.45000E+00  5.00000E-03          826          343       2            3.33 
   1.45500E+00  5.00000E-03          828          344       2            3.34 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   1.45500E+00  5.00000E-03          828          344       2            3.34 
   1.46000E+00  5.00000E-03          830          345       2            3.34 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   1.46000E+00  5.00000E-03          830          345       2            3.34 
   1.46500E+00  5.00000E-03          832          346       2            3.36 

  End Simulation
 command: var/7, fun= -3.63251132769544483e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   1.46500E+00  5.00000E-03          832          346       2            3.36 
   1.47000E+00  5.00000E-03          834          347       2            3.38 

  End Simulation
 command: var/7, fun= -3.63251132769544483e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   1.47000E+00  5.00000E-03          834          347       2            3.38 
   1.47500E+00  5.00000E-03          836          348       2            3.39 

  End Simulation
 command: var/7, fun= -3.63251132769544074e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   1.47500E+00  5.00000E-03          836          348       2            3.39 
   1.48000E+00  5.00000E-03          838          349       2            3.41 

  End Simulation
 command: var/7, fun= -3.63251132769544074e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   1.48000E+00  5.00000E-03          838          349       2            3.41 
   1.48500E+00  5.00000E-03          840          350       2            3.41 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   1.48500E+00  5.00000E-03          840          350       2            3.41 
   1.49000E+00  5.00000E-03          842          351       2            3.42 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   1.49000E+00  5.00000E-03          842          351       2            3.42 
   1.49500E+00  5.00000E-03          844          352       2            3.44 

  End Simulation
 command: var/7, fun= -3.63251132769544483e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   1.49500E+00  5.00000E-03          844          352       2            3.44 
   1.50000E+00  5.00000E-03          846          353       2            3.44 

  End Simulation
 command: var/7, fun= -3.63251132769544483e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   1.50000E+00  5.00000E-03          846          353       2            3.44 
   1.50500E+00  5.00000E-03          848          354       2            3.45 

  End Simulation
 command: var/7, fun= -3.63251132769544074e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   1.50500E+00  5.00000E-03          848          354       2            3.45 
   1.51000E+00  5.00000E-03          850          355       2            3.47 

  End Simulation
 command: var/7, fun= -3.63251132769544074e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   1.51000E+00  5.00000E-03          850          355       2            3.47 
   1.51500E+00  5.00000E-03          852          356       2            3.48 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   1.51500E+00  5.00000E-03          852          356       2            3.48 
   1.52000E+00  5.00000E-03          854          357       2            3.48 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   1.52000E+00  5.00000E-03          854          357       2            3.48 
   1.52500E+00  5.00000E-03          856          358       2            3.50 

  End Simulation
 command: var/7, fun= -3.63251132769544483e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   1.52500E+00  5.00000E-03          856          358       2            3.50 
   1.53000E+00  5.00000E-03          858          359       2            3.52 

  End Simulation
 command: var/7, fun= -3.63251132769544483e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   1.53000E+00  5.00000E-03          858          359       2            3.52 
   1.53500E+00  5.00000E-03          860          360       2            3.53 

  End Simulation
 command: var/7, fun= -3.63251132769544120e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   1.53500E+00  5.00000E-03          860          360       2            3.53 
   1.54000E+00  5.00000E-03          862          361       2            3.53 

  End Simulation
 command: var/7, fun= -3.63251132769544120e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   1.54000E+00  5.00000E-03          862          361       2            3.53 
   1.54500E+00  5.00000E-03          864          362       2            3.53 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   1.54500E+00  5.00000E-03          864          362       2            3.53 
   1.55000E+00  5.00000E-03          866          363       2            3.55 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   1.55000E+00  5.00000E-03          866          363       2            3.55 
   1.55500E+00  5.00000E-03          868          364       2            3.56 

  End Simulation
 command: var/7, fun= -3.63251132769544483e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   1.55500E+00  5.00000E-03          868          364       2            3.56 
   1.56000E+00  5.00000E-03          870          365       2            3.58 

  End Simulation
 command: var/7, fun= -3.63251132769544483e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   1.56000E+00  5.00000E-03          870          365       2            3.58 
   1.56500E+00  5.00000E-03          872          366       2            3.58 

  End Simulation
 command: var/7, fun= -3.63251132769543983e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   1.56500E+00  5.00000E-03          872          366       2            3.58 
   1.57000E+00  5.00000E-03          874          367       2            3.59 

  End Simulation
 command: var/7, fun= -3.63251132769543983e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   1.57000E+00  5.00000E-03          874          367       2            3.59 
   1.57500E+00  5.00000E-03          876          368       2            3.61 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   1.57500E+00  5.00000E-03          876          368       2            3.61 
   1.58000E+00  5.00000E-03          878          369       2            3.63 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   1.58000E+00  5.00000E-03          878          369       2            3.63 
   1.58500E+00  5.00000E-03          880          370       2            3.64 

  End Simulation
 command: var/7, fun= -3.63251132769544483e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   1.58500E+00  5.00000E-03          880          370       2            3.64 
   1.59000E+00  5.00000E-03          882          371       2            3.64 

  End Simulation
 command: var/7, fun= -3.63251132769544483e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   1.59000E+00  5.00000E-03          882          371       2            3.64 
   1.59500E+00  5.00000E-03          884          372       2            3.66 

  End Simulation
 command: var/7, fun= -3.63251132769544120e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   1.59500E+00  5.00000E-03          884          372       2            3.66 
   1.60000E+00  5.00000E-03          886          373       2            3.67 

  End Simulation
 command: var/7, fun= -3.63251132769544120e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   1.60000E+00  5.00000E-03          886          373       2            3.67 
   1.60500E+00  5.00000E-03          888          374       2            3.69 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   1.60500E+00  5.00000E-03          888          374       2            3.69 
   1.61000E+00  5.00000E-03          890          375       2            3.69 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   1.61000E+00  5.00000E-03          890          375       2            3.69 
   1.61500E+00  5.00000E-03          892          376       2            3.69 

  End Simulation
 command: var/7, fun= -3.63251132769544483e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   1.61500E+00  5.00000E-03          892          376       2            3.69 
   1.62000E+00  5.00000E-03          894          377       2            3.70 

  End Simulation
 command: var/7, fun= -3.63251132769544483e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   1.62000E+00  5.00000E-03          894          377       2            3.70 
   1.62500E+00  5.00000E-03          896          378       2            3.72 

  End Simulation
 command: var/7, fun= -3.63251132769544074e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   1.62500E+00  5.00000E-03          896          378       2            3.72 
   1.63000E+00  5.00000E-03          898          379       2            3.73 

  End Simulation
 command: var/7, fun= -3.63251132769544074e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   1.63000E+00  5.00000E-03          898          379       2            3.73 
   1.63500E+00  5.00000E-03          900          380       2            3.73 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   1.63500E+00  5.00000E-03          900          380       2            3.73 
   1.64000E+00  5.00000E-03          902          381       2            3.73 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   1.64000E+00  5.00000E-03          902          381       2            3.73 
   1.64500E+00  5.00000E-03          904          382       2            3.75 

  End Simulation
 command: var/7, fun= -3.63251132769544483e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   1.64500E+00  5.00000E-03          904          382       2            3.75 
   1.65000E+00  5.00000E-03          906          383       2            3.77 

  End Simulation
 command: var/7, fun= -3.63251132769544483e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   1.65000E+00  5.00000E-03          906          383       2            3.77 
   1.65500E+00  5.00000E-03          908          384       2            3.78 

  End Simulation
 command: var/7, fun= -3.63251132769544074e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   1.65500E+00  5.00000E-03          908          384       2            3.78 
   1.66000E+00  5.00000E-03          910          385       2            3.78 

  End Simulation
 command: var/7, fun= -3.63251132769544074e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   1.66000E+00  5.00000E-03          910          385       2            3.78 
   1.66500E+00  5.00000E-03          912          386       2            3.80 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   1.66500E+00  5.00000E-03          912          386       2            3.80 
   1.67000E+00  5.00000E-03          914          387       2            3.81 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   1.67000E+00  5.00000E-03          914          387       2            3.81 
   1.67500E+00  5.00000E-03          916          388       2            3.83 

  End Simulation
 command: var/7, fun= -3.63251132769544438e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   1.67500E+00  5.00000E-03          916          388       2            3.83 
   1.68000E+00  5.00000E-03          918          389       2            3.84 

  End Simulation
 command: var/7, fun= -3.63251132769544438e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   1.68000E+00  5.00000E-03          918          389       2            3.84 
   1.68500E+00  5.00000E-03          920          390       2            3.86 

  End Simulation
 command: var/7, fun= -3.63251132769544074e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   1.68500E+00  5.00000E-03          920          390       2            3.86 
   1.69000E+00  5.00000E-03          922          391       2            3.86 

  End Simulation
 command: var/7, fun= -3.63251132769544074e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   1.69000E+00  5.00000E-03          922          391       2            3.86 
   1.69500E+00  5.00000E-03          924          392       2            3.98 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   1.69500E+00  5.00000E-03          924          392       2            3.98 
   1.70000E+00  5.00000E-03          926          393       2            3.98 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   1.70000E+00  5.00000E-03          926          393       2            3.98 
   1.70500E+00  5.00000E-03          928          394       2            3.98 

  End Simulation
 command: var/7, fun= -3.63251132769544483e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   1.70500E+00  5.00000E-03          928          394       2            3.98 
   1.71000E+00  5.00000E-03          930          395       2            4.00 

  End Simulation
 command: var/7, fun= -3.63251132769544483e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   1.71000E+00  5.00000E-03          930          395       2            4.00 
   1.71500E+00  5.00000E-03          932          396       2            4.02 

  End Simulation
 command: var/7, fun= -3.63251132769543983e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   1.71500E+00  5.00000E-03          932          396       2            4.02 
   1.72000E+00  5.00000E-03          934          397       2            4.02 

  End Simulation
 command: var/7, fun= -3.63251132769543983e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   1.72000E+00  5.00000E-03          934          397       2            4.02 
   1.72500E+00  5.00000E-03          936          398       2            4.03 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   1.72500E+00  5.00000E-03          936          398       2            4.03 
   1.73000E+00  5.00000E-03          938          399       2            4.05 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   1.73000E+00  5.00000E-03          938          399       2            4.05 
   1.73500E+00  5.00000E-03          940          400       2            4.06 

  End Simulation
 command: var/7, fun= -3.63251132769544483e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   1.73500E+00  5.00000E-03          940          400       2            4.06 
   1.74000E+00  5.00000E-03          942          401       2            4.08 

  End Simulation
 command: var/7, fun= -3.63251132769544483e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   1.74000E+00  5.00000E-03          942          401       2            4.08 
   1.74500E+00  5.00000E-03          944          402       2            4.08 

  End Simulation
 command: var/7, fun= -3.63251132769544074e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   1.74500E+00  5.00000E-03          944          402       2            4.08 
   1.75000E+00  5.00000E-03          946          403       2            4.08 

  End Simulation
 command: var/7, fun= -3.63251132769544074e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   1.75000E+00  5.00000E-03          946          403       2            4.08 
   1.75500E+00  5.00000E-03          948          404       2            4.09 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   1.75500E+00  5.00000E-03          948          404       2            4.09 
   1.76000E+00  5.00000E-03          950          405       2            4.11 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   1.76000E+00  5.00000E-03          950          405       2            4.11 
   1.76500E+00  5.00000E-03          952          406       2            4.13 

  End Simulation
 command: var/7, fun= -3.63251132769544483e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   1.76500E+00  5.00000E-03          952          406       2            4.13 
   1.77000E+00  5.00000E-03          954          407       2            4.13 

  End Simulation
 command: var/7, fun= -3.63251132769544483e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   1.77000E+00  5.00000E-03          954          407       2            4.13 
   1.77500E+00  5.00000E-03          956          408       2            4.14 

  End Simulation
 command: var/7, fun= -3.63251132769544074e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   1.77500E+00  5.00000E-03          956          408       2            4.14 
   1.78000E+00  5.00000E-03          958          409       2            4.16 

  End Simulation
 command: var/7, fun= -3.63251132769544074e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   1.78000E+00  5.00000E-03          958          409       2            4.16 
   1.78500E+00  5.00000E-03          960          410       2            4.17 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   1.78500E+00  5.00000E-03          960          410       2            4.17 
   1.79000E+00  5.00000E-03          962          411       2            4.19 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   1.79000E+00  5.00000E-03          962          411       2            4.19 
   1.79500E+00  5.00000E-03          964          412       2            4.20 

  End Simulation
 command: var/7, fun= -3.63251132769544483e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   1.79500E+00  5.00000E-03          964          412       2            4.20 
   1.80000E+00  5.00000E-03          966          413       2            4.20 

  End Simulation
 command: var/7, fun= -3.63251132769544483e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   1.80000E+00  5.00000E-03          966          413       2            4.20 
   1.80500E+00  5.00000E-03          968          414       2            4.22 

  End Simulation
 command: var/7, fun= -3.63251132769544074e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   1.80500E+00  5.00000E-03          968          414       2            4.22 
   1.81000E+00  5.00000E-03          970          415       2            4.23 

  End Simulation
 command: var/7, fun= -3.63251132769544074e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   1.81000E+00  5.00000E-03          970          415       2            4.23 
   1.81500E+00  5.00000E-03          972          416       2            4.25 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   1.81500E+00  5.00000E-03          972          416       2            4.25 
   1.82000E+00  5.00000E-03          974          417       2            4.27 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   1.82000E+00  5.00000E-03          974          417       2            4.27 
   1.82500E+00  5.00000E-03          976          418       2            4.27 

  End Simulation
 command: var/7, fun= -3.63251132769544483e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   1.82500E+00  5.00000E-03          976          418       2            4.27 
   1.83000E+00  5.00000E-03          978          419       2            4.27 

  End Simulation
 command: var/7, fun= -3.63251132769544483e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   1.83000E+00  5.00000E-03          978          419       2            4.27 
   1.83500E+00  5.00000E-03          980          420       2            4.28 

  End Simulation
 command: var/7, fun= -3.63251132769544074e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   1.83500E+00  5.00000E-03          980          420       2            4.28 
   1.84000E+00  5.00000E-03          982          421       2            4.30 

  End Simulation
 command: var/7, fun= -3.63251132769544074e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   1.84000E+00  5.00000E-03          982          421       2            4.30 
   1.84500E+00  5.00000E-03          984          422       2            4.31 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   1.84500E+00  5.00000E-03          984          422       2            4.31 
   1.85000E+00  5.00000E-03          986          423       2            4.33 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   1.85000E+00  5.00000E-03          986          423       2            4.33 
   1.85500E+00  5.00000E-03          988          424       2            4.33 

  End Simulation
 command: var/7, fun= -3.63251132769544438e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   1.85500E+00  5.00000E-03          988          424       2            4.33 
   1.86000E+00  5.00000E-03          990          425       2            4.34 

  End Simulation
 command: var/7, fun= -3.63251132769544438e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   1.86000E+00  5.00000E-03          990          425       2            4.34 
   1.86500E+00  5.00000E-03          992          426       2            4.36 

  End Simulation
 command: var/7, fun= -3.63251132769544074e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   1.86500E+00  5.00000E-03          992          426       2            4.36 
   1.87000E+00  5.00000E-03          994          427       2            4.38 

  End Simulation
 command: var/7, fun= -3.63251132769544074e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   1.87000E+00  5.00000E-03          994          427       2            4.38 
   1.87500E+00  5.00000E-03          996          428       2            4.39 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   1.87500E+00  5.00000E-03          996          428       2            4.39 
   1.88000E+00  5.00000E-03          998          429       2            4.39 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   1.88000E+00  5.00000E-03          998          429       2            4.39 
   1.88500E+00  5.00000E-03         1000          430       2            4.41 

  End Simulation
 command: var/7, fun= -3.63251132769544483e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   1.88500E+00  5.00000E-03         1000          430       2            4.41 
   1.89000E+00  5.00000E-03         1002          431       2            4.42 

  End Simulation
 command: var/7, fun= -3.63251132769544483e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   1.89000E+00  5.00000E-03         1002          431       2            4.42 
   1.89500E+00  5.00000E-03         1004          432       2            4.42 

  End Simulation
 command: var/7, fun= -3.63251132769543983e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   1.89500E+00  5.00000E-03         1004          432       2            4.42 
   1.90000E+00  5.00000E-03         1006          433       2            4.42 

  End Simulation
 command: var/7, fun= -3.63251132769543983e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   1.90000E+00  5.00000E-03         1006          433       2            4.42 
   1.90500E+00  5.00000E-03         1008          434       2            4.44 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   1.90500E+00  5.00000E-03         1008          434       2            4.44 
   1.91000E+00  5.00000E-03         1010          435       2            4.45 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   1.91000E+00  5.00000E-03         1010          435       2            4.45 
   1.91500E+00  5.00000E-03         1012          436       2            4.45 

  End Simulation
 command: var/7, fun= -3.63251132769544483e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   1.91500E+00  5.00000E-03         1012          436       2            4.45 
   1.92000E+00  5.00000E-03         1014          437       2            4.47 

  End Simulation
 command: var/7, fun= -3.63251132769544483e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   1.92000E+00  5.00000E-03         1014          437       2            4.47 
   1.92500E+00  5.00000E-03         1016          438       2            4.48 

  End Simulation
 command: var/7, fun= -3.63251132769544074e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   1.92500E+00  5.00000E-03         1016          438       2            4.48 
   1.93000E+00  5.00000E-03         1018          439       2            4.50 

  End Simulation
 command: var/7, fun= -3.63251132769544074e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   1.93000E+00  5.00000E-03         1018          439       2            4.50 
   1.93500E+00  5.00000E-03         1020          440       2            4.52 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   1.93500E+00  5.00000E-03         1020          440       2            4.52 
   1.94000E+00  5.00000E-03         1022          441       2            4.52 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   1.94000E+00  5.00000E-03         1022          441       2            4.52 
   1.94500E+00  5.00000E-03         1024          442       2            4.52 

  End Simulation
 command: var/7, fun= -3.63251132769544483e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   1.94500E+00  5.00000E-03         1024          442       2            4.52 
   1.95000E+00  5.00000E-03         1026          443       2            4.53 

  End Simulation
 command: var/7, fun= -3.63251132769544483e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   1.95000E+00  5.00000E-03         1026          443       2            4.53 
   1.95500E+00  5.00000E-03         1028          444       2            4.55 

  End Simulation
 command: var/7, fun= -3.63251132769544074e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   1.95500E+00  5.00000E-03         1028          444       2            4.55 
   1.96000E+00  5.00000E-03         1030          445       2            4.56 

  End Simulation
 command: var/7, fun= -3.63251132769544074e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   1.96000E+00  5.00000E-03         1030          445       2            4.56 
   1.96500E+00  5.00000E-03         1032          446       2            4.56 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   1.96500E+00  5.00000E-03         1032          446       2            4.56 
   1.97000E+00  5.00000E-03         1034          447       2            4.58 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   1.97000E+00  5.00000E-03         1034          447       2            4.58 
   1.97500E+00  5.00000E-03         1036          448       2            4.59 

  End Simulation
 command: var/7, fun= -3.63251132769544483e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   1.97500E+00  5.00000E-03         1036          448       2            4.59 
   1.98000E+00  5.00000E-03         1038          449       2            4.61 

  End Simulation
 command: var/7, fun= -3.63251132769544483e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   1.98000E+00  5.00000E-03         1038          449       2            4.61 
   1.98500E+00  5.00000E-03         1040          450       2            4.61 

  End Simulation
 command: var/7, fun= -3.63251132769544074e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   1.98500E+00  5.00000E-03         1040          450       2            4.61 
   1.99000E+00  5.00000E-03         1042          451       2            4.61 

  End Simulation
 command: var/7, fun= -3.63251132769544074e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   1.99000E+00  5.00000E-03         1042          451       2            4.61 
   1.99500E+00  5.00000E-03         1044          452       2            4.63 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   1.99500E+00  5.00000E-03         1044          452       2            4.63 
   2.00000E+00  5.00000E-03         1046          453       2            4.64 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   2.00000E+00  5.00000E-03         1046          453       2            4.64 
   2.00500E+00  5.00000E-03         1048          454       2            4.64 

  End Simulation
 command: var/7, fun= -3.63251132769544483e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   2.00500E+00  5.00000E-03         1048          454       2            4.64 
   2.01000E+00  5.00000E-03         1050          455       2            4.66 

  End Simulation
 command: var/7, fun= -3.63251132769544483e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   2.01000E+00  5.00000E-03         1050          455       2            4.66 
   2.01500E+00  5.00000E-03         1052          456       2            4.67 

  End Simulation
 command: var/7, fun= -3.63251132769544074e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   2.01500E+00  5.00000E-03         1052          456       2            4.67 
   2.02000E+00  5.00000E-03         1054          457       2            4.69 

  End Simulation
 command: var/7, fun= -3.63251132769544074e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   2.02000E+00  5.00000E-03         1054          457       2            4.69 
   2.02500E+00  5.00000E-03         1056          458       2            4.69 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   2.02500E+00  5.00000E-03         1056          458       2            4.69 
   2.03000E+00  5.00000E-03         1058          459       2            4.69 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   2.03000E+00  5.00000E-03         1058          459       2            4.69 
   2.03500E+00  5.00000E-03         1060          460       2            4.70 

  End Simulation
 command: var/7, fun= -3.63251132769544438e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   2.03500E+00  5.00000E-03         1060          460       2            4.70 
   2.04000E+00  5.00000E-03         1062          461       2            4.72 

  End Simulation
 command: var/7, fun= -3.63251132769544438e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   2.04000E+00  5.00000E-03         1062          461       2            4.72 
   2.04500E+00  5.00000E-03         1064          462       2            4.72 

  End Simulation
 command: var/7, fun= -3.63251132769544074e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   2.04500E+00  5.00000E-03         1064          462       2            4.72 
   2.05000E+00  5.00000E-03         1066          463       2            4.73 

  End Simulation
 command: var/7, fun= -3.63251132769544074e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   2.05000E+00  5.00000E-03         1066          463       2            4.73 
   2.05500E+00  5.00000E-03         1068          464       2            4.75 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   2.05500E+00  5.00000E-03         1068          464       2            4.75 
   2.06000E+00  5.00000E-03         1070          465       2            4.77 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   2.06000E+00  5.00000E-03         1070          465       2            4.77 
   2.06500E+00  5.00000E-03         1072          466       2            4.77 

  End Simulation
 command: var/7, fun= -3.63251132769544483e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   2.06500E+00  5.00000E-03         1072          466       2            4.77 
   2.07000E+00  5.00000E-03         1074          467       2            4.78 

  End Simulation
 command: var/7, fun= -3.63251132769544483e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   2.07000E+00  5.00000E-03         1074          467       2            4.78 
   2.07500E+00  5.00000E-03         1076          468       2            4.80 

  End Simulation
 command: var/7, fun= -3.63251132769543983e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   2.07500E+00  5.00000E-03         1076          468       2            4.80 
   2.08000E+00  5.00000E-03         1078          469       2            4.81 

  End Simulation
 command: var/7, fun= -3.63251132769543983e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   2.08000E+00  5.00000E-03         1078          469       2            4.81 
   2.08500E+00  5.00000E-03         1080          470       2            4.81 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   2.08500E+00  5.00000E-03         1080          470       2            4.81 
   2.09000E+00  5.00000E-03         1082          471       2            4.83 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   2.09000E+00  5.00000E-03         1082          471       2            4.83 
   2.09500E+00  5.00000E-03         1084          472       2            4.84 

  End Simulation
 command: var/7, fun= -3.63251132769544483e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   2.09500E+00  5.00000E-03         1084          472       2            4.84 
   2.10000E+00  5.00000E-03         1086          473       2            4.86 

  End Simulation
 command: var/7, fun= -3.63251132769544483e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   2.10000E+00  5.00000E-03         1086          473       2            4.86 
   2.10500E+00  5.00000E-03         1088          474       2            4.86 

  End Simulation
 command: var/7, fun= -3.63251132769544074e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   2.10500E+00  5.00000E-03         1088          474       2            4.86 
   2.11000E+00  5.00000E-03         1090          475       2            4.88 

  End Simulation
 command: var/7, fun= -3.63251132769544074e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   2.11000E+00  5.00000E-03         1090          475       2            4.88 
   2.11500E+00  5.00000E-03         1092          476       2            4.88 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   2.11500E+00  5.00000E-03         1092          476       2            4.88 
   2.12000E+00  5.00000E-03         1094          477       2            4.89 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   2.12000E+00  5.00000E-03         1094          477       2            4.89 
   2.12500E+00  5.00000E-03         1096          478       2            4.91 

  End Simulation
 command: var/7, fun= -3.63251132769544483e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   2.12500E+00  5.00000E-03         1096          478       2            4.91 
   2.13000E+00  5.00000E-03         1098          479       2            4.92 

  End Simulation
 command: var/7, fun= -3.63251132769544483e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   2.13000E+00  5.00000E-03         1098          479       2            4.92 
   2.13500E+00  5.00000E-03         1100          480       2            4.92 

  End Simulation
 command: var/7, fun= -3.63251132769544074e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   2.13500E+00  5.00000E-03         1100          480       2            4.92 
   2.14000E+00  5.00000E-03         1102          481       2            4.94 

  End Simulation
 command: var/7, fun= -3.63251132769544074e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   2.14000E+00  5.00000E-03         1102          481       2            4.94 
   2.14500E+00  5.00000E-03         1104          482       2            4.95 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   2.14500E+00  5.00000E-03         1104          482       2            4.95 
   2.15000E+00  5.00000E-03         1106          483       2            4.97 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   2.15000E+00  5.00000E-03         1106          483       2            4.97 
   2.15500E+00  5.00000E-03         1108          484       2            4.98 

  End Simulation
 command: var/7, fun= -3.63251132769544483e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   2.15500E+00  5.00000E-03         1108          484       2            4.98 
   2.16000E+00  5.00000E-03         1110          485       2            4.98 

  End Simulation
 command: var/7, fun= -3.63251132769544483e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   2.16000E+00  5.00000E-03         1110          485       2            4.98 
   2.16500E+00  5.00000E-03         1112          486       2            5.00 

  End Simulation
 command: var/7, fun= -3.63251132769544074e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   2.16500E+00  5.00000E-03         1112          486       2            5.00 
   2.17000E+00  5.00000E-03         1114          487       2            5.02 

  End Simulation
 command: var/7, fun= -3.63251132769544074e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   2.17000E+00  5.00000E-03         1114          487       2            5.02 
   2.17500E+00  5.00000E-03         1116          488       2            5.03 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   2.17500E+00  5.00000E-03         1116          488       2            5.03 
   2.18000E+00  5.00000E-03         1119          489       2            5.03 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   2.18000E+00  5.00000E-03         1119          489       2            5.03 
   2.18500E+00  5.00000E-03         1122          490       2            5.05 

  End Simulation
 command: var/7, fun= -3.63251132769544483e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   2.18500E+00  5.00000E-03         1122          490       2            5.05 
   2.19000E+00  5.00000E-03         1125          491       2            5.06 

  End Simulation
 command: var/7, fun= -3.63251132769544483e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   2.19000E+00  5.00000E-03         1125          491       2            5.06 
   2.19500E+00  5.00000E-03         1128          492       2            5.08 

  End Simulation
 command: var/7, fun= -3.63251132769544074e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   2.19500E+00  5.00000E-03         1128          492       2            5.08 
   2.20000E+00  5.00000E-03         1131          493       2            5.09 

  End Simulation
 command: var/7, fun= -3.63251132769544074e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   2.20000E+00  5.00000E-03         1131          493       2            5.09 
   2.20500E+00  5.00000E-03         1134          494       2            5.09 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   2.20500E+00  5.00000E-03         1134          494       2            5.09 
   2.21000E+00  5.00000E-03         1137          495       2            5.09 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   2.21000E+00  5.00000E-03         1137          495       2            5.09 
   2.21500E+00  5.00000E-03         1140          496       2            5.11 

  End Simulation
 command: var/7, fun= -3.63251132769544438e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   2.21500E+00  5.00000E-03         1140          496       2            5.11 
   2.22000E+00  5.00000E-03         1143          497       2            5.11 

  End Simulation
 command: var/7, fun= -3.63251132769544438e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   2.22000E+00  5.00000E-03         1143          497       2            5.11 
   2.22500E+00  5.00000E-03         1146          498       2            5.13 

  End Simulation
 command: var/7, fun= -3.63251132769544074e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   2.22500E+00  5.00000E-03         1146          498       2            5.13 
   2.23000E+00  5.00000E-03         1149          499       2            5.14 

  End Simulation
 command: var/7, fun= -3.63251132769544074e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   2.23000E+00  5.00000E-03         1149          499       2            5.14 
   2.23500E+00  5.00000E-03         1152          500       2            5.16 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   2.23500E+00  5.00000E-03         1152          500       2            5.16 
   2.24000E+00  5.00000E-03         1155          501       2            5.17 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   2.24000E+00  5.00000E-03         1155          501       2            5.17 
   2.24500E+00  5.00000E-03         1158          502       2            5.19 

  End Simulation
 command: var/7, fun= -3.63251132769544483e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   2.24500E+00  5.00000E-03         1158          502       2            5.19 
   2.25000E+00  5.00000E-03         1161          503       2            5.19 

  End Simulation
 command: var/7, fun= -3.63251132769544483e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   2.25000E+00  5.00000E-03         1161          503       2            5.19 
   2.25500E+00  5.00000E-03         1164          504       2            5.20 

  End Simulation
 command: var/7, fun= -3.63251132769543983e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   2.25500E+00  5.00000E-03         1164          504       2            5.20 
   2.26000E+00  5.00000E-03         1167          505       2            5.22 

  End Simulation
 command: var/7, fun= -3.63251132769543983e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   2.26000E+00  5.00000E-03         1167          505       2            5.22 
   2.26500E+00  5.00000E-03         1170          506       2            5.23 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   2.26500E+00  5.00000E-03         1170          506       2            5.23 
   2.27000E+00  5.00000E-03         1173          507       2            5.25 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   2.27000E+00  5.00000E-03         1173          507       2            5.25 
   2.27500E+00  5.00000E-03         1176          508       2            5.25 

  End Simulation
 command: var/7, fun= -3.63251132769544483e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   2.27500E+00  5.00000E-03         1176          508       2            5.25 
   2.28000E+00  5.00000E-03         1179          509       2            5.25 

  End Simulation
 command: var/7, fun= -3.63251132769544483e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   2.28000E+00  5.00000E-03         1179          509       2            5.25 
   2.28500E+00  5.00000E-03         1182          510       2            5.27 

  End Simulation
 command: var/7, fun= -3.63251132769544074e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   2.28500E+00  5.00000E-03         1182          510       2            5.27 
   2.29000E+00  5.00000E-03         1185          511       2            5.28 

  End Simulation
 command: var/7, fun= -3.63251132769544074e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   2.29000E+00  5.00000E-03         1185          511       2            5.28 
   2.29500E+00  5.00000E-03         1187          512       2            5.28 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   2.29500E+00  5.00000E-03         1187          512       2            5.28 
   2.30000E+00  5.00000E-03         1189          513       2            5.30 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   2.30000E+00  5.00000E-03         1189          513       2            5.30 
   2.30500E+00  5.00000E-03         1191          514       2            5.31 

  End Simulation
 command: var/7, fun= -3.63251132769544483e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   2.30500E+00  5.00000E-03         1191          514       2            5.31 
   2.31000E+00  5.00000E-03         1193          515       2            5.33 

  End Simulation
 command: var/7, fun= -3.63251132769544483e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   2.31000E+00  5.00000E-03         1193          515       2            5.33 
   2.31500E+00  5.00000E-03         1195          516       2            5.34 

  End Simulation
 command: var/7, fun= -3.63251132769544074e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   2.31500E+00  5.00000E-03         1195          516       2            5.34 
   2.32000E+00  5.00000E-03         1197          517       2            5.36 

  End Simulation
 command: var/7, fun= -3.63251132769544074e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   2.32000E+00  5.00000E-03         1197          517       2            5.36 
   2.32500E+00  5.00000E-03         1199          518       2            5.38 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   2.32500E+00  5.00000E-03         1199          518       2            5.38 
   2.33000E+00  5.00000E-03         1201          519       2            5.38 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   2.33000E+00  5.00000E-03         1201          519       2            5.38 
   2.33500E+00  5.00000E-03         1203          520       2            5.38 

  End Simulation
 command: var/7, fun= -3.63251132769544483e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   2.33500E+00  5.00000E-03         1203          520       2            5.38 
   2.34000E+00  5.00000E-03         1205          521       2            5.39 

  End Simulation
 command: var/7, fun= -3.63251132769544483e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   2.34000E+00  5.00000E-03         1205          521       2            5.39 
   2.34500E+00  5.00000E-03         1207          522       2            5.41 

  End Simulation
 command: var/7, fun= -3.63251132769544074e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   2.34500E+00  5.00000E-03         1207          522       2            5.41 
   2.35000E+00  5.00000E-03         1209          523       2            5.41 

  End Simulation
 command: var/7, fun= -3.63251132769544074e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   2.35000E+00  5.00000E-03         1209          523       2            5.41 
   2.35500E+00  5.00000E-03         1211          524       2            5.42 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   2.35500E+00  5.00000E-03         1211          524       2            5.42 
   2.36000E+00  5.00000E-03         1213          525       2            5.44 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   2.36000E+00  5.00000E-03         1213          525       2            5.44 
   2.36500E+00  5.00000E-03         1215          526       2            5.45 

  End Simulation
 command: var/7, fun= -3.63251132769544483e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   2.36500E+00  5.00000E-03         1215          526       2            5.45 
   2.37000E+00  5.00000E-03         1217          527       2            5.47 

  End Simulation
 command: var/7, fun= -3.63251132769544483e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   2.37000E+00  5.00000E-03         1217          527       2            5.47 
   2.37500E+00  5.00000E-03         1219          528       2            5.47 

  End Simulation
 command: var/7, fun= -3.63251132769544074e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   2.37500E+00  5.00000E-03         1219          528       2            5.47 
   2.38000E+00  5.00000E-03         1221          529       2            5.48 

  End Simulation
 command: var/7, fun= -3.63251132769544074e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   2.38000E+00  5.00000E-03         1221          529       2            5.48 
   2.38500E+00  5.00000E-03         1223          530       2            5.50 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   2.38500E+00  5.00000E-03         1223          530       2            5.50 
   2.39000E+00  5.00000E-03         1225          531       2            5.52 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   2.39000E+00  5.00000E-03         1225          531       2            5.52 
   2.39500E+00  5.00000E-03         1227          532       2            5.53 

  End Simulation
 command: var/7, fun= -3.63251132769544438e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   2.39500E+00  5.00000E-03         1227          532       2            5.53 
   2.40000E+00  5.00000E-03         1229          533       2            5.53 

  End Simulation
 command: var/7, fun= -3.63251132769544438e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   2.40000E+00  5.00000E-03         1229          533       2            5.53 
   2.40500E+00  5.00000E-03         1231          534       2            5.53 

  End Simulation
 command: var/7, fun= -3.63251132769544074e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   2.40500E+00  5.00000E-03         1231          534       2            5.53 
   2.41000E+00  5.00000E-03         1233          535       2            5.55 

  End Simulation
 command: var/7, fun= -3.63251132769544074e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   2.41000E+00  5.00000E-03         1233          535       2            5.55 
   2.41500E+00  5.00000E-03         1235          536       2            5.56 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   2.41500E+00  5.00000E-03         1235          536       2            5.56 
   2.42000E+00  5.00000E-03         1237          537       2            5.58 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   2.42000E+00  5.00000E-03         1237          537       2            5.58 
   2.42500E+00  5.00000E-03         1239          538       2            5.58 

  End Simulation
 command: var/7, fun= -3.63251132769544483e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   2.42500E+00  5.00000E-03         1239          538       2            5.58 
   2.43000E+00  5.00000E-03         1241          539       2            5.59 

  End Simulation
 command: var/7, fun= -3.63251132769544483e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   2.43000E+00  5.00000E-03         1241          539       2            5.59 
   2.43500E+00  5.00000E-03         1243          540       2            5.61 

  End Simulation
 command: var/7, fun= -3.63251132769543983e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   2.43500E+00  5.00000E-03         1243          540       2            5.61 
   2.44000E+00  5.00000E-03         1245          541       2            5.61 

  End Simulation
 command: var/7, fun= -3.63251132769543983e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   2.44000E+00  5.00000E-03         1245          541       2            5.61 
   2.44500E+00  5.00000E-03         1247          542       2            5.61 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   2.44500E+00  5.00000E-03         1247          542       2            5.61 
   2.45000E+00  5.00000E-03         1249          543       2            5.63 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   2.45000E+00  5.00000E-03         1249          543       2            5.63 
   2.45500E+00  5.00000E-03         1251          544       2            5.64 

  End Simulation
 command: var/7, fun= -3.63251132769544483e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   2.45500E+00  5.00000E-03         1251          544       2            5.64 
   2.46000E+00  5.00000E-03         1253          545       2            5.66 

  End Simulation
 command: var/7, fun= -3.63251132769544483e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   2.46000E+00  5.00000E-03         1253          545       2            5.66 
   2.46500E+00  5.00000E-03         1255          546       2            5.66 

  End Simulation
 command: var/7, fun= -3.63251132769544074e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   2.46500E+00  5.00000E-03         1255          546       2            5.66 
   2.47000E+00  5.00000E-03         1257          547       2            5.67 

  End Simulation
 command: var/7, fun= -3.63251132769544074e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   2.47000E+00  5.00000E-03         1257          547       2            5.67 
   2.47500E+00  5.00000E-03         1259          548       2            5.69 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   2.47500E+00  5.00000E-03         1259          548       2            5.69 
   2.48000E+00  5.00000E-03         1261          549       2            5.70 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   2.48000E+00  5.00000E-03         1261          549       2            5.70 
   2.48500E+00  5.00000E-03         1263          550       2            5.72 

  End Simulation
 command: var/7, fun= -3.63251132769544483e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   2.48500E+00  5.00000E-03         1263          550       2            5.72 
   2.49000E+00  5.00000E-03         1265          551       2            5.72 

  End Simulation
 command: var/7, fun= -3.63251132769544483e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   2.49000E+00  5.00000E-03         1265          551       2            5.72 
   2.49500E+00  5.00000E-03         1267          552       2            5.72 

  End Simulation
 command: var/7, fun= -3.63251132769544074e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   2.49500E+00  5.00000E-03         1267          552       2            5.72 
   2.50000E+00  5.00000E-03         1269          553       2            5.73 

  End Simulation
 command: var/7, fun= -3.63251132769544074e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   2.50000E+00  5.00000E-03         1269          553       2            5.73 
   2.50500E+00  5.00000E-03         1271          554       2            5.75 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   2.50500E+00  5.00000E-03         1271          554       2            5.75 
   2.51000E+00  5.00000E-03         1273          555       2            5.75 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   2.51000E+00  5.00000E-03         1273          555       2            5.75 
   2.51500E+00  5.00000E-03         1275          556       2            5.77 

  End Simulation
 command: var/7, fun= -3.63251132769544483e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   2.51500E+00  5.00000E-03         1275          556       2            5.77 
   2.52000E+00  5.00000E-03         1277          557       2            5.78 

  End Simulation
 command: var/7, fun= -3.63251132769544483e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   2.52000E+00  5.00000E-03         1277          557       2            5.78 
   2.52500E+00  5.00000E-03         1279          558       2            5.80 

  End Simulation
 command: var/7, fun= -3.63251132769544074e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   2.52500E+00  5.00000E-03         1279          558       2            5.80 
   2.53000E+00  5.00000E-03         1281          559       2            5.81 

  End Simulation
 command: var/7, fun= -3.63251132769544074e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   2.53000E+00  5.00000E-03         1281          559       2            5.81 
   2.53500E+00  5.00000E-03         1283          560       2            5.81 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   2.53500E+00  5.00000E-03         1283          560       2            5.81 
   2.54000E+00  5.00000E-03         1285          561       2            5.83 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   2.54000E+00  5.00000E-03         1285          561       2            5.83 
   2.54500E+00  5.00000E-03         1287          562       2            5.84 

  End Simulation
 command: var/7, fun= -3.63251132769544483e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   2.54500E+00  5.00000E-03         1287          562       2            5.84 
   2.55000E+00  5.00000E-03         1289          563       2            5.86 

  End Simulation
 command: var/7, fun= -3.63251132769544483e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   2.55000E+00  5.00000E-03         1289          563       2            5.86 
   2.55500E+00  5.00000E-03         1291          564       2            5.86 

  End Simulation
 command: var/7, fun= -3.63251132769544074e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   2.55500E+00  5.00000E-03         1291          564       2            5.86 
   2.56000E+00  5.00000E-03         1293          565       2            5.86 

  End Simulation
 command: var/7, fun= -3.63251132769544074e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   2.56000E+00  5.00000E-03         1293          565       2            5.86 
   2.56500E+00  5.00000E-03         1295          566       2            5.86 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   2.56500E+00  5.00000E-03         1295          566       2            5.86 
   2.57000E+00  5.00000E-03         1297          567       2            5.88 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   2.57000E+00  5.00000E-03         1297          567       2            5.88 
   2.57500E+00  5.00000E-03         1299          568       2            5.89 

  End Simulation
 command: var/7, fun= -3.63251132769544438e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   2.57500E+00  5.00000E-03         1299          568       2            5.89 
   2.58000E+00  5.00000E-03         1301          569       2            5.89 

  End Simulation
 command: var/7, fun= -3.63251132769544438e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   2.58000E+00  5.00000E-03         1301          569       2            5.89 
   2.58500E+00  5.00000E-03         1303          570       2            5.91 

  End Simulation
 command: var/7, fun= -3.63251132769544074e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   2.58500E+00  5.00000E-03         1303          570       2            5.91 
   2.59000E+00  5.00000E-03         1305          571       2            5.92 

  End Simulation
 command: var/7, fun= -3.63251132769544074e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   2.59000E+00  5.00000E-03         1305          571       2            5.92 
   2.59500E+00  5.00000E-03         1307          572       2            5.94 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   2.59500E+00  5.00000E-03         1307          572       2            5.94 
   2.60000E+00  5.00000E-03         1309          573       2            5.94 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   2.60000E+00  5.00000E-03         1309          573       2            5.94 
   2.60500E+00  5.00000E-03         1311          574       2            5.94 

  End Simulation
 command: var/7, fun= -3.63251132769544483e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   2.60500E+00  5.00000E-03         1311          574       2            5.94 
   2.61000E+00  5.00000E-03         1313          575       2            5.95 

  End Simulation
 command: var/7, fun= -3.63251132769544483e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   2.61000E+00  5.00000E-03         1313          575       2            5.95 
   2.61500E+00  5.00000E-03         1315          576       2            5.97 

  End Simulation
 command: var/7, fun= -3.63251132769543983e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   2.61500E+00  5.00000E-03         1315          576       2            5.97 
   2.62000E+00  5.00000E-03         1317          577       2            5.98 

  End Simulation
 command: var/7, fun= -3.63251132769543983e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   2.62000E+00  5.00000E-03         1317          577       2            5.98 
   2.62500E+00  5.00000E-03         1319          578       2            5.98 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   2.62500E+00  5.00000E-03         1319          578       2            5.98 
   2.63000E+00  5.00000E-03         1321          579       2            6.00 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   2.63000E+00  5.00000E-03         1321          579       2            6.00 
   2.63500E+00  5.00000E-03         1323          580       2            6.02 

  End Simulation
 command: var/7, fun= -3.63251132769544483e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   2.63500E+00  5.00000E-03         1323          580       2            6.02 
   2.64000E+00  5.00000E-03         1325          581       2            6.03 

  End Simulation
 command: var/7, fun= -3.63251132769544483e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   2.64000E+00  5.00000E-03         1325          581       2            6.03 
   2.64500E+00  5.00000E-03         1327          582       2            6.05 

  End Simulation
 command: var/7, fun= -3.63251132769544074e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   2.64500E+00  5.00000E-03         1327          582       2            6.05 
   2.65000E+00  5.00000E-03         1329          583       2            6.05 

  End Simulation
 command: var/7, fun= -3.63251132769544074e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   2.65000E+00  5.00000E-03         1329          583       2            6.05 
   2.65500E+00  5.00000E-03         1331          584       2            6.06 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   2.65500E+00  5.00000E-03         1331          584       2            6.06 
   2.66000E+00  5.00000E-03         1333          585       2            6.08 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   2.66000E+00  5.00000E-03         1333          585       2            6.08 
   2.66500E+00  5.00000E-03         1335          586       2            6.09 

  End Simulation
 command: var/7, fun= -3.63251132769544483e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   2.66500E+00  5.00000E-03         1335          586       2            6.09 
   2.67000E+00  5.00000E-03         1337          587       2            6.09 

  End Simulation
 command: var/7, fun= -3.63251132769544483e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   2.67000E+00  5.00000E-03         1337          587       2            6.09 
   2.67500E+00  5.00000E-03         1339          588       2            6.11 

  End Simulation
 command: var/7, fun= -3.63251132769544074e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   2.67500E+00  5.00000E-03         1339          588       2            6.11 
   2.68000E+00  5.00000E-03         1341          589       2            6.13 

  End Simulation
 command: var/7, fun= -3.63251132769544074e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   2.68000E+00  5.00000E-03         1341          589       2            6.13 
   2.68500E+00  5.00000E-03         1343          590       2            6.14 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   2.68500E+00  5.00000E-03         1343          590       2            6.14 
   2.69000E+00  5.00000E-03         1345          591       2            6.14 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   2.69000E+00  5.00000E-03         1345          591       2            6.14 
   2.69500E+00  5.00000E-03         1347          592       2            6.16 

  End Simulation
 command: var/7, fun= -3.63251132769544483e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   2.69500E+00  5.00000E-03         1347          592       2            6.16 
   2.70000E+00  5.00000E-03         1349          593       2            6.17 

  End Simulation
 command: var/7, fun= -3.63251132769544483e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   2.70000E+00  5.00000E-03         1349          593       2            6.17 
   2.70500E+00  5.00000E-03         1351          594       2            6.19 

  End Simulation
 command: var/7, fun= -3.63251132769544074e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   2.70500E+00  5.00000E-03         1351          594       2            6.19 
   2.71000E+00  5.00000E-03         1353          595       2            6.20 

  End Simulation
 command: var/7, fun= -3.63251132769544074e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   2.71000E+00  5.00000E-03         1353          595       2            6.20 
   2.71500E+00  5.00000E-03         1355          596       2            6.22 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   2.71500E+00  5.00000E-03         1355          596       2            6.22 
   2.72000E+00  5.00000E-03         1357          597       2            6.22 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   2.72000E+00  5.00000E-03         1357          597       2            6.22 
   2.72500E+00  5.00000E-03         1359          598       2            6.23 

  End Simulation
 command: var/7, fun= -3.63251132769544483e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   2.72500E+00  5.00000E-03         1359          598       2            6.23 
   2.73000E+00  5.00000E-03         1361          599       2            6.25 

  End Simulation
 command: var/7, fun= -3.63251132769544483e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   2.73000E+00  5.00000E-03         1361          599       2            6.25 
   2.73500E+00  5.00000E-03         1363          600       2            6.27 

  End Simulation
 command: var/7, fun= -3.63251132769544074e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   2.73500E+00  5.00000E-03         1363          600       2            6.27 
   2.74000E+00  5.00000E-03         1365          601       2            6.28 

  End Simulation
 command: var/7, fun= -3.63251132769544074e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   2.74000E+00  5.00000E-03         1365          601       2            6.28 
   2.74500E+00  5.00000E-03         1367          602       2            6.28 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   2.74500E+00  5.00000E-03         1367          602       2            6.28 
   2.75000E+00  5.00000E-03         1369          603       2            6.30 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   2.75000E+00  5.00000E-03         1369          603       2            6.30 
   2.75500E+00  5.00000E-03         1371          604       2            6.31 

  End Simulation
 command: var/7, fun= -3.63251132769544438e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   2.75500E+00  5.00000E-03         1371          604       2            6.31 
   2.76000E+00  5.00000E-03         1373          605       2            6.33 

  End Simulation
 command: var/7, fun= -3.63251132769544438e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   2.76000E+00  5.00000E-03         1373          605       2            6.33 
   2.76500E+00  5.00000E-03         1375          606       2            6.33 

  End Simulation
 command: var/7, fun= -3.63251132769544074e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   2.76500E+00  5.00000E-03         1375          606       2            6.33 
   2.77000E+00  5.00000E-03         1377          607       2            6.33 

  End Simulation
 command: var/7, fun= -3.63251132769544074e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   2.77000E+00  5.00000E-03         1377          607       2            6.33 
   2.77500E+00  5.00000E-03         1379          608       2            6.34 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   2.77500E+00  5.00000E-03         1379          608       2            6.34 
   2.78000E+00  5.00000E-03         1381          609       2            6.36 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   2.78000E+00  5.00000E-03         1381          609       2            6.36 
   2.78500E+00  5.00000E-03         1383          610       2            6.38 

  End Simulation
 command: var/7, fun= -3.63251132769544483e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   2.78500E+00  5.00000E-03         1383          610       2            6.38 
   2.79000E+00  5.00000E-03         1385          611       2            6.38 

  End Simulation
 command: var/7, fun= -3.63251132769544483e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   2.79000E+00  5.00000E-03         1385          611       2            6.38 
   2.79500E+00  5.00000E-03         1387          612       2            6.38 

  End Simulation
 command: var/7, fun= -3.63251132769543983e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   2.79500E+00  5.00000E-03         1387          612       2            6.38 
   2.80000E+00  5.00000E-03         1389          613       2            6.39 

  End Simulation
 command: var/7, fun= -3.63251132769543983e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   2.80000E+00  5.00000E-03         1389          613       2            6.39 
   2.80500E+00  5.00000E-03         1391          614       2            6.41 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   2.80500E+00  5.00000E-03         1391          614       2            6.41 
   2.81000E+00  5.00000E-03         1393          615       2            6.41 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   2.81000E+00  5.00000E-03         1393          615       2            6.41 
   2.81500E+00  5.00000E-03         1395          616       2            6.42 

  End Simulation
 command: var/7, fun= -3.63251132769544483e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   2.81500E+00  5.00000E-03         1395          616       2            6.42 
   2.82000E+00  5.00000E-03         1397          617       2            6.44 

  End Simulation
 command: var/7, fun= -3.63251132769544483e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   2.82000E+00  5.00000E-03         1397          617       2            6.44 
   2.82500E+00  5.00000E-03         1399          618       2            6.45 

  End Simulation
 command: var/7, fun= -3.63251132769544074e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   2.82500E+00  5.00000E-03         1399          618       2            6.45 
   2.83000E+00  5.00000E-03         1401          619       2            6.45 

  End Simulation
 command: var/7, fun= -3.63251132769544074e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   2.83000E+00  5.00000E-03         1401          619       2            6.45 
   2.83500E+00  5.00000E-03         1403          620       2            6.47 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   2.83500E+00  5.00000E-03         1403          620       2            6.47 
   2.84000E+00  5.00000E-03         1405          621       2            6.48 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   2.84000E+00  5.00000E-03         1405          621       2            6.48 
   2.84500E+00  5.00000E-03         1407          622       2            6.50 

  End Simulation
 command: var/7, fun= -3.63251132769544483e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   2.84500E+00  5.00000E-03         1407          622       2            6.50 
   2.85000E+00  5.00000E-03         1409          623       2            6.50 

  End Simulation
 command: var/7, fun= -3.63251132769544483e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   2.85000E+00  5.00000E-03         1409          623       2            6.50 
   2.85500E+00  5.00000E-03         1411          624       2            6.52 

  End Simulation
 command: var/7, fun= -3.63251132769544074e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   2.85500E+00  5.00000E-03         1411          624       2            6.52 
   2.86000E+00  5.00000E-03         1413          625       2            6.53 

  End Simulation
 command: var/7, fun= -3.63251132769544074e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   2.86000E+00  5.00000E-03         1413          625       2            6.53 
   2.86500E+00  5.00000E-03         1415          626       2            6.55 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   2.86500E+00  5.00000E-03         1415          626       2            6.55 
   2.87000E+00  5.00000E-03         1417          627       2            6.55 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   2.87000E+00  5.00000E-03         1417          627       2            6.55 
   2.87500E+00  5.00000E-03         1419          628       2            6.56 

  End Simulation
 command: var/7, fun= -3.63251132769544483e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   2.87500E+00  5.00000E-03         1419          628       2            6.56 
   2.88000E+00  5.00000E-03         1421          629       2            6.58 

  End Simulation
 command: var/7, fun= -3.63251132769544483e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   2.88000E+00  5.00000E-03         1421          629       2            6.58 
   2.88500E+00  5.00000E-03         1423          630       2            6.59 

  End Simulation
 command: var/7, fun= -3.63251132769544074e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   2.88500E+00  5.00000E-03         1423          630       2            6.59 
   2.89000E+00  5.00000E-03         1425          631       2            6.59 

  End Simulation
 command: var/7, fun= -3.63251132769544074e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   2.89000E+00  5.00000E-03         1425          631       2            6.59 
   2.89500E+00  5.00000E-03         1427          632       2            6.61 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   2.89500E+00  5.00000E-03         1427          632       2            6.61 
   2.90000E+00  5.00000E-03         1429          633       2            6.63 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   2.90000E+00  5.00000E-03         1429          633       2            6.63 
   2.90500E+00  5.00000E-03         1431          634       2            6.64 

  End Simulation
 command: var/7, fun= -3.63251132769544483e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   2.90500E+00  5.00000E-03         1431          634       2            6.64 
   2.91000E+00  5.00000E-03         1433          635       2            6.64 

  End Simulation
 command: var/7, fun= -3.63251132769544483e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   2.91000E+00  5.00000E-03         1433          635       2            6.64 
   2.91500E+00  5.00000E-03         1435          636       2            6.66 

  End Simulation
 command: var/7, fun= -3.63251132769544074e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   2.91500E+00  5.00000E-03         1435          636       2            6.66 
   2.92000E+00  5.00000E-03         1437          637       2            6.67 

  End Simulation
 command: var/7, fun= -3.63251132769544074e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   2.92000E+00  5.00000E-03         1437          637       2            6.67 
   2.92500E+00  5.00000E-03         1439          638       2            6.67 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   2.92500E+00  5.00000E-03         1439          638       2            6.67 
   2.93000E+00  5.00000E-03         1441          639       2            6.69 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   2.93000E+00  5.00000E-03         1441          639       2            6.69 
   2.93500E+00  5.00000E-03         1443          640       2            6.70 

  End Simulation
 command: var/7, fun= -3.63251132769544483e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   2.93500E+00  5.00000E-03         1443          640       2            6.70 
   2.94000E+00  5.00000E-03         1445          641       2            6.72 

  End Simulation
 command: var/7, fun= -3.63251132769544483e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   2.94000E+00  5.00000E-03         1445          641       2            6.72 
   2.94500E+00  5.00000E-03         1447          642       2            6.72 

  End Simulation
 command: var/7, fun= -3.63251132769544074e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   2.94500E+00  5.00000E-03         1447          642       2            6.72 
   2.95000E+00  5.00000E-03         1449          643       2            6.73 

  End Simulation
 command: var/7, fun= -3.63251132769544074e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   2.95000E+00  5.00000E-03         1449          643       2            6.73 
   2.95500E+00  5.00000E-03         1451          644       2            6.75 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   2.95500E+00  5.00000E-03         1451          644       2            6.75 
   2.96000E+00  5.00000E-03         1453          645       2            6.75 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   2.96000E+00  5.00000E-03         1453          645       2            6.75 
   2.96500E+00  5.00000E-03         1455          646       2            6.77 

  End Simulation
 command: var/7, fun= -3.63251132769544483e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   2.96500E+00  5.00000E-03         1455          646       2            6.77 
   2.97000E+00  5.00000E-03         1457          647       2            6.78 

  End Simulation
 command: var/7, fun= -3.63251132769544483e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   2.97000E+00  5.00000E-03         1457          647       2            6.78 
   2.97500E+00  5.00000E-03         1459          648       2            6.78 

  End Simulation
 command: var/7, fun= -3.63251132769544074e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   2.97500E+00  5.00000E-03         1459          648       2            6.78 
   2.98000E+00  5.00000E-03         1461          649       2            6.78 

  End Simulation
 command: var/7, fun= -3.63251132769544074e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   2.98000E+00  5.00000E-03         1461          649       2            6.78 
   2.98500E+00  5.00000E-03         1463          650       2            6.80 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   2.98500E+00  5.00000E-03         1463          650       2            6.80 
   2.99000E+00  5.00000E-03         1465          651       2            6.81 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   2.99000E+00  5.00000E-03         1465          651       2            6.81 
   2.99500E+00  5.00000E-03         1467          652       2            6.83 

  End Simulation
 command: var/7, fun= -3.63251132769544438e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   2.99500E+00  5.00000E-03         1467          652       2            6.83 
   3.00000E+00  5.00000E-03         1469          653       2            6.83 

  End Simulation
 command: var/7, fun= -3.63251132769544438e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   3.00000E+00  5.00000E-03         1469          653       2            6.83 
   3.00500E+00  5.00000E-03         1471          654       2            6.84 

  End Simulation
 command: var/7, fun= -3.63251132769544074e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   3.00500E+00  5.00000E-03         1471          654       2            6.84 
   3.01000E+00  5.00000E-03         1474          655       2            6.86 

  End Simulation
 command: var/7, fun= -3.63251132769544074e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   3.01000E+00  5.00000E-03         1474          655       2            6.86 
   3.01500E+00  5.00000E-03         1477          656       2            6.88 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   3.01500E+00  5.00000E-03         1477          656       2            6.88 
   3.02000E+00  5.00000E-03         1480          657       2            6.88 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   3.02000E+00  5.00000E-03         1480          657       2            6.88 
   3.02500E+00  5.00000E-03         1483          658       2            6.89 

  End Simulation
 command: var/7, fun= -3.63251132769544483e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   3.02500E+00  5.00000E-03         1483          658       2            6.89 
   3.03000E+00  5.00000E-03         1486          659       2            6.91 

  End Simulation
 command: var/7, fun= -3.63251132769544483e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   3.03000E+00  5.00000E-03         1486          659       2            6.91 
   3.03500E+00  5.00000E-03         1489          660       2            6.92 

  End Simulation
 command: var/7, fun= -3.63251132769543983e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   3.03500E+00  5.00000E-03         1489          660       2            6.92 
   3.04000E+00  5.00000E-03         1492          661       2            6.92 

  End Simulation
 command: var/7, fun= -3.63251132769543983e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   3.04000E+00  5.00000E-03         1492          661       2            6.92 
   3.04500E+00  5.00000E-03         1495          662       2            6.94 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   3.04500E+00  5.00000E-03         1495          662       2            6.94 
   3.05000E+00  5.00000E-03         1498          663       2            6.95 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   3.05000E+00  5.00000E-03         1498          663       2            6.95 
   3.05500E+00  5.00000E-03         1501          664       2            6.97 

  End Simulation
 command: var/7, fun= -3.63251132769544483e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   3.05500E+00  5.00000E-03         1501          664       2            6.97 
   3.06000E+00  5.00000E-03         1504          665       2            6.97 

  End Simulation
 command: var/7, fun= -3.63251132769544483e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   3.06000E+00  5.00000E-03         1504          665       2            6.97 
   3.06500E+00  5.00000E-03         1507          666       2            6.98 

  End Simulation
 command: var/7, fun= -3.63251132769544074e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   3.06500E+00  5.00000E-03         1507          666       2            6.98 
   3.07000E+00  5.00000E-03         1510          667       2            7.00 

  End Simulation
 command: var/7, fun= -3.63251132769544074e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   3.07000E+00  5.00000E-03         1510          667       2            7.00 
   3.07500E+00  5.00000E-03         1513          668       2            7.02 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   3.07500E+00  5.00000E-03         1513          668       2            7.02 
   3.08000E+00  5.00000E-03         1516          669       2            7.02 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   3.08000E+00  5.00000E-03         1516          669       2            7.02 
   3.08500E+00  5.00000E-03         1519          670       2            7.03 

  End Simulation
 command: var/7, fun= -3.63251132769544483e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   3.08500E+00  5.00000E-03         1519          670       2            7.03 
   3.09000E+00  5.00000E-03         1522          671       2            7.05 

  End Simulation
 command: var/7, fun= -3.63251132769544483e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   3.09000E+00  5.00000E-03         1522          671       2            7.05 
   3.09500E+00  5.00000E-03         1525          672       2            7.05 

  End Simulation
 command: var/7, fun= -3.63251132769544074e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   3.09500E+00  5.00000E-03         1525          672       2            7.05 
   3.10000E+00  5.00000E-03         1528          673       2            7.08 

  End Simulation
 command: var/7, fun= -3.63251132769544074e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   3.10000E+00  5.00000E-03         1528          673       2            7.08 
   3.10500E+00  5.00000E-03         1531          674       2            7.08 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   3.10500E+00  5.00000E-03         1531          674       2            7.08 
   3.11000E+00  5.00000E-03         1534          675       2            7.08 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   3.11000E+00  5.00000E-03         1534          675       2            7.08 
   3.11500E+00  5.00000E-03         1537          676       2            7.09 

  End Simulation
 command: var/7, fun= -3.63251132769544483e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   3.11500E+00  5.00000E-03         1537          676       2            7.09 
   3.12000E+00  5.00000E-03         1540          677       2            7.11 

  End Simulation
 command: var/7, fun= -3.63251132769544483e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   3.12000E+00  5.00000E-03         1540          677       2            7.11 
   3.12500E+00  5.00000E-03         1542          678       2            7.11 

  End Simulation
 command: var/7, fun= -3.63251132769544074e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   3.12500E+00  5.00000E-03         1542          678       2            7.11 
   3.13000E+00  5.00000E-03         1544          679       2            7.13 

  End Simulation
 command: var/7, fun= -3.63251132769544074e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   3.13000E+00  5.00000E-03         1544          679       2            7.13 
   3.13500E+00  5.00000E-03         1546          680       2            7.14 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   3.13500E+00  5.00000E-03         1546          680       2            7.14 
   3.14000E+00  5.00000E-03         1548          681       2            7.14 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   3.14000E+00  5.00000E-03         1548          681       2            7.14 
   3.14500E+00  5.00000E-03         1550          682       2            7.16 

  End Simulation
 command: var/7, fun= -3.63251132769544483e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   3.14500E+00  5.00000E-03         1550          682       2            7.16 
   3.15000E+00  5.00000E-03         1552          683       2            7.17 

  End Simulation
 command: var/7, fun= -3.63251132769544483e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   3.15000E+00  5.00000E-03         1552          683       2            7.17 
   3.15500E+00  5.00000E-03         1554          684       2            7.19 

  End Simulation
 command: var/7, fun= -3.63251132769544074e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   3.15500E+00  5.00000E-03         1554          684       2            7.19 
   3.16000E+00  5.00000E-03         1556          685       2            7.19 

  End Simulation
 command: var/7, fun= -3.63251132769544074e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   3.16000E+00  5.00000E-03         1556          685       2            7.19 
   3.16500E+00  5.00000E-03         1558          686       2            7.19 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   3.16500E+00  5.00000E-03         1558          686       2            7.19 
   3.17000E+00  5.00000E-03         1560          687       2            7.20 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   3.17000E+00  5.00000E-03         1560          687       2            7.20 
   3.17500E+00  5.00000E-03         1562          688       2            7.22 

  End Simulation
 command: var/7, fun= -3.63251132769544438e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   3.17500E+00  5.00000E-03         1562          688       2            7.22 
   3.18000E+00  5.00000E-03         1564          689       2            7.22 

  End Simulation
 command: var/7, fun= -3.63251132769544438e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   3.18000E+00  5.00000E-03         1564          689       2            7.22 
   3.18500E+00  5.00000E-03         1566          690       2            7.23 

  End Simulation
 command: var/7, fun= -3.63251132769544074e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   3.18500E+00  5.00000E-03         1566          690       2            7.23 
   3.19000E+00  5.00000E-03         1568          691       2            7.25 

  End Simulation
 command: var/7, fun= -3.63251132769544074e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   3.19000E+00  5.00000E-03         1568          691       2            7.25 
   3.19500E+00  5.00000E-03         1570          692       2            7.27 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   3.19500E+00  5.00000E-03         1570          692       2            7.27 
   3.20000E+00  5.00000E-03         1572          693       2            7.27 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   3.20000E+00  5.00000E-03         1572          693       2            7.27 
   3.20500E+00  5.00000E-03         1574          694       2            7.28 

  End Simulation
 command: var/7, fun= -3.63251132769544483e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   3.20500E+00  5.00000E-03         1574          694       2            7.28 
   3.21000E+00  5.00000E-03         1576          695       2            7.30 

  End Simulation
 command: var/7, fun= -3.63251132769544483e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   3.21000E+00  5.00000E-03         1576          695       2            7.30 
   3.21500E+00  5.00000E-03         1578          696       2            7.31 

  End Simulation
 command: var/7, fun= -3.63251132769543983e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   3.21500E+00  5.00000E-03         1578          696       2            7.31 
   3.22000E+00  5.00000E-03         1580          697       2            7.31 

  End Simulation
 command: var/7, fun= -3.63251132769543983e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   3.22000E+00  5.00000E-03         1580          697       2            7.31 
   3.22500E+00  5.00000E-03         1582          698       2            7.33 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   3.22500E+00  5.00000E-03         1582          698       2            7.33 
   3.23000E+00  5.00000E-03         1584          699       2            7.34 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   3.23000E+00  5.00000E-03         1584          699       2            7.34 
   3.23500E+00  5.00000E-03         1586          700       2            7.36 

  End Simulation
 command: var/7, fun= -3.63251132769544483e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   3.23500E+00  5.00000E-03         1586          700       2            7.36 
   3.24000E+00  5.00000E-03         1588          701       2            7.36 

  End Simulation
 command: var/7, fun= -3.63251132769544483e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   3.24000E+00  5.00000E-03         1588          701       2            7.36 
   3.24500E+00  5.00000E-03         1590          702       2            7.36 

  End Simulation
 command: var/7, fun= -3.63251132769544074e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   3.24500E+00  5.00000E-03         1590          702       2            7.36 
   3.25000E+00  5.00000E-03         1592          703       2            7.38 

  End Simulation
 command: var/7, fun= -3.63251132769544074e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   3.25000E+00  5.00000E-03         1592          703       2            7.38 
   3.25500E+00  5.00000E-03         1594          704       2            7.39 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   3.25500E+00  5.00000E-03         1594          704       2            7.39 
   3.26000E+00  5.00000E-03         1596          705       2            7.41 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   3.26000E+00  5.00000E-03         1596          705       2            7.41 
   3.26500E+00  5.00000E-03         1598          706       2            7.41 

  End Simulation
 command: var/7, fun= -3.63251132769544483e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   3.26500E+00  5.00000E-03         1598          706       2            7.41 
   3.27000E+00  5.00000E-03         1600          707       2            7.42 

  End Simulation
 command: var/7, fun= -3.63251132769544483e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   3.27000E+00  5.00000E-03         1600          707       2            7.42 
   3.27500E+00  5.00000E-03         1602          708       2            7.44 

  End Simulation
 command: var/7, fun= -3.63251132769544074e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   3.27500E+00  5.00000E-03         1602          708       2            7.44 
   3.28000E+00  5.00000E-03         1604          709       2            7.44 

  End Simulation
 command: var/7, fun= -3.63251132769544074e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   3.28000E+00  5.00000E-03         1604          709       2            7.44 
   3.28500E+00  5.00000E-03         1606          710       2            7.44 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   3.28500E+00  5.00000E-03         1606          710       2            7.44 
   3.29000E+00  5.00000E-03         1608          711       2            7.45 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   3.29000E+00  5.00000E-03         1608          711       2            7.45 
   3.29500E+00  5.00000E-03         1610          712       2            7.47 

  End Simulation
 command: var/7, fun= -3.63251132769544438e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   3.29500E+00  5.00000E-03         1610          712       2            7.47 
   3.30000E+00  5.00000E-03         1612          713       2            7.48 

  End Simulation
 command: var/7, fun= -3.63251132769544438e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   3.30000E+00  5.00000E-03         1612          713       2            7.48 
   3.30500E+00  5.00000E-03         1614          714       2            7.48 

  End Simulation
 command: var/7, fun= -3.63251132769544074e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   3.30500E+00  5.00000E-03         1614          714       2            7.48 
   3.31000E+00  5.00000E-03         1616          715       2            7.50 

  End Simulation
 command: var/7, fun= -3.63251132769544074e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   3.31000E+00  5.00000E-03         1616          715       2            7.50 
   3.31500E+00  5.00000E-03         1618          716       2            7.52 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   3.31500E+00  5.00000E-03         1618          716       2            7.52 
   3.32000E+00  5.00000E-03         1620          717       2            7.52 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   3.32000E+00  5.00000E-03         1620          717       2            7.52 
   3.32500E+00  5.00000E-03         1622          718       2            7.53 

  End Simulation
 command: var/7, fun= -3.63251132769544483e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   3.32500E+00  5.00000E-03         1622          718       2            7.53 
   3.33000E+00  5.00000E-03         1624          719       2            7.55 

  End Simulation
 command: var/7, fun= -3.63251132769544483e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   3.33000E+00  5.00000E-03         1624          719       2            7.55 
   3.33500E+00  5.00000E-03         1626          720       2            7.56 

  End Simulation
 command: var/7, fun= -3.63251132769543983e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   3.33500E+00  5.00000E-03         1626          720       2            7.56 
   3.34000E+00  5.00000E-03         1628          721       2            7.58 

  End Simulation
 command: var/7, fun= -3.63251132769543983e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   3.34000E+00  5.00000E-03         1628          721       2            7.58 
   3.34500E+00  5.00000E-03         1630          722       2            7.58 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   3.34500E+00  5.00000E-03         1630          722       2            7.58 
   3.35000E+00  5.00000E-03         1632          723       2            7.59 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   3.35000E+00  5.00000E-03         1632          723       2            7.59 
   3.35500E+00  5.00000E-03         1634          724       2            7.61 

  End Simulation
 command: var/7, fun= -3.63251132769544483e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   3.35500E+00  5.00000E-03         1634          724       2            7.61 
   3.36000E+00  5.00000E-03         1636          725       2            7.63 

  End Simulation
 command: var/7, fun= -3.63251132769544483e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   3.36000E+00  5.00000E-03         1636          725       2            7.63 
   3.36500E+00  5.00000E-03         1638          726       2            7.64 

  End Simulation
 command: var/7, fun= -3.63251132769544074e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   3.36500E+00  5.00000E-03         1638          726       2            7.64 
   3.37000E+00  5.00000E-03         1640          727       2            7.66 

  End Simulation
 command: var/7, fun= -3.63251132769544074e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   3.37000E+00  5.00000E-03         1640          727       2            7.66 
   3.37500E+00  5.00000E-03         1642          728       2            7.66 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   3.37500E+00  5.00000E-03         1642          728       2            7.66 
   3.38000E+00  5.00000E-03         1644          729       2            7.67 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   3.38000E+00  5.00000E-03         1644          729       2            7.67 
   3.38500E+00  5.00000E-03         1646          730       2            7.69 

  End Simulation
 command: var/7, fun= -3.63251132769544483e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   3.38500E+00  5.00000E-03         1646          730       2            7.69 
   3.39000E+00  5.00000E-03         1648          731       2            7.69 

  End Simulation
 command: var/7, fun= -3.63251132769544483e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   3.39000E+00  5.00000E-03         1648          731       2            7.69 
   3.39500E+00  5.00000E-03         1650          732       2            7.72 

  End Simulation
 command: var/7, fun= -3.63251132769544074e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   3.39500E+00  5.00000E-03         1650          732       2            7.72 
   3.40000E+00  5.00000E-03         1652          733       2            7.72 

  End Simulation
 command: var/7, fun= -3.63251132769544074e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   3.40000E+00  5.00000E-03         1652          733       2            7.72 
   3.40500E+00  5.00000E-03         1654          734       2            7.72 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   3.40500E+00  5.00000E-03         1654          734       2            7.72 
   3.41000E+00  5.00000E-03         1656          735       2            7.72 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   3.41000E+00  5.00000E-03         1656          735       2            7.72 
   3.41500E+00  5.00000E-03         1658          736       2            7.73 

  End Simulation
 command: var/7, fun= -3.63251132769544483e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   3.41500E+00  5.00000E-03         1658          736       2            7.73 
   3.42000E+00  5.00000E-03         1660          737       2            7.73 

  End Simulation
 command: var/7, fun= -3.63251132769544483e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   3.42000E+00  5.00000E-03         1660          737       2            7.73 
   3.42500E+00  5.00000E-03         1662          738       2            7.75 

  End Simulation
 command: var/7, fun= -3.63251132769544074e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   3.42500E+00  5.00000E-03         1662          738       2            7.75 
   3.43000E+00  5.00000E-03         1664          739       2            7.77 

  End Simulation
 command: var/7, fun= -3.63251132769544074e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   3.43000E+00  5.00000E-03         1664          739       2            7.77 
   3.43500E+00  5.00000E-03         1666          740       2            7.78 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   3.43500E+00  5.00000E-03         1666          740       2            7.78 
   3.44000E+00  5.00000E-03         1668          741       2            7.78 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   3.44000E+00  5.00000E-03         1668          741       2            7.78 
   3.44500E+00  5.00000E-03         1670          742       2            7.80 

  End Simulation
 command: var/7, fun= -3.63251132769544483e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   3.44500E+00  5.00000E-03         1670          742       2            7.80 
   3.45000E+00  5.00000E-03         1672          743       2            7.81 

  End Simulation
 command: var/7, fun= -3.63251132769544483e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   3.45000E+00  5.00000E-03         1672          743       2            7.81 
   3.45500E+00  5.00000E-03         1674          744       2            7.83 

  End Simulation
 command: var/7, fun= -3.63251132769544074e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   3.45500E+00  5.00000E-03         1674          744       2            7.83 
   3.46000E+00  5.00000E-03         1676          745       2            7.83 

  End Simulation
 command: var/7, fun= -3.63251132769544074e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   3.46000E+00  5.00000E-03         1676          745       2            7.83 
   3.46500E+00  5.00000E-03         1678          746       2            7.84 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   3.46500E+00  5.00000E-03         1678          746       2            7.84 
   3.47000E+00  5.00000E-03         1680          747       2            7.86 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   3.47000E+00  5.00000E-03         1680          747       2            7.86 
   3.47500E+00  5.00000E-03         1682          748       2            7.88 

  End Simulation
 command: var/7, fun= -3.63251132769544438e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   3.47500E+00  5.00000E-03         1682          748       2            7.88 
   3.48000E+00  5.00000E-03         1684          749       2            7.88 

  End Simulation
 command: var/7, fun= -3.63251132769544438e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   3.48000E+00  5.00000E-03         1684          749       2            7.88 
   3.48500E+00  5.00000E-03         1686          750       2            7.89 

  End Simulation
 command: var/7, fun= -3.63251132769544074e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   3.48500E+00  5.00000E-03         1686          750       2            7.89 
   3.49000E+00  5.00000E-03         1688          751       2            7.91 

  End Simulation
 command: var/7, fun= -3.63251132769544074e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   3.49000E+00  5.00000E-03         1688          751       2            7.91 
   3.49500E+00  5.00000E-03         1690          752       2            7.92 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   3.49500E+00  5.00000E-03         1690          752       2            7.92 
   3.50000E+00  5.00000E-03         1692          753       2            7.92 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   3.50000E+00  5.00000E-03         1692          753       2            7.92 
   3.50500E+00  5.00000E-03         1694          754       2            7.94 

  End Simulation
 command: var/7, fun= -3.63251132769544483e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   3.50500E+00  5.00000E-03         1694          754       2            7.94 
   3.51000E+00  5.00000E-03         1696          755       2            7.95 

  End Simulation
 command: var/7, fun= -3.63251132769544483e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   3.51000E+00  5.00000E-03         1696          755       2            7.95 
   3.51500E+00  5.00000E-03         1698          756       2            7.95 

  End Simulation
 command: var/7, fun= -3.63251132769543983e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   3.51500E+00  5.00000E-03         1698          756       2            7.95 
   3.52000E+00  5.00000E-03         1700          757       2            7.97 

  End Simulation
 command: var/7, fun= -3.63251132769543983e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   3.52000E+00  5.00000E-03         1700          757       2            7.97 
   3.52500E+00  5.00000E-03         1702          758       2            7.98 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   3.52500E+00  5.00000E-03         1702          758       2            7.98 
   3.53000E+00  5.00000E-03         1704          759       2            8.00 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   3.53000E+00  5.00000E-03         1704          759       2            8.00 
   3.53500E+00  5.00000E-03         1706          760       2            8.00 

  End Simulation
 command: var/7, fun= -3.63251132769544483e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   3.53500E+00  5.00000E-03         1706          760       2            8.00 
   3.54000E+00  5.00000E-03         1708          761       2            8.02 

  End Simulation
 command: var/7, fun= -3.63251132769544483e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   3.54000E+00  5.00000E-03         1708          761       2            8.02 
   3.54500E+00  5.00000E-03         1710          762       2            8.03 

  End Simulation
 command: var/7, fun= -3.63251132769544074e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   3.54500E+00  5.00000E-03         1710          762       2            8.03 
   3.55000E+00  5.00000E-03         1712          763       2            8.03 

  End Simulation
 command: var/7, fun= -3.63251132769544074e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   3.55000E+00  5.00000E-03         1712          763       2            8.03 
   3.55500E+00  5.00000E-03         1714          764       2            8.05 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   3.55500E+00  5.00000E-03         1714          764       2            8.05 
   3.56000E+00  5.00000E-03         1716          765       2            8.06 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   3.56000E+00  5.00000E-03         1716          765       2            8.06 
   3.56500E+00  5.00000E-03         1718          766       2            8.08 

  End Simulation
 command: var/7, fun= -3.63251132769544483e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   3.56500E+00  5.00000E-03         1718          766       2            8.08 
   3.57000E+00  5.00000E-03         1720          767       2            8.11 

  End Simulation
 command: var/7, fun= -3.63251132769544483e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   3.57000E+00  5.00000E-03         1720          767       2            8.11 
   3.57500E+00  5.00000E-03         1722          768       2            8.11 

  End Simulation
 command: var/7, fun= -3.63251132769544074e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   3.57500E+00  5.00000E-03         1722          768       2            8.11 
   3.58000E+00  5.00000E-03         1724          769       2            8.11 

  End Simulation
 command: var/7, fun= -3.63251132769544074e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   3.58000E+00  5.00000E-03         1724          769       2            8.11 
   3.58500E+00  5.00000E-03         1726          770       2            8.13 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   3.58500E+00  5.00000E-03         1726          770       2            8.13 
   3.59000E+00  5.00000E-03         1728          771       2            8.14 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   3.59000E+00  5.00000E-03         1728          771       2            8.14 
   3.59500E+00  5.00000E-03         1730          772       2            8.16 

  End Simulation
 command: var/7, fun= -3.63251132769544483e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   3.59500E+00  5.00000E-03         1730          772       2            8.16 
   3.60000E+00  5.00000E-03         1732          773       2            8.17 

  End Simulation
 command: var/7, fun= -3.63251132769544483e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   3.60000E+00  5.00000E-03         1732          773       2            8.17 
   3.60500E+00  5.00000E-03         1734          774       2            8.17 

  End Simulation
 command: var/7, fun= -3.63251132769544074e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   3.60500E+00  5.00000E-03         1734          774       2            8.17 
   3.61000E+00  5.00000E-03         1736          775       2            8.19 

  End Simulation
 command: var/7, fun= -3.63251132769544074e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   3.61000E+00  5.00000E-03         1736          775       2            8.19 
   3.61500E+00  5.00000E-03         1738          776       2            8.20 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   3.61500E+00  5.00000E-03         1738          776       2            8.20 
   3.62000E+00  5.00000E-03         1740          777       2            8.22 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   3.62000E+00  5.00000E-03         1740          777       2            8.22 
   3.62500E+00  5.00000E-03         1742          778       2            8.23 

  End Simulation
 command: var/7, fun= -3.63251132769544483e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   3.62500E+00  5.00000E-03         1742          778       2            8.23 
   3.63000E+00  5.00000E-03         1744          779       2            8.23 

  End Simulation
 command: var/7, fun= -3.63251132769544483e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   3.63000E+00  5.00000E-03         1744          779       2            8.23 
   3.63500E+00  5.00000E-03         1746          780       2            8.23 

  End Simulation
 command: var/7, fun= -3.63251132769544074e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   3.63500E+00  5.00000E-03         1746          780       2            8.23 
   3.64000E+00  5.00000E-03         1748          781       2            8.25 

  End Simulation
 command: var/7, fun= -3.63251132769544074e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   3.64000E+00  5.00000E-03         1748          781       2            8.25 
   3.64500E+00  5.00000E-03         1750          782       2            8.27 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   3.64500E+00  5.00000E-03         1750          782       2            8.27 
   3.65000E+00  5.00000E-03         1752          783       2            8.27 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   3.65000E+00  5.00000E-03         1752          783       2            8.27 
   3.65500E+00  5.00000E-03         1754          784       2            8.28 

  End Simulation
 command: var/7, fun= -3.63251132769544438e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   3.65500E+00  5.00000E-03         1754          784       2            8.28 
   3.66000E+00  5.00000E-03         1756          785       2            8.30 

  End Simulation
 command: var/7, fun= -3.63251132769544438e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   3.66000E+00  5.00000E-03         1756          785       2            8.30 
   3.66500E+00  5.00000E-03         1758          786       2            8.31 

  End Simulation
 command: var/7, fun= -3.63251132769544074e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   3.66500E+00  5.00000E-03         1758          786       2            8.31 
   3.67000E+00  5.00000E-03         1760          787       2            8.31 

  End Simulation
 command: var/7, fun= -3.63251132769544074e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   3.67000E+00  5.00000E-03         1760          787       2            8.31 
   3.67500E+00  5.00000E-03         1762          788       2            8.33 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   3.67500E+00  5.00000E-03         1762          788       2            8.33 
   3.68000E+00  5.00000E-03         1764          789       2            8.34 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   3.68000E+00  5.00000E-03         1764          789       2            8.34 
   3.68500E+00  5.00000E-03         1766          790       2            8.36 

  End Simulation
 command: var/7, fun= -3.63251132769544483e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   3.68500E+00  5.00000E-03         1766          790       2            8.36 
   3.69000E+00  5.00000E-03         1768          791       2            8.36 

  End Simulation
 command: var/7, fun= -3.63251132769544483e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   3.69000E+00  5.00000E-03         1768          791       2            8.36 
   3.69500E+00  5.00000E-03         1770          792       2            8.36 

  End Simulation
 command: var/7, fun= -3.63251132769543983e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   3.69500E+00  5.00000E-03         1770          792       2            8.36 
   3.70000E+00  5.00000E-03         1772          793       2            8.38 

  End Simulation
 command: var/7, fun= -3.63251132769543983e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   3.70000E+00  5.00000E-03         1772          793       2            8.38 
   3.70500E+00  5.00000E-03         1774          794       2            8.39 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   3.70500E+00  5.00000E-03         1774          794       2            8.39 
   3.71000E+00  5.00000E-03         1776          795       2            8.41 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   3.71000E+00  5.00000E-03         1776          795       2            8.41 
   3.71500E+00  5.00000E-03         1778          796       2            8.41 

  End Simulation
 command: var/7, fun= -3.63251132769544483e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   3.71500E+00  5.00000E-03         1778          796       2            8.41 
   3.72000E+00  5.00000E-03         1780          797       2            8.42 

  End Simulation
 command: var/7, fun= -3.63251132769544483e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   3.72000E+00  5.00000E-03         1780          797       2            8.42 
   3.72500E+00  5.00000E-03         1782          798       2            8.42 

  End Simulation
 command: var/7, fun= -3.63251132769544074e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   3.72500E+00  5.00000E-03         1782          798       2            8.42 
   3.73000E+00  5.00000E-03         1784          799       2            8.42 

  End Simulation
 command: var/7, fun= -3.63251132769544074e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   3.73000E+00  5.00000E-03         1784          799       2            8.42 
   3.73500E+00  5.00000E-03         1786          800       2            8.44 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   3.73500E+00  5.00000E-03         1786          800       2            8.44 
   3.74000E+00  5.00000E-03         1788          801       2            8.45 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   3.74000E+00  5.00000E-03         1788          801       2            8.45 
   3.74500E+00  5.00000E-03         1790          802       2            8.45 

  End Simulation
 command: var/7, fun= -3.63251132769544483e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   3.74500E+00  5.00000E-03         1790          802       2            8.45 
   3.75000E+00  5.00000E-03         1792          803       2            8.47 

  End Simulation
 command: var/7, fun= -3.63251132769544483e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   3.75000E+00  5.00000E-03         1792          803       2            8.47 
   3.75500E+00  5.00000E-03         1794          804       2            8.48 

  End Simulation
 command: var/7, fun= -3.63251132769544074e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   3.75500E+00  5.00000E-03         1794          804       2            8.48 
   3.76000E+00  5.00000E-03         1796          805       2            8.50 

  End Simulation
 command: var/7, fun= -3.63251132769544074e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   3.76000E+00  5.00000E-03         1796          805       2            8.50 
   3.76500E+00  5.00000E-03         1798          806       2            8.50 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   3.76500E+00  5.00000E-03         1798          806       2            8.50 
   3.77000E+00  5.00000E-03         1800          807       2            8.52 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   3.77000E+00  5.00000E-03         1800          807       2            8.52 
   3.77500E+00  5.00000E-03         1802          808       2            8.53 

  End Simulation
 command: var/7, fun= -3.63251132769544483e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   3.77500E+00  5.00000E-03         1802          808       2            8.53 
   3.78000E+00  5.00000E-03         1804          809       2            8.53 

  End Simulation
 command: var/7, fun= -3.63251132769544483e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   3.78000E+00  5.00000E-03         1804          809       2            8.53 
   3.78500E+00  5.00000E-03         1806          810       2            8.55 

  End Simulation
 command: var/7, fun= -3.63251132769544074e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   3.78500E+00  5.00000E-03         1806          810       2            8.55 
   3.79000E+00  5.00000E-03         1808          811       2            8.56 

  End Simulation
 command: var/7, fun= -3.63251132769544074e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   3.79000E+00  5.00000E-03         1808          811       2            8.56 
   3.79500E+00  5.00000E-03         1810          812       2            8.58 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   3.79500E+00  5.00000E-03         1810          812       2            8.58 
   3.80000E+00  5.00000E-03         1812          813       2            8.58 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   3.80000E+00  5.00000E-03         1812          813       2            8.58 
   3.80500E+00  5.00000E-03         1814          814       2            8.59 

  End Simulation
 command: var/7, fun= -3.63251132769544483e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   3.80500E+00  5.00000E-03         1814          814       2            8.59 
   3.81000E+00  5.00000E-03         1817          815       2            8.61 

  End Simulation
 command: var/7, fun= -3.63251132769544483e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   3.81000E+00  5.00000E-03         1817          815       2            8.61 
   3.81500E+00  5.00000E-03         1820          816       2            8.63 

  End Simulation
 command: var/7, fun= -3.63251132769544074e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   3.81500E+00  5.00000E-03         1820          816       2            8.63 
   3.82000E+00  5.00000E-03         1823          817       2            8.63 

  End Simulation
 command: var/7, fun= -3.63251132769544074e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   3.82000E+00  5.00000E-03         1823          817       2            8.63 
   3.82500E+00  5.00000E-03         1826          818       2            8.64 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   3.82500E+00  5.00000E-03         1826          818       2            8.64 
   3.83000E+00  5.00000E-03         1829          819       2            8.66 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   3.83000E+00  5.00000E-03         1829          819       2            8.66 
   3.83500E+00  5.00000E-03         1832          820       2            8.67 

  End Simulation
 command: var/7, fun= -3.63251132769544438e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   3.83500E+00  5.00000E-03         1832          820       2            8.67 
   3.84000E+00  5.00000E-03         1835          821       2            8.67 

  End Simulation
 command: var/7, fun= -3.63251132769544438e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   3.84000E+00  5.00000E-03         1835          821       2            8.67 
   3.84500E+00  5.00000E-03         1838          822       2            8.69 

  End Simulation
 command: var/7, fun= -3.63251132769544074e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   3.84500E+00  5.00000E-03         1838          822       2            8.69 
   3.85000E+00  5.00000E-03         1841          823       2            8.70 

  End Simulation
 command: var/7, fun= -3.63251132769544074e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   3.85000E+00  5.00000E-03         1841          823       2            8.70 
   3.85500E+00  5.00000E-03         1844          824       2            8.70 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   3.85500E+00  5.00000E-03         1844          824       2            8.70 
   3.86000E+00  5.00000E-03         1847          825       2            8.73 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   3.86000E+00  5.00000E-03         1847          825       2            8.73 
   3.86500E+00  5.00000E-03         1850          826       2            8.75 

  End Simulation
 command: var/7, fun= -3.63251132769544483e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   3.86500E+00  5.00000E-03         1850          826       2            8.75 
   3.87000E+00  5.00000E-03         1853          827       2            8.75 

  End Simulation
 command: var/7, fun= -3.63251132769544483e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   3.87000E+00  5.00000E-03         1853          827       2            8.75 
   3.87500E+00  5.00000E-03         1856          828       2            8.77 

  End Simulation
 command: var/7, fun= -3.63251132769543983e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   3.87500E+00  5.00000E-03         1856          828       2            8.77 
   3.88000E+00  5.00000E-03         1859          829       2            8.78 

  End Simulation
 command: var/7, fun= -3.63251132769543983e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   3.88000E+00  5.00000E-03         1859          829       2            8.78 
   3.88500E+00  5.00000E-03         1862          830       2            8.80 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   3.88500E+00  5.00000E-03         1862          830       2            8.80 
   3.89000E+00  5.00000E-03         1865          831       2            8.81 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   3.89000E+00  5.00000E-03         1865          831       2            8.81 
   3.89500E+00  5.00000E-03         1868          832       2            8.83 

  End Simulation
 command: var/7, fun= -3.63251132769544483e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   3.89500E+00  5.00000E-03         1868          832       2            8.83 
   3.90000E+00  5.00000E-03         1871          833       2            8.84 

  End Simulation
 command: var/7, fun= -3.63251132769544483e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   3.90000E+00  5.00000E-03         1871          833       2            8.84 
   3.90500E+00  5.00000E-03         1874          834       2            8.86 

  End Simulation
 command: var/7, fun= -3.63251132769544074e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   3.90500E+00  5.00000E-03         1874          834       2            8.86 
   3.91000E+00  5.00000E-03         1877          835       2            8.86 

  End Simulation
 command: var/7, fun= -3.63251132769544074e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   3.91000E+00  5.00000E-03         1877          835       2            8.86 
   3.91500E+00  5.00000E-03         1880          836       2            8.88 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   3.91500E+00  5.00000E-03         1880          836       2            8.88 
   3.92000E+00  5.00000E-03         1883          837       2            8.89 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   3.92000E+00  5.00000E-03         1883          837       2            8.89 
   3.92500E+00  5.00000E-03         1886          838       2            8.91 

  End Simulation
 command: var/7, fun= -3.63251132769544483e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   3.92500E+00  5.00000E-03         1886          838       2            8.91 
   3.93000E+00  5.00000E-03         1888          839       2            8.92 

  End Simulation
 command: var/7, fun= -3.63251132769544483e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   3.93000E+00  5.00000E-03         1888          839       2            8.92 
   3.93500E+00  5.00000E-03         1890          840       2            8.94 

  End Simulation
 command: var/7, fun= -3.63251132769544074e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   3.93500E+00  5.00000E-03         1890          840       2            8.94 
   3.94000E+00  5.00000E-03         1892          841       2            8.95 

  End Simulation
 command: var/7, fun= -3.63251132769544074e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   3.94000E+00  5.00000E-03         1892          841       2            8.95 
   3.94500E+00  5.00000E-03         1894          842       2            8.97 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   3.94500E+00  5.00000E-03         1894          842       2            8.97 
   3.95000E+00  5.00000E-03         1896          843       2            8.98 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   3.95000E+00  5.00000E-03         1896          843       2            8.98 
   3.95500E+00  5.00000E-03         1898          844       2            9.00 

  End Simulation
 command: var/7, fun= -3.63251132769544483e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   3.95500E+00  5.00000E-03         1898          844       2            9.00 
   3.96000E+00  5.00000E-03         1900          845       2            9.02 

  End Simulation
 command: var/7, fun= -3.63251132769544483e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   3.96000E+00  5.00000E-03         1900          845       2            9.02 
   3.96500E+00  5.00000E-03         1902          846       2            9.02 

  End Simulation
 command: var/7, fun= -3.63251132769544074e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   3.96500E+00  5.00000E-03         1902          846       2            9.02 
   3.97000E+00  5.00000E-03         1904          847       2            9.03 

  End Simulation
 command: var/7, fun= -3.63251132769544074e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   3.97000E+00  5.00000E-03         1904          847       2            9.03 
   3.97500E+00  5.00000E-03         1906          848       2            9.03 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   3.97500E+00  5.00000E-03         1906          848       2            9.03 
   3.98000E+00  5.00000E-03         1908          849       2            9.05 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   3.98000E+00  5.00000E-03         1908          849       2            9.05 
   3.98500E+00  5.00000E-03         1910          850       2            9.06 

  End Simulation
 command: var/7, fun= -3.63251132769544483e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   3.98500E+00  5.00000E-03         1910          850       2            9.06 
   3.99000E+00  5.00000E-03         1912          851       2            9.08 

  End Simulation
 command: var/7, fun= -3.63251132769544483e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   3.99000E+00  5.00000E-03         1912          851       2            9.08 
   3.99500E+00  5.00000E-03         1914          852       2            9.09 

  End Simulation
 command: var/7, fun= -3.63251132769544074e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   3.99500E+00  5.00000E-03         1914          852       2            9.09 
   4.00000E+00  5.00000E-03         1916          853       2            9.11 

  End Simulation
 command: var/7, fun= -3.63251132769544074e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   4.00000E+00  5.00000E-03         1916          853       2            9.11 
   4.00500E+00  5.00000E-03         1918          854       2            9.13 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   4.00500E+00  5.00000E-03         1918          854       2            9.13 
   4.01000E+00  5.00000E-03         1920          855       2            9.14 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   4.01000E+00  5.00000E-03         1920          855       2            9.14 
   4.01500E+00  5.00000E-03         1922          856       2            9.16 

  End Simulation
 command: var/7, fun= -3.63251132769544438e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   4.01500E+00  5.00000E-03         1922          856       2            9.16 
   4.02000E+00  5.00000E-03         1924          857       2            9.17 

  End Simulation
 command: var/7, fun= -3.63251132769544438e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   4.02000E+00  5.00000E-03         1924          857       2            9.17 
   4.02500E+00  5.00000E-03         1926          858       2            9.17 

  End Simulation
 command: var/7, fun= -3.63251132769544074e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   4.02500E+00  5.00000E-03         1926          858       2            9.17 
   4.03000E+00  5.00000E-03         1928          859       2            9.19 

  End Simulation
 command: var/7, fun= -3.63251132769544074e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   4.03000E+00  5.00000E-03         1928          859       2            9.19 
   4.03500E+00  5.00000E-03         1930          860       2            9.19 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   4.03500E+00  5.00000E-03         1930          860       2            9.19 
   4.04000E+00  5.00000E-03         1932          861       2            9.20 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   4.04000E+00  5.00000E-03         1932          861       2            9.20 
   4.04500E+00  5.00000E-03         1934          862       2            9.22 

  End Simulation
 command: var/7, fun= -3.63251132769544483e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   4.04500E+00  5.00000E-03         1934          862       2            9.22 
   4.05000E+00  5.00000E-03         1936          863       2            9.23 

  End Simulation
 command: var/7, fun= -3.63251132769544483e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   4.05000E+00  5.00000E-03         1936          863       2            9.23 
   4.05500E+00  5.00000E-03         1938          864       2            9.25 

  End Simulation
 command: var/7, fun= -3.63251132769543983e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   4.05500E+00  5.00000E-03         1938          864       2            9.25 
   4.06000E+00  5.00000E-03         1940          865       2            9.27 

  End Simulation
 command: var/7, fun= -3.63251132769543983e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   4.06000E+00  5.00000E-03         1940          865       2            9.27 
   4.06500E+00  5.00000E-03         1942          866       2            9.28 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   4.06500E+00  5.00000E-03         1942          866       2            9.28 
   4.07000E+00  5.00000E-03         1944          867       2            9.28 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   4.07000E+00  5.00000E-03         1944          867       2            9.28 
   4.07500E+00  5.00000E-03         1946          868       2            9.28 

  End Simulation
 command: var/7, fun= -3.63251132769544483e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   4.07500E+00  5.00000E-03         1946          868       2            9.28 
   4.08000E+00  5.00000E-03         1948          869       2            9.30 

  End Simulation
 command: var/7, fun= -3.63251132769544483e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   4.08000E+00  5.00000E-03         1948          869       2            9.30 
   4.08500E+00  5.00000E-03         1950          870       2            9.31 

  End Simulation
 command: var/7, fun= -3.63251132769544074e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   4.08500E+00  5.00000E-03         1950          870       2            9.31 
   4.09000E+00  5.00000E-03         1952          871       2            9.33 

  End Simulation
 command: var/7, fun= -3.63251132769544074e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   4.09000E+00  5.00000E-03         1952          871       2            9.33 
   4.09500E+00  5.00000E-03         1954          872       2            9.34 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   4.09500E+00  5.00000E-03         1954          872       2            9.34 
   4.10000E+00  5.00000E-03         1956          873       2            9.34 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   4.10000E+00  5.00000E-03         1956          873       2            9.34 
   4.10500E+00  5.00000E-03         1958          874       2            9.34 

  End Simulation
 command: var/7, fun= -3.63251132769544483e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   4.10500E+00  5.00000E-03         1958          874       2            9.34 
   4.11000E+00  5.00000E-03         1960          875       2            9.34 

  End Simulation
 command: var/7, fun= -3.63251132769544483e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   4.11000E+00  5.00000E-03         1960          875       2            9.34 
   4.11500E+00  5.00000E-03         1962          876       2            9.36 

  End Simulation
 command: var/7, fun= -3.63251132769544074e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   4.11500E+00  5.00000E-03         1962          876       2            9.36 
   4.12000E+00  5.00000E-03         1964          877       2            9.38 

  End Simulation
 command: var/7, fun= -3.63251132769544074e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   4.12000E+00  5.00000E-03         1964          877       2            9.38 
   4.12500E+00  5.00000E-03         1966          878       2            9.39 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   4.12500E+00  5.00000E-03         1966          878       2            9.39 
   4.13000E+00  5.00000E-03         1968          879       2            9.41 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   4.13000E+00  5.00000E-03         1968          879       2            9.41 
   4.13500E+00  5.00000E-03         1970          880       2            9.42 

  End Simulation
 command: var/7, fun= -3.63251132769544483e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   4.13500E+00  5.00000E-03         1970          880       2            9.42 
   4.14000E+00  5.00000E-03         1972          881       2            9.42 

  End Simulation
 command: var/7, fun= -3.63251132769544483e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   4.14000E+00  5.00000E-03         1972          881       2            9.42 
   4.14500E+00  5.00000E-03         1974          882       2            9.44 

  End Simulation
 command: var/7, fun= -3.63251132769544074e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   4.14500E+00  5.00000E-03         1974          882       2            9.44 
   4.15000E+00  5.00000E-03         1976          883       2            9.45 

  End Simulation
 command: var/7, fun= -3.63251132769544074e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   4.15000E+00  5.00000E-03         1976          883       2            9.45 
   4.15500E+00  5.00000E-03         1978          884       2            9.47 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   4.15500E+00  5.00000E-03         1978          884       2            9.47 
   4.16000E+00  5.00000E-03         1980          885       2            9.47 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   4.16000E+00  5.00000E-03         1980          885       2            9.47 
   4.16500E+00  5.00000E-03         1982          886       2            9.47 

  End Simulation
 command: var/7, fun= -3.63251132769544483e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   4.16500E+00  5.00000E-03         1982          886       2            9.47 
   4.17000E+00  5.00000E-03         1984          887       2            9.48 

  End Simulation
 command: var/7, fun= -3.63251132769544483e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   4.17000E+00  5.00000E-03         1984          887       2            9.48 
   4.17500E+00  5.00000E-03         1986          888       2            9.50 

  End Simulation
 command: var/7, fun= -3.63251132769544074e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   4.17500E+00  5.00000E-03         1986          888       2            9.50 
   4.18000E+00  5.00000E-03         1988          889       2            9.50 

  End Simulation
 command: var/7, fun= -3.63251132769544074e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   4.18000E+00  5.00000E-03         1988          889       2            9.50 
   4.18500E+00  5.00000E-03         1990          890       2            9.52 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   4.18500E+00  5.00000E-03         1990          890       2            9.52 
   4.19000E+00  5.00000E-03         1992          891       2            9.53 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   4.19000E+00  5.00000E-03         1992          891       2            9.53 
   4.19500E+00  5.00000E-03         1994          892       2            9.55 

  End Simulation
 command: var/7, fun= -3.63251132769544438e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   4.19500E+00  5.00000E-03         1994          892       2            9.55 
   4.20000E+00  5.00000E-03         1996          893       2            9.55 

  End Simulation
 command: var/7, fun= -3.63251132769544438e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   4.20000E+00  5.00000E-03         1996          893       2            9.55 
   4.20500E+00  5.00000E-03         1998          894       2            9.56 

  End Simulation
 command: var/7, fun= -3.63251132769544074e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   4.20500E+00  5.00000E-03         1998          894       2            9.56 
   4.21000E+00  5.00000E-03         2000          895       2            9.58 

  End Simulation
 command: var/7, fun= -3.63251132769544074e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   4.21000E+00  5.00000E-03         2000          895       2            9.58 
   4.21500E+00  5.00000E-03         2002          896       2            9.58 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   4.21500E+00  5.00000E-03         2002          896       2            9.58 
   4.22000E+00  5.00000E-03         2004          897       2            9.59 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   4.22000E+00  5.00000E-03         2004          897       2            9.59 
   4.22500E+00  5.00000E-03         2006          898       2            9.61 

  End Simulation
 command: var/7, fun= -3.63251132769544483e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   4.22500E+00  5.00000E-03         2006          898       2            9.61 
   4.23000E+00  5.00000E-03         2008          899       2            9.63 

  End Simulation
 command: var/7, fun= -3.63251132769544483e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   4.23000E+00  5.00000E-03         2008          899       2            9.63 
   4.23500E+00  5.00000E-03         2010          900       2            9.63 

  End Simulation
 command: var/7, fun= -3.63251132769543983e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   4.23500E+00  5.00000E-03         2010          900       2            9.63 
   4.24000E+00  5.00000E-03         2012          901       2            9.64 

  End Simulation
 command: var/7, fun= -3.63251132769543983e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   4.24000E+00  5.00000E-03         2012          901       2            9.64 
   4.24500E+00  5.00000E-03         2014          902       2            9.66 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   4.24500E+00  5.00000E-03         2014          902       2            9.66 
   4.25000E+00  5.00000E-03         2016          903       2            9.67 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   4.25000E+00  5.00000E-03         2016          903       2            9.67 
   4.25500E+00  5.00000E-03         2018          904       2            9.67 

  End Simulation
 command: var/7, fun= -3.63251132769544483e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   4.25500E+00  5.00000E-03         2018          904       2            9.67 
   4.26000E+00  5.00000E-03         2020          905       2            9.69 

  End Simulation
 command: var/7, fun= -3.63251132769544483e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   4.26000E+00  5.00000E-03         2020          905       2            9.69 
   4.26500E+00  5.00000E-03         2022          906       2            9.70 

  End Simulation
 command: var/7, fun= -3.63251132769544074e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   4.26500E+00  5.00000E-03         2022          906       2            9.70 
   4.27000E+00  5.00000E-03         2024          907       2            9.70 

  End Simulation
 command: var/7, fun= -3.63251132769544074e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   4.27000E+00  5.00000E-03         2024          907       2            9.70 
   4.27500E+00  5.00000E-03         2026          908       2            9.70 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   4.27500E+00  5.00000E-03         2026          908       2            9.70 
   4.28000E+00  5.00000E-03         2028          909       2            9.72 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   4.28000E+00  5.00000E-03         2028          909       2            9.72 
   4.28500E+00  5.00000E-03         2030          910       2            9.72 

  End Simulation
 command: var/7, fun= -3.63251132769544483e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   4.28500E+00  5.00000E-03         2030          910       2            9.72 
   4.29000E+00  5.00000E-03         2032          911       2            9.73 

  End Simulation
 command: var/7, fun= -3.63251132769544483e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   4.29000E+00  5.00000E-03         2032          911       2            9.73 
   4.29500E+00  5.00000E-03         2034          912       2            9.75 

  End Simulation
 command: var/7, fun= -3.63251132769544074e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   4.29500E+00  5.00000E-03         2034          912       2            9.75 
   4.30000E+00  5.00000E-03         2036          913       2            9.77 

  End Simulation
 command: var/7, fun= -3.63251132769544074e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   4.30000E+00  5.00000E-03         2036          913       2            9.77 
   4.30500E+00  5.00000E-03         2038          914       2            9.77 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   4.30500E+00  5.00000E-03         2038          914       2            9.77 
   4.31000E+00  5.00000E-03         2040          915       2            9.78 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   4.31000E+00  5.00000E-03         2040          915       2            9.78 
   4.31500E+00  5.00000E-03         2042          916       2            9.80 

  End Simulation
 command: var/7, fun= -3.63251132769544483e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   4.31500E+00  5.00000E-03         2042          916       2            9.80 
   4.32000E+00  5.00000E-03         2044          917       2            9.80 

  End Simulation
 command: var/7, fun= -3.63251132769544483e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   4.32000E+00  5.00000E-03         2044          917       2            9.80 
   4.32500E+00  5.00000E-03         2046          918       2            9.81 

  End Simulation
 command: var/7, fun= -3.63251132769544074e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   4.32500E+00  5.00000E-03         2046          918       2            9.81 
   4.33000E+00  5.00000E-03         2048          919       2            9.83 

  End Simulation
 command: var/7, fun= -3.63251132769544074e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   4.33000E+00  5.00000E-03         2048          919       2            9.83 
   4.33500E+00  5.00000E-03         2050          920       2            9.84 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   4.33500E+00  5.00000E-03         2050          920       2            9.84 
   4.34000E+00  5.00000E-03         2052          921       2            9.84 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   4.34000E+00  5.00000E-03         2052          921       2            9.84 
   4.34500E+00  5.00000E-03         2054          922       2            9.86 

  End Simulation
 command: var/7, fun= -3.63251132769544483e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   4.34500E+00  5.00000E-03         2054          922       2            9.86 
   4.35000E+00  5.00000E-03         2056          923       2            9.88 

  End Simulation
 command: var/7, fun= -3.63251132769544483e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   4.35000E+00  5.00000E-03         2056          923       2            9.88 
   4.35500E+00  5.00000E-03         2058          924       2            9.89 

  End Simulation
 command: var/7, fun= -3.63251132769544074e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   4.35500E+00  5.00000E-03         2058          924       2            9.89 
   4.36000E+00  5.00000E-03         2060          925       2            9.89 

  End Simulation
 command: var/7, fun= -3.63251132769544074e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   4.36000E+00  5.00000E-03         2060          925       2            9.89 
   4.36500E+00  5.00000E-03         2062          926       2            9.91 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   4.36500E+00  5.00000E-03         2062          926       2            9.91 
   4.37000E+00  5.00000E-03         2064          927       2            9.92 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   4.37000E+00  5.00000E-03         2064          927       2            9.92 
   4.37500E+00  5.00000E-03         2066          928       2            9.94 

  End Simulation
 command: var/7, fun= -3.63251132769544438e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   4.37500E+00  5.00000E-03         2066          928       2            9.94 
   4.38000E+00  5.00000E-03         2068          929       2            9.94 

  End Simulation
 command: var/7, fun= -3.63251132769544438e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   4.38000E+00  5.00000E-03         2068          929       2            9.94 
   4.38500E+00  5.00000E-03         2070          930       2            9.95 

  End Simulation
 command: var/7, fun= -3.63251132769544074e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   4.38500E+00  5.00000E-03         2070          930       2            9.95 
   4.39000E+00  5.00000E-03         2072          931       2            9.97 

  End Simulation
 command: var/7, fun= -3.63251132769544074e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   4.39000E+00  5.00000E-03         2072          931       2            9.97 
   4.39500E+00  5.00000E-03         2074          932       2            9.97 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   4.39500E+00  5.00000E-03         2074          932       2            9.97 
   4.40000E+00  5.00000E-03         2076          933       2            9.98 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   4.40000E+00  5.00000E-03         2076          933       2            9.98 
   4.40500E+00  5.00000E-03         2078          934       2           10.00 

  End Simulation
 command: var/7, fun= -3.63251132769544483e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   4.40500E+00  5.00000E-03         2078          934       2           10.00 
   4.41000E+00  5.00000E-03         2080          935       2           10.02 

  End Simulation
 command: var/7, fun= -3.63251132769544483e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   4.41000E+00  5.00000E-03         2080          935       2           10.02 
   4.41500E+00  5.00000E-03         2082          936       2           10.02 

  End Simulation
 command: var/7, fun= -3.63251132769543983e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   4.41500E+00  5.00000E-03         2082          936       2           10.02 
   4.42000E+00  5.00000E-03         2084          937       2           10.03 

  End Simulation
 command: var/7, fun= -3.63251132769543983e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   4.42000E+00  5.00000E-03         2084          937       2           10.03 
   4.42500E+00  5.00000E-03         2086          938       2           10.05 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   4.42500E+00  5.00000E-03         2086          938       2           10.05 
   4.43000E+00  5.00000E-03         2088          939       2           10.06 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   4.43000E+00  5.00000E-03         2088          939       2           10.06 
   4.43500E+00  5.00000E-03         2090          940       2           10.06 

  End Simulation
 command: var/7, fun= -3.63251132769544483e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   4.43500E+00  5.00000E-03         2090          940       2           10.06 
   4.44000E+00  5.00000E-03         2092          941       2           10.08 

  End Simulation
 command: var/7, fun= -3.63251132769544483e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   4.44000E+00  5.00000E-03         2092          941       2           10.08 
   4.44500E+00  5.00000E-03         2094          942       2           10.09 

  End Simulation
 command: var/7, fun= -3.63251132769544074e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   4.44500E+00  5.00000E-03         2094          942       2           10.09 
   4.45000E+00  5.00000E-03         2096          943       2           10.11 

  End Simulation
 command: var/7, fun= -3.63251132769544074e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   4.45000E+00  5.00000E-03         2096          943       2           10.11 
   4.45500E+00  5.00000E-03         2098          944       2           10.11 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   4.45500E+00  5.00000E-03         2098          944       2           10.11 
   4.46000E+00  5.00000E-03         2100          945       2           10.13 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   4.46000E+00  5.00000E-03         2100          945       2           10.13 
   4.46500E+00  5.00000E-03         2102          946       2           10.14 

  End Simulation
 command: var/7, fun= -3.63251132769544483e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   4.46500E+00  5.00000E-03         2102          946       2           10.14 
   4.47000E+00  5.00000E-03         2104          947       2           10.14 

  End Simulation
 command: var/7, fun= -3.63251132769544483e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   4.47000E+00  5.00000E-03         2104          947       2           10.14 
   4.47500E+00  5.00000E-03         2106          948       2           10.14 

  End Simulation
 command: var/7, fun= -3.63251132769544074e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   4.47500E+00  5.00000E-03         2106          948       2           10.14 
   4.48000E+00  5.00000E-03         2108          949       2           10.16 

  End Simulation
 command: var/7, fun= -3.63251132769544074e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   4.48000E+00  5.00000E-03         2108          949       2           10.16 
   4.48500E+00  5.00000E-03         2110          950       2           10.17 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   4.48500E+00  5.00000E-03         2110          950       2           10.17 
   4.49000E+00  5.00000E-03         2112          951       2           10.19 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   4.49000E+00  5.00000E-03         2112          951       2           10.19 
   4.49500E+00  5.00000E-03         2114          952       2           10.19 

  End Simulation
 command: var/7, fun= -3.63251132769544483e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   4.49500E+00  5.00000E-03         2114          952       2           10.19 
   4.50000E+00  5.00000E-03         2116          953       2           10.20 

  End Simulation
 command: var/7, fun= -3.63251132769544483e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   4.50000E+00  5.00000E-03         2116          953       2           10.20 
   4.50500E+00  5.00000E-03         2118          954       2           10.22 

  End Simulation
 command: var/7, fun= -3.63251132769544074e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   4.50500E+00  5.00000E-03         2118          954       2           10.22 
   4.51000E+00  5.00000E-03         2120          955       2           10.23 

  End Simulation
 command: var/7, fun= -3.63251132769544074e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   4.51000E+00  5.00000E-03         2120          955       2           10.23 
   4.51500E+00  5.00000E-03         2122          956       2           10.23 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   4.51500E+00  5.00000E-03         2122          956       2           10.23 
   4.52000E+00  5.00000E-03         2124          957       2           10.25 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   4.52000E+00  5.00000E-03         2124          957       2           10.25 
   4.52500E+00  5.00000E-03         2126          958       2           10.27 

  End Simulation
 command: var/7, fun= -3.63251132769544483e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   4.52500E+00  5.00000E-03         2126          958       2           10.27 
   4.53000E+00  5.00000E-03         2128          959       2           10.28 

  End Simulation
 command: var/7, fun= -3.63251132769544483e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   4.53000E+00  5.00000E-03         2128          959       2           10.28 
   4.53500E+00  5.00000E-03         2130          960       2           10.28 

  End Simulation
 command: var/7, fun= -3.63251132769544074e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   4.53500E+00  5.00000E-03         2130          960       2           10.28 
   4.54000E+00  5.00000E-03         2132          961       2           10.30 

  End Simulation
 command: var/7, fun= -3.63251132769544074e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   4.54000E+00  5.00000E-03         2132          961       2           10.30 
   4.54500E+00  5.00000E-03         2134          962       2           10.31 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   4.54500E+00  5.00000E-03         2134          962       2           10.31 
   4.55000E+00  5.00000E-03         2136          963       2           10.31 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   4.55000E+00  5.00000E-03         2136          963       2           10.31 
   4.55500E+00  5.00000E-03         2138          964       2           10.33 

  End Simulation
 command: var/7, fun= -3.63251132769544483e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   4.55500E+00  5.00000E-03         2138          964       2           10.33 
   4.56000E+00  5.00000E-03         2140          965       2           10.34 

  End Simulation
 command: var/7, fun= -3.63251132769544483e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   4.56000E+00  5.00000E-03         2140          965       2           10.34 
   4.56500E+00  5.00000E-03         2142          966       2           10.36 

  End Simulation
 command: var/7, fun= -3.63251132769544074e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   4.56500E+00  5.00000E-03         2142          966       2           10.36 
   4.57000E+00  5.00000E-03         2144          967       2           10.36 

  End Simulation
 command: var/7, fun= -3.63251132769544074e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   4.57000E+00  5.00000E-03         2144          967       2           10.36 
   4.57500E+00  5.00000E-03         2146          968       2           10.38 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   4.57500E+00  5.00000E-03         2146          968       2           10.38 
   4.58000E+00  5.00000E-03         2148          969       2           10.39 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   4.58000E+00  5.00000E-03         2148          969       2           10.39 
   4.58500E+00  5.00000E-03         2150          970       2           10.41 

  End Simulation
 command: var/7, fun= -3.63251132769544483e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   4.58500E+00  5.00000E-03         2150          970       2           10.41 
   4.59000E+00  5.00000E-03         2152          971       2           10.41 

  End Simulation
 command: var/7, fun= -3.63251132769544483e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   4.59000E+00  5.00000E-03         2152          971       2           10.41 
   4.59500E+00  5.00000E-03         2154          972       2           10.42 

  End Simulation
 command: var/7, fun= -3.63251132769544074e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   4.59500E+00  5.00000E-03         2154          972       2           10.42 
   4.60000E+00  5.00000E-03         2157          973       2           10.44 

  End Simulation
 command: var/7, fun= -3.63251132769544074e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   4.60000E+00  5.00000E-03         2157          973       2           10.44 
   4.60500E+00  5.00000E-03         2160          974       2           10.44 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   4.60500E+00  5.00000E-03         2160          974       2           10.44 
   4.61000E+00  5.00000E-03         2163          975       2           10.44 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   4.61000E+00  5.00000E-03         2163          975       2           10.44 
   4.61500E+00  5.00000E-03         2166          976       2           10.45 

  End Simulation
 command: var/7, fun= -3.63251132769544438e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   4.61500E+00  5.00000E-03         2166          976       2           10.45 
   4.62000E+00  5.00000E-03         2169          977       2           10.47 

  End Simulation
 command: var/7, fun= -3.63251132769544438e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   4.62000E+00  5.00000E-03         2169          977       2           10.47 
   4.62500E+00  5.00000E-03         2172          978       2           10.47 

  End Simulation
 command: var/7, fun= -3.63251132769544074e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   4.62500E+00  5.00000E-03         2172          978       2           10.47 
   4.63000E+00  5.00000E-03         2175          979       2           10.48 

  End Simulation
 command: var/7, fun= -3.63251132769544074e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   4.63000E+00  5.00000E-03         2175          979       2           10.48 
   4.63500E+00  5.00000E-03         2178          980       2           10.50 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   4.63500E+00  5.00000E-03         2178          980       2           10.50 
   4.64000E+00  5.00000E-03         2181          981       2           10.50 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   4.64000E+00  5.00000E-03         2181          981       2           10.50 
   4.64500E+00  5.00000E-03         2184          982       2           10.52 

  End Simulation
 command: var/7, fun= -3.63251132769544483e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   4.64500E+00  5.00000E-03         2184          982       2           10.52 
   4.65000E+00  5.00000E-03         2187          983       2           10.53 

  End Simulation
 command: var/7, fun= -3.63251132769544483e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   4.65000E+00  5.00000E-03         2187          983       2           10.53 
   4.65500E+00  5.00000E-03         2190          984       2           10.55 

  End Simulation
 command: var/7, fun= -3.63251132769543983e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   4.65500E+00  5.00000E-03         2190          984       2           10.55 
   4.66000E+00  5.00000E-03         2193          985       2           10.55 

  End Simulation
 command: var/7, fun= -3.63251132769543983e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   4.66000E+00  5.00000E-03         2193          985       2           10.55 
   4.66500E+00  5.00000E-03         2196          986       2           10.56 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   4.66500E+00  5.00000E-03         2196          986       2           10.56 
   4.67000E+00  5.00000E-03         2199          987       2           10.58 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   4.67000E+00  5.00000E-03         2199          987       2           10.58 
   4.67500E+00  5.00000E-03         2202          988       2           10.58 

  End Simulation
 command: var/7, fun= -3.63251132769544483e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   4.67500E+00  5.00000E-03         2202          988       2           10.58 
   4.68000E+00  5.00000E-03         2205          989       2           10.59 

  End Simulation
 command: var/7, fun= -3.63251132769544483e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   4.68000E+00  5.00000E-03         2205          989       2           10.59 
   4.68500E+00  5.00000E-03         2208          990       2           10.61 

  End Simulation
 command: var/7, fun= -3.63251132769544074e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   4.68500E+00  5.00000E-03         2208          990       2           10.61 
   4.69000E+00  5.00000E-03         2211          991       2           10.61 

  End Simulation
 command: var/7, fun= -3.63251132769544074e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   4.69000E+00  5.00000E-03         2211          991       2           10.61 
   4.69500E+00  5.00000E-03         2214          992       2           10.61 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   4.69500E+00  5.00000E-03         2214          992       2           10.61 
   4.70000E+00  5.00000E-03         2217          993       2           10.63 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   4.70000E+00  5.00000E-03         2217          993       2           10.63 
   4.70500E+00  5.00000E-03         2220          994       2           10.64 

  End Simulation
 command: var/7, fun= -3.63251132769544483e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   4.70500E+00  5.00000E-03         2220          994       2           10.64 
   4.71000E+00  5.00000E-03         2223          995       2           10.64 

  End Simulation
 command: var/7, fun= -3.63251132769544483e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   4.71000E+00  5.00000E-03         2223          995       2           10.64 
   4.71500E+00  5.00000E-03         2225          996       2           10.66 

  End Simulation
 command: var/7, fun= -3.63251132769544074e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   4.71500E+00  5.00000E-03         2225          996       2           10.66 
   4.72000E+00  5.00000E-03         2227          997       2           10.67 

  End Simulation
 command: var/7, fun= -3.63251132769544074e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   4.72000E+00  5.00000E-03         2227          997       2           10.67 
   4.72500E+00  5.00000E-03         2229          998       2           10.67 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   4.72500E+00  5.00000E-03         2229          998       2           10.67 
   4.73000E+00  5.00000E-03         2231          999       2           10.69 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   4.73000E+00  5.00000E-03         2231          999       2           10.69 
   4.73500E+00  5.00000E-03         2233         1000       2           10.70 

  End Simulation
 command: var/7, fun= -3.63251132769544483e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   4.73500E+00  5.00000E-03         2233         1000       2           10.70 
   4.74000E+00  5.00000E-03         2235         1001       2           10.70 

  End Simulation
 command: var/7, fun= -3.63251132769544483e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   4.74000E+00  5.00000E-03         2235         1001       2           10.70 
   4.74500E+00  5.00000E-03         2237         1002       2           10.72 

  End Simulation
 command: var/7, fun= -3.63251132769544074e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   4.74500E+00  5.00000E-03         2237         1002       2           10.72 
   4.75000E+00  5.00000E-03         2239         1003       2           10.73 

  End Simulation
 command: var/7, fun= -3.63251132769544074e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   4.75000E+00  5.00000E-03         2239         1003       2           10.73 
   4.75500E+00  5.00000E-03         2241         1004       2           10.75 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   4.75500E+00  5.00000E-03         2241         1004       2           10.75 
   4.76000E+00  5.00000E-03         2243         1005       2           10.75 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   4.76000E+00  5.00000E-03         2243         1005       2           10.75 
   4.76500E+00  5.00000E-03         2245         1006       2           10.77 

  End Simulation
 command: var/7, fun= -3.63251132769544483e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   4.76500E+00  5.00000E-03         2245         1006       2           10.77 
   4.77000E+00  5.00000E-03         2247         1007       2           10.78 

  End Simulation
 command: var/7, fun= -3.63251132769544483e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   4.77000E+00  5.00000E-03         2247         1007       2           10.78 
   4.77500E+00  5.00000E-03         2249         1008       2           10.78 

  End Simulation
 command: var/7, fun= -3.63251132769544074e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   4.77500E+00  5.00000E-03         2249         1008       2           10.78 
   4.78000E+00  5.00000E-03         2251         1009       2           10.80 

  End Simulation
 command: var/7, fun= -3.63251132769544074e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   4.78000E+00  5.00000E-03         2251         1009       2           10.80 
   4.78500E+00  5.00000E-03         2253         1010       2           10.80 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   4.78500E+00  5.00000E-03         2253         1010       2           10.80 
   4.79000E+00  5.00000E-03         2255         1011       2           10.81 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   4.79000E+00  5.00000E-03         2255         1011       2           10.81 
   4.79500E+00  5.00000E-03         2257         1012       2           10.83 

  End Simulation
 command: var/7, fun= -3.63251132769544438e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   4.79500E+00  5.00000E-03         2257         1012       2           10.83 
   4.80000E+00  5.00000E-03         2259         1013       2           10.83 

  End Simulation
 command: var/7, fun= -3.63251132769544438e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   4.80000E+00  5.00000E-03         2259         1013       2           10.83 
   4.80500E+00  5.00000E-03         2261         1014       2           10.84 

  End Simulation
 command: var/7, fun= -3.63251132769544074e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   4.80500E+00  5.00000E-03         2261         1014       2           10.84 
   4.81000E+00  5.00000E-03         2263         1015       2           10.86 

  End Simulation
 command: var/7, fun= -3.63251132769544074e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   4.81000E+00  5.00000E-03         2263         1015       2           10.86 
   4.81500E+00  5.00000E-03         2265         1016       2           10.88 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   4.81500E+00  5.00000E-03         2265         1016       2           10.88 
   4.82000E+00  5.00000E-03         2267         1017       2           10.88 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   4.82000E+00  5.00000E-03         2267         1017       2           10.88 
   4.82500E+00  5.00000E-03         2269         1018       2           10.89 

  End Simulation
 command: var/7, fun= -3.63251132769544483e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   4.82500E+00  5.00000E-03         2269         1018       2           10.89 
   4.83000E+00  5.00000E-03         2271         1019       2           10.91 

  End Simulation
 command: var/7, fun= -3.63251132769544483e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   4.83000E+00  5.00000E-03         2271         1019       2           10.91 
   4.83500E+00  5.00000E-03         2273         1020       2           10.91 

  End Simulation
 command: var/7, fun= -3.63251132769543983e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   4.83500E+00  5.00000E-03         2273         1020       2           10.91 
   4.84000E+00  5.00000E-03         2275         1021       2           10.92 

  End Simulation
 command: var/7, fun= -3.63251132769543983e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   4.84000E+00  5.00000E-03         2275         1021       2           10.92 
   4.84500E+00  5.00000E-03         2277         1022       2           10.94 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   4.84500E+00  5.00000E-03         2277         1022       2           10.94 
   4.85000E+00  5.00000E-03         2279         1023       2           10.95 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   4.85000E+00  5.00000E-03         2279         1023       2           10.95 
   4.85500E+00  5.00000E-03         2281         1024       2           10.95 

  End Simulation
 command: var/7, fun= -3.63251132769544483e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   4.85500E+00  5.00000E-03         2281         1024       2           10.95 
   4.86000E+00  5.00000E-03         2283         1025       2           10.97 

  End Simulation
 command: var/7, fun= -3.63251132769544483e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   4.86000E+00  5.00000E-03         2283         1025       2           10.97 
   4.86500E+00  5.00000E-03         2285         1026       2           10.98 

  End Simulation
 command: var/7, fun= -3.63251132769544074e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   4.86500E+00  5.00000E-03         2285         1026       2           10.98 
   4.87000E+00  5.00000E-03         2287         1027       2           11.00 

  End Simulation
 command: var/7, fun= -3.63251132769544074e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   4.87000E+00  5.00000E-03         2287         1027       2           11.00 
   4.87500E+00  5.00000E-03         2289         1028       2           11.00 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   4.87500E+00  5.00000E-03         2289         1028       2           11.00 
   4.88000E+00  5.00000E-03         2291         1029       2           11.02 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   4.88000E+00  5.00000E-03         2291         1029       2           11.02 
   4.88500E+00  5.00000E-03         2293         1030       2           11.03 

  End Simulation
 command: var/7, fun= -3.63251132769544483e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   4.88500E+00  5.00000E-03         2293         1030       2           11.03 
   4.89000E+00  5.00000E-03         2295         1031       2           11.03 

  End Simulation
 command: var/7, fun= -3.63251132769544483e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   4.89000E+00  5.00000E-03         2295         1031       2           11.03 
   4.89500E+00  5.00000E-03         2297         1032       2           11.05 

  End Simulation
 command: var/7, fun= -3.63251132769544074e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   4.89500E+00  5.00000E-03         2297         1032       2           11.05 
   4.90000E+00  5.00000E-03         2299         1033       2           11.06 

  End Simulation
 command: var/7, fun= -3.63251132769544074e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   4.90000E+00  5.00000E-03         2299         1033       2           11.06 
   4.90500E+00  5.00000E-03         2301         1034       2           11.06 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   4.90500E+00  5.00000E-03         2301         1034       2           11.06 
   4.91000E+00  5.00000E-03         2303         1035       2           11.08 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   4.91000E+00  5.00000E-03         2303         1035       2           11.08 
   4.91500E+00  5.00000E-03         2305         1036       2           11.09 

  End Simulation
 command: var/7, fun= -3.63251132769544483e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   4.91500E+00  5.00000E-03         2305         1036       2           11.09 
   4.92000E+00  5.00000E-03         2307         1037       2           11.09 

  End Simulation
 command: var/7, fun= -3.63251132769544483e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   4.92000E+00  5.00000E-03         2307         1037       2           11.09 
   4.92500E+00  5.00000E-03         2309         1038       2           11.09 

  End Simulation
 command: var/7, fun= -3.63251132769544074e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   4.92500E+00  5.00000E-03         2309         1038       2           11.09 
   4.93000E+00  5.00000E-03         2311         1039       2           11.11 

  End Simulation
 command: var/7, fun= -3.63251132769544074e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   4.93000E+00  5.00000E-03         2311         1039       2           11.11 
   4.93500E+00  5.00000E-03         2313         1040       2           11.13 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   4.93500E+00  5.00000E-03         2313         1040       2           11.13 
   4.94000E+00  5.00000E-03         2315         1041       2           11.13 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   4.94000E+00  5.00000E-03         2315         1041       2           11.13 
   4.94500E+00  5.00000E-03         2317         1042       2           11.14 

  End Simulation
 command: var/7, fun= -3.63251132769544483e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   4.94500E+00  5.00000E-03         2317         1042       2           11.14 
   4.95000E+00  5.00000E-03         2319         1043       2           11.16 

  End Simulation
 command: var/7, fun= -3.63251132769544483e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   4.95000E+00  5.00000E-03         2319         1043       2           11.16 
   4.95500E+00  5.00000E-03         2321         1044       2           11.17 

  End Simulation
 command: var/7, fun= -3.63251132769544074e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   4.95500E+00  5.00000E-03         2321         1044       2           11.17 
   4.96000E+00  5.00000E-03         2323         1045       2           11.17 

  End Simulation
 command: var/7, fun= -3.63251132769544074e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   4.96000E+00  5.00000E-03         2323         1045       2           11.17 
   4.96500E+00  5.00000E-03         2325         1046       2           11.19 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   4.96500E+00  5.00000E-03         2325         1046       2           11.19 
   4.97000E+00  5.00000E-03         2327         1047       2           11.20 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   4.97000E+00  5.00000E-03         2327         1047       2           11.20 
   4.97500E+00  5.00000E-03         2329         1048       2           11.20 

  End Simulation
 command: var/7, fun= -3.63251132769544438e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   4.97500E+00  5.00000E-03         2329         1048       2           11.20 
   4.98000E+00  5.00000E-03         2331         1049       2           11.20 

  End Simulation
 command: var/7, fun= -3.63251132769544438e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   4.98000E+00  5.00000E-03         2331         1049       2           11.20 
   4.98500E+00  5.00000E-03         2333         1050       2           11.22 

  End Simulation
 command: var/7, fun= -3.63251132769544074e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   4.98500E+00  5.00000E-03         2333         1050       2           11.22 
   4.99000E+00  5.00000E-03         2335         1051       2           11.22 

  End Simulation
 command: var/7, fun= -3.63251132769544074e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   4.99000E+00  5.00000E-03         2335         1051       2           11.22 
   4.99500E+00  5.00000E-03         2337         1052       2           11.22 

  End Simulation
 command: var/7, fun= -3.63251132769544893e+03


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-03

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   4.99500E+00  5.00000E-03         2337         1052       2           11.22 
   5.00000E+00  5.00000E-03         2339         1053       2           11.23 

  End Simulation


Finished -----
Elapsed time = 42.61s,  CPU time = 11.27s,  26.44%

