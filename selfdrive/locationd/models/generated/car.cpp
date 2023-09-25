#include "car.h"

namespace {
#define DIM 9
#define EDIM 9
#define MEDIM 9
typedef void (*Hfun)(double *, double *, double *);

double mass;

void set_mass(double x){ mass = x;}

double rotational_inertia;

void set_rotational_inertia(double x){ rotational_inertia = x;}

double center_to_front;

void set_center_to_front(double x){ center_to_front = x;}

double center_to_rear;

void set_center_to_rear(double x){ center_to_rear = x;}

double stiffness_front;

void set_stiffness_front(double x){ stiffness_front = x;}

double stiffness_rear;

void set_stiffness_rear(double x){ stiffness_rear = x;}
const static double MAHA_THRESH_25 = 3.8414588206941227;
const static double MAHA_THRESH_24 = 5.991464547107981;
const static double MAHA_THRESH_30 = 3.8414588206941227;
const static double MAHA_THRESH_26 = 3.8414588206941227;
const static double MAHA_THRESH_27 = 3.8414588206941227;
const static double MAHA_THRESH_29 = 3.8414588206941227;
const static double MAHA_THRESH_28 = 3.8414588206941227;
const static double MAHA_THRESH_31 = 3.8414588206941227;

/******************************************************************************
 *                       Code generated with SymPy 1.12                       *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                         This file is part of 'ekf'                         *
 ******************************************************************************/
void err_fun(double *nom_x, double *delta_x, double *out_6405586198045064763) {
   out_6405586198045064763[0] = delta_x[0] + nom_x[0];
   out_6405586198045064763[1] = delta_x[1] + nom_x[1];
   out_6405586198045064763[2] = delta_x[2] + nom_x[2];
   out_6405586198045064763[3] = delta_x[3] + nom_x[3];
   out_6405586198045064763[4] = delta_x[4] + nom_x[4];
   out_6405586198045064763[5] = delta_x[5] + nom_x[5];
   out_6405586198045064763[6] = delta_x[6] + nom_x[6];
   out_6405586198045064763[7] = delta_x[7] + nom_x[7];
   out_6405586198045064763[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_3316869061539667119) {
   out_3316869061539667119[0] = -nom_x[0] + true_x[0];
   out_3316869061539667119[1] = -nom_x[1] + true_x[1];
   out_3316869061539667119[2] = -nom_x[2] + true_x[2];
   out_3316869061539667119[3] = -nom_x[3] + true_x[3];
   out_3316869061539667119[4] = -nom_x[4] + true_x[4];
   out_3316869061539667119[5] = -nom_x[5] + true_x[5];
   out_3316869061539667119[6] = -nom_x[6] + true_x[6];
   out_3316869061539667119[7] = -nom_x[7] + true_x[7];
   out_3316869061539667119[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_818954866575854649) {
   out_818954866575854649[0] = 1.0;
   out_818954866575854649[1] = 0;
   out_818954866575854649[2] = 0;
   out_818954866575854649[3] = 0;
   out_818954866575854649[4] = 0;
   out_818954866575854649[5] = 0;
   out_818954866575854649[6] = 0;
   out_818954866575854649[7] = 0;
   out_818954866575854649[8] = 0;
   out_818954866575854649[9] = 0;
   out_818954866575854649[10] = 1.0;
   out_818954866575854649[11] = 0;
   out_818954866575854649[12] = 0;
   out_818954866575854649[13] = 0;
   out_818954866575854649[14] = 0;
   out_818954866575854649[15] = 0;
   out_818954866575854649[16] = 0;
   out_818954866575854649[17] = 0;
   out_818954866575854649[18] = 0;
   out_818954866575854649[19] = 0;
   out_818954866575854649[20] = 1.0;
   out_818954866575854649[21] = 0;
   out_818954866575854649[22] = 0;
   out_818954866575854649[23] = 0;
   out_818954866575854649[24] = 0;
   out_818954866575854649[25] = 0;
   out_818954866575854649[26] = 0;
   out_818954866575854649[27] = 0;
   out_818954866575854649[28] = 0;
   out_818954866575854649[29] = 0;
   out_818954866575854649[30] = 1.0;
   out_818954866575854649[31] = 0;
   out_818954866575854649[32] = 0;
   out_818954866575854649[33] = 0;
   out_818954866575854649[34] = 0;
   out_818954866575854649[35] = 0;
   out_818954866575854649[36] = 0;
   out_818954866575854649[37] = 0;
   out_818954866575854649[38] = 0;
   out_818954866575854649[39] = 0;
   out_818954866575854649[40] = 1.0;
   out_818954866575854649[41] = 0;
   out_818954866575854649[42] = 0;
   out_818954866575854649[43] = 0;
   out_818954866575854649[44] = 0;
   out_818954866575854649[45] = 0;
   out_818954866575854649[46] = 0;
   out_818954866575854649[47] = 0;
   out_818954866575854649[48] = 0;
   out_818954866575854649[49] = 0;
   out_818954866575854649[50] = 1.0;
   out_818954866575854649[51] = 0;
   out_818954866575854649[52] = 0;
   out_818954866575854649[53] = 0;
   out_818954866575854649[54] = 0;
   out_818954866575854649[55] = 0;
   out_818954866575854649[56] = 0;
   out_818954866575854649[57] = 0;
   out_818954866575854649[58] = 0;
   out_818954866575854649[59] = 0;
   out_818954866575854649[60] = 1.0;
   out_818954866575854649[61] = 0;
   out_818954866575854649[62] = 0;
   out_818954866575854649[63] = 0;
   out_818954866575854649[64] = 0;
   out_818954866575854649[65] = 0;
   out_818954866575854649[66] = 0;
   out_818954866575854649[67] = 0;
   out_818954866575854649[68] = 0;
   out_818954866575854649[69] = 0;
   out_818954866575854649[70] = 1.0;
   out_818954866575854649[71] = 0;
   out_818954866575854649[72] = 0;
   out_818954866575854649[73] = 0;
   out_818954866575854649[74] = 0;
   out_818954866575854649[75] = 0;
   out_818954866575854649[76] = 0;
   out_818954866575854649[77] = 0;
   out_818954866575854649[78] = 0;
   out_818954866575854649[79] = 0;
   out_818954866575854649[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_9199471104524812554) {
   out_9199471104524812554[0] = state[0];
   out_9199471104524812554[1] = state[1];
   out_9199471104524812554[2] = state[2];
   out_9199471104524812554[3] = state[3];
   out_9199471104524812554[4] = state[4];
   out_9199471104524812554[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8000000000000007*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_9199471104524812554[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_9199471104524812554[7] = state[7];
   out_9199471104524812554[8] = state[8];
}
void F_fun(double *state, double dt, double *out_8094123874431399722) {
   out_8094123874431399722[0] = 1;
   out_8094123874431399722[1] = 0;
   out_8094123874431399722[2] = 0;
   out_8094123874431399722[3] = 0;
   out_8094123874431399722[4] = 0;
   out_8094123874431399722[5] = 0;
   out_8094123874431399722[6] = 0;
   out_8094123874431399722[7] = 0;
   out_8094123874431399722[8] = 0;
   out_8094123874431399722[9] = 0;
   out_8094123874431399722[10] = 1;
   out_8094123874431399722[11] = 0;
   out_8094123874431399722[12] = 0;
   out_8094123874431399722[13] = 0;
   out_8094123874431399722[14] = 0;
   out_8094123874431399722[15] = 0;
   out_8094123874431399722[16] = 0;
   out_8094123874431399722[17] = 0;
   out_8094123874431399722[18] = 0;
   out_8094123874431399722[19] = 0;
   out_8094123874431399722[20] = 1;
   out_8094123874431399722[21] = 0;
   out_8094123874431399722[22] = 0;
   out_8094123874431399722[23] = 0;
   out_8094123874431399722[24] = 0;
   out_8094123874431399722[25] = 0;
   out_8094123874431399722[26] = 0;
   out_8094123874431399722[27] = 0;
   out_8094123874431399722[28] = 0;
   out_8094123874431399722[29] = 0;
   out_8094123874431399722[30] = 1;
   out_8094123874431399722[31] = 0;
   out_8094123874431399722[32] = 0;
   out_8094123874431399722[33] = 0;
   out_8094123874431399722[34] = 0;
   out_8094123874431399722[35] = 0;
   out_8094123874431399722[36] = 0;
   out_8094123874431399722[37] = 0;
   out_8094123874431399722[38] = 0;
   out_8094123874431399722[39] = 0;
   out_8094123874431399722[40] = 1;
   out_8094123874431399722[41] = 0;
   out_8094123874431399722[42] = 0;
   out_8094123874431399722[43] = 0;
   out_8094123874431399722[44] = 0;
   out_8094123874431399722[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_8094123874431399722[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_8094123874431399722[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_8094123874431399722[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_8094123874431399722[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_8094123874431399722[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_8094123874431399722[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_8094123874431399722[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_8094123874431399722[53] = -9.8000000000000007*dt;
   out_8094123874431399722[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_8094123874431399722[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_8094123874431399722[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_8094123874431399722[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_8094123874431399722[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_8094123874431399722[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_8094123874431399722[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_8094123874431399722[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_8094123874431399722[62] = 0;
   out_8094123874431399722[63] = 0;
   out_8094123874431399722[64] = 0;
   out_8094123874431399722[65] = 0;
   out_8094123874431399722[66] = 0;
   out_8094123874431399722[67] = 0;
   out_8094123874431399722[68] = 0;
   out_8094123874431399722[69] = 0;
   out_8094123874431399722[70] = 1;
   out_8094123874431399722[71] = 0;
   out_8094123874431399722[72] = 0;
   out_8094123874431399722[73] = 0;
   out_8094123874431399722[74] = 0;
   out_8094123874431399722[75] = 0;
   out_8094123874431399722[76] = 0;
   out_8094123874431399722[77] = 0;
   out_8094123874431399722[78] = 0;
   out_8094123874431399722[79] = 0;
   out_8094123874431399722[80] = 1;
}
void h_25(double *state, double *unused, double *out_301876459492834287) {
   out_301876459492834287[0] = state[6];
}
void H_25(double *state, double *unused, double *out_1314426040298649704) {
   out_1314426040298649704[0] = 0;
   out_1314426040298649704[1] = 0;
   out_1314426040298649704[2] = 0;
   out_1314426040298649704[3] = 0;
   out_1314426040298649704[4] = 0;
   out_1314426040298649704[5] = 0;
   out_1314426040298649704[6] = 1;
   out_1314426040298649704[7] = 0;
   out_1314426040298649704[8] = 0;
}
void h_24(double *state, double *unused, double *out_7349785640378725628) {
   out_7349785640378725628[0] = state[4];
   out_7349785640378725628[1] = state[5];
}
void H_24(double *state, double *unused, double *out_6103965593378277189) {
   out_6103965593378277189[0] = 0;
   out_6103965593378277189[1] = 0;
   out_6103965593378277189[2] = 0;
   out_6103965593378277189[3] = 0;
   out_6103965593378277189[4] = 1;
   out_6103965593378277189[5] = 0;
   out_6103965593378277189[6] = 0;
   out_6103965593378277189[7] = 0;
   out_6103965593378277189[8] = 0;
   out_6103965593378277189[9] = 0;
   out_6103965593378277189[10] = 0;
   out_6103965593378277189[11] = 0;
   out_6103965593378277189[12] = 0;
   out_6103965593378277189[13] = 0;
   out_6103965593378277189[14] = 1;
   out_6103965593378277189[15] = 0;
   out_6103965593378277189[16] = 0;
   out_6103965593378277189[17] = 0;
}
void h_30(double *state, double *unused, double *out_6889704385969129450) {
   out_6889704385969129450[0] = state[4];
}
void H_30(double *state, double *unused, double *out_5842122370426257902) {
   out_5842122370426257902[0] = 0;
   out_5842122370426257902[1] = 0;
   out_5842122370426257902[2] = 0;
   out_5842122370426257902[3] = 0;
   out_5842122370426257902[4] = 1;
   out_5842122370426257902[5] = 0;
   out_5842122370426257902[6] = 0;
   out_5842122370426257902[7] = 0;
   out_5842122370426257902[8] = 0;
}
void h_26(double *state, double *unused, double *out_2491752304687500438) {
   out_2491752304687500438[0] = state[7];
}
void H_26(double *state, double *unused, double *out_5055929359172705928) {
   out_5055929359172705928[0] = 0;
   out_5055929359172705928[1] = 0;
   out_5055929359172705928[2] = 0;
   out_5055929359172705928[3] = 0;
   out_5055929359172705928[4] = 0;
   out_5055929359172705928[5] = 0;
   out_5055929359172705928[6] = 0;
   out_5055929359172705928[7] = 1;
   out_5055929359172705928[8] = 0;
}
void h_27(double *state, double *unused, double *out_6097165057921014859) {
   out_6097165057921014859[0] = state[3];
}
void H_27(double *state, double *unused, double *out_3618528299242314685) {
   out_3618528299242314685[0] = 0;
   out_3618528299242314685[1] = 0;
   out_3618528299242314685[2] = 0;
   out_3618528299242314685[3] = 1;
   out_3618528299242314685[4] = 0;
   out_3618528299242314685[5] = 0;
   out_3618528299242314685[6] = 0;
   out_3618528299242314685[7] = 0;
   out_3618528299242314685[8] = 0;
}
void h_29(double *state, double *unused, double *out_6555366420079576521) {
   out_6555366420079576521[0] = state[1];
}
void H_29(double *state, double *unused, double *out_5331891026111865718) {
   out_5331891026111865718[0] = 0;
   out_5331891026111865718[1] = 1;
   out_5331891026111865718[2] = 0;
   out_5331891026111865718[3] = 0;
   out_5331891026111865718[4] = 0;
   out_5331891026111865718[5] = 0;
   out_5331891026111865718[6] = 0;
   out_5331891026111865718[7] = 0;
   out_5331891026111865718[8] = 0;
}
void h_28(double *state, double *unused, double *out_7040674910822677294) {
   out_7040674910822677294[0] = state[0];
}
void H_28(double *state, double *unused, double *out_3368260754546539467) {
   out_3368260754546539467[0] = 1;
   out_3368260754546539467[1] = 0;
   out_3368260754546539467[2] = 0;
   out_3368260754546539467[3] = 0;
   out_3368260754546539467[4] = 0;
   out_3368260754546539467[5] = 0;
   out_3368260754546539467[6] = 0;
   out_3368260754546539467[7] = 0;
   out_3368260754546539467[8] = 0;
}
void h_31(double *state, double *unused, double *out_3335573068761188063) {
   out_3335573068761188063[0] = state[8];
}
void H_31(double *state, double *unused, double *out_5682137461406057404) {
   out_5682137461406057404[0] = 0;
   out_5682137461406057404[1] = 0;
   out_5682137461406057404[2] = 0;
   out_5682137461406057404[3] = 0;
   out_5682137461406057404[4] = 0;
   out_5682137461406057404[5] = 0;
   out_5682137461406057404[6] = 0;
   out_5682137461406057404[7] = 0;
   out_5682137461406057404[8] = 1;
}
#include <eigen3/Eigen/Dense>
#include <iostream>

typedef Eigen::Matrix<double, DIM, DIM, Eigen::RowMajor> DDM;
typedef Eigen::Matrix<double, EDIM, EDIM, Eigen::RowMajor> EEM;
typedef Eigen::Matrix<double, DIM, EDIM, Eigen::RowMajor> DEM;

void predict(double *in_x, double *in_P, double *in_Q, double dt) {
  typedef Eigen::Matrix<double, MEDIM, MEDIM, Eigen::RowMajor> RRM;

  double nx[DIM] = {0};
  double in_F[EDIM*EDIM] = {0};

  // functions from sympy
  f_fun(in_x, dt, nx);
  F_fun(in_x, dt, in_F);


  EEM F(in_F);
  EEM P(in_P);
  EEM Q(in_Q);

  RRM F_main = F.topLeftCorner(MEDIM, MEDIM);
  P.topLeftCorner(MEDIM, MEDIM) = (F_main * P.topLeftCorner(MEDIM, MEDIM)) * F_main.transpose();
  P.topRightCorner(MEDIM, EDIM - MEDIM) = F_main * P.topRightCorner(MEDIM, EDIM - MEDIM);
  P.bottomLeftCorner(EDIM - MEDIM, MEDIM) = P.bottomLeftCorner(EDIM - MEDIM, MEDIM) * F_main.transpose();

  P = P + dt*Q;

  // copy out state
  memcpy(in_x, nx, DIM * sizeof(double));
  memcpy(in_P, P.data(), EDIM * EDIM * sizeof(double));
}

// note: extra_args dim only correct when null space projecting
// otherwise 1
template <int ZDIM, int EADIM, bool MAHA_TEST>
void update(double *in_x, double *in_P, Hfun h_fun, Hfun H_fun, Hfun Hea_fun, double *in_z, double *in_R, double *in_ea, double MAHA_THRESHOLD) {
  typedef Eigen::Matrix<double, ZDIM, ZDIM, Eigen::RowMajor> ZZM;
  typedef Eigen::Matrix<double, ZDIM, DIM, Eigen::RowMajor> ZDM;
  typedef Eigen::Matrix<double, Eigen::Dynamic, EDIM, Eigen::RowMajor> XEM;
  //typedef Eigen::Matrix<double, EDIM, ZDIM, Eigen::RowMajor> EZM;
  typedef Eigen::Matrix<double, Eigen::Dynamic, 1> X1M;
  typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> XXM;

  double in_hx[ZDIM] = {0};
  double in_H[ZDIM * DIM] = {0};
  double in_H_mod[EDIM * DIM] = {0};
  double delta_x[EDIM] = {0};
  double x_new[DIM] = {0};


  // state x, P
  Eigen::Matrix<double, ZDIM, 1> z(in_z);
  EEM P(in_P);
  ZZM pre_R(in_R);

  // functions from sympy
  h_fun(in_x, in_ea, in_hx);
  H_fun(in_x, in_ea, in_H);
  ZDM pre_H(in_H);

  // get y (y = z - hx)
  Eigen::Matrix<double, ZDIM, 1> pre_y(in_hx); pre_y = z - pre_y;
  X1M y; XXM H; XXM R;
  if (Hea_fun){
    typedef Eigen::Matrix<double, ZDIM, EADIM, Eigen::RowMajor> ZAM;
    double in_Hea[ZDIM * EADIM] = {0};
    Hea_fun(in_x, in_ea, in_Hea);
    ZAM Hea(in_Hea);
    XXM A = Hea.transpose().fullPivLu().kernel();


    y = A.transpose() * pre_y;
    H = A.transpose() * pre_H;
    R = A.transpose() * pre_R * A;
  } else {
    y = pre_y;
    H = pre_H;
    R = pre_R;
  }
  // get modified H
  H_mod_fun(in_x, in_H_mod);
  DEM H_mod(in_H_mod);
  XEM H_err = H * H_mod;

  // Do mahalobis distance test
  if (MAHA_TEST){
    XXM a = (H_err * P * H_err.transpose() + R).inverse();
    double maha_dist = y.transpose() * a * y;
    if (maha_dist > MAHA_THRESHOLD){
      R = 1.0e16 * R;
    }
  }

  // Outlier resilient weighting
  double weight = 1;//(1.5)/(1 + y.squaredNorm()/R.sum());

  // kalman gains and I_KH
  XXM S = ((H_err * P) * H_err.transpose()) + R/weight;
  XEM KT = S.fullPivLu().solve(H_err * P.transpose());
  //EZM K = KT.transpose(); TODO: WHY DOES THIS NOT COMPILE?
  //EZM K = S.fullPivLu().solve(H_err * P.transpose()).transpose();
  //std::cout << "Here is the matrix rot:\n" << K << std::endl;
  EEM I_KH = Eigen::Matrix<double, EDIM, EDIM>::Identity() - (KT.transpose() * H_err);

  // update state by injecting dx
  Eigen::Matrix<double, EDIM, 1> dx(delta_x);
  dx  = (KT.transpose() * y);
  memcpy(delta_x, dx.data(), EDIM * sizeof(double));
  err_fun(in_x, delta_x, x_new);
  Eigen::Matrix<double, DIM, 1> x(x_new);

  // update cov
  P = ((I_KH * P) * I_KH.transpose()) + ((KT.transpose() * R) * KT);

  // copy out state
  memcpy(in_x, x.data(), DIM * sizeof(double));
  memcpy(in_P, P.data(), EDIM * EDIM * sizeof(double));
  memcpy(in_z, y.data(), y.rows() * sizeof(double));
}




}
extern "C" {

void car_update_25(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_25, H_25, NULL, in_z, in_R, in_ea, MAHA_THRESH_25);
}
void car_update_24(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<2, 3, 0>(in_x, in_P, h_24, H_24, NULL, in_z, in_R, in_ea, MAHA_THRESH_24);
}
void car_update_30(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_30, H_30, NULL, in_z, in_R, in_ea, MAHA_THRESH_30);
}
void car_update_26(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_26, H_26, NULL, in_z, in_R, in_ea, MAHA_THRESH_26);
}
void car_update_27(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_27, H_27, NULL, in_z, in_R, in_ea, MAHA_THRESH_27);
}
void car_update_29(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_29, H_29, NULL, in_z, in_R, in_ea, MAHA_THRESH_29);
}
void car_update_28(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_28, H_28, NULL, in_z, in_R, in_ea, MAHA_THRESH_28);
}
void car_update_31(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_31, H_31, NULL, in_z, in_R, in_ea, MAHA_THRESH_31);
}
void car_err_fun(double *nom_x, double *delta_x, double *out_6405586198045064763) {
  err_fun(nom_x, delta_x, out_6405586198045064763);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_3316869061539667119) {
  inv_err_fun(nom_x, true_x, out_3316869061539667119);
}
void car_H_mod_fun(double *state, double *out_818954866575854649) {
  H_mod_fun(state, out_818954866575854649);
}
void car_f_fun(double *state, double dt, double *out_9199471104524812554) {
  f_fun(state,  dt, out_9199471104524812554);
}
void car_F_fun(double *state, double dt, double *out_8094123874431399722) {
  F_fun(state,  dt, out_8094123874431399722);
}
void car_h_25(double *state, double *unused, double *out_301876459492834287) {
  h_25(state, unused, out_301876459492834287);
}
void car_H_25(double *state, double *unused, double *out_1314426040298649704) {
  H_25(state, unused, out_1314426040298649704);
}
void car_h_24(double *state, double *unused, double *out_7349785640378725628) {
  h_24(state, unused, out_7349785640378725628);
}
void car_H_24(double *state, double *unused, double *out_6103965593378277189) {
  H_24(state, unused, out_6103965593378277189);
}
void car_h_30(double *state, double *unused, double *out_6889704385969129450) {
  h_30(state, unused, out_6889704385969129450);
}
void car_H_30(double *state, double *unused, double *out_5842122370426257902) {
  H_30(state, unused, out_5842122370426257902);
}
void car_h_26(double *state, double *unused, double *out_2491752304687500438) {
  h_26(state, unused, out_2491752304687500438);
}
void car_H_26(double *state, double *unused, double *out_5055929359172705928) {
  H_26(state, unused, out_5055929359172705928);
}
void car_h_27(double *state, double *unused, double *out_6097165057921014859) {
  h_27(state, unused, out_6097165057921014859);
}
void car_H_27(double *state, double *unused, double *out_3618528299242314685) {
  H_27(state, unused, out_3618528299242314685);
}
void car_h_29(double *state, double *unused, double *out_6555366420079576521) {
  h_29(state, unused, out_6555366420079576521);
}
void car_H_29(double *state, double *unused, double *out_5331891026111865718) {
  H_29(state, unused, out_5331891026111865718);
}
void car_h_28(double *state, double *unused, double *out_7040674910822677294) {
  h_28(state, unused, out_7040674910822677294);
}
void car_H_28(double *state, double *unused, double *out_3368260754546539467) {
  H_28(state, unused, out_3368260754546539467);
}
void car_h_31(double *state, double *unused, double *out_3335573068761188063) {
  h_31(state, unused, out_3335573068761188063);
}
void car_H_31(double *state, double *unused, double *out_5682137461406057404) {
  H_31(state, unused, out_5682137461406057404);
}
void car_predict(double *in_x, double *in_P, double *in_Q, double dt) {
  predict(in_x, in_P, in_Q, dt);
}
void car_set_mass(double x) {
  set_mass(x);
}
void car_set_rotational_inertia(double x) {
  set_rotational_inertia(x);
}
void car_set_center_to_front(double x) {
  set_center_to_front(x);
}
void car_set_center_to_rear(double x) {
  set_center_to_rear(x);
}
void car_set_stiffness_front(double x) {
  set_stiffness_front(x);
}
void car_set_stiffness_rear(double x) {
  set_stiffness_rear(x);
}
}

const EKF car = {
  .name = "car",
  .kinds = { 25, 24, 30, 26, 27, 29, 28, 31 },
  .feature_kinds = {  },
  .f_fun = car_f_fun,
  .F_fun = car_F_fun,
  .err_fun = car_err_fun,
  .inv_err_fun = car_inv_err_fun,
  .H_mod_fun = car_H_mod_fun,
  .predict = car_predict,
  .hs = {
    { 25, car_h_25 },
    { 24, car_h_24 },
    { 30, car_h_30 },
    { 26, car_h_26 },
    { 27, car_h_27 },
    { 29, car_h_29 },
    { 28, car_h_28 },
    { 31, car_h_31 },
  },
  .Hs = {
    { 25, car_H_25 },
    { 24, car_H_24 },
    { 30, car_H_30 },
    { 26, car_H_26 },
    { 27, car_H_27 },
    { 29, car_H_29 },
    { 28, car_H_28 },
    { 31, car_H_31 },
  },
  .updates = {
    { 25, car_update_25 },
    { 24, car_update_24 },
    { 30, car_update_30 },
    { 26, car_update_26 },
    { 27, car_update_27 },
    { 29, car_update_29 },
    { 28, car_update_28 },
    { 31, car_update_31 },
  },
  .Hes = {
  },
  .sets = {
    { "mass", car_set_mass },
    { "rotational_inertia", car_set_rotational_inertia },
    { "center_to_front", car_set_center_to_front },
    { "center_to_rear", car_set_center_to_rear },
    { "stiffness_front", car_set_stiffness_front },
    { "stiffness_rear", car_set_stiffness_rear },
  },
  .extra_routines = {
  },
};

ekf_init(car);
