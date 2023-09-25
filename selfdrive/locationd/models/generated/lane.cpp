#include "lane.h"

namespace {
#define DIM 6
#define EDIM 6
#define MEDIM 6
typedef void (*Hfun)(double *, double *, double *);
const static double MAHA_THRESH_18 = 7.814727903251177;

/******************************************************************************
 *                       Code generated with SymPy 1.12                       *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                         This file is part of 'ekf'                         *
 ******************************************************************************/
void err_fun(double *nom_x, double *delta_x, double *out_1860508488552952987) {
   out_1860508488552952987[0] = delta_x[0] + nom_x[0];
   out_1860508488552952987[1] = delta_x[1] + nom_x[1];
   out_1860508488552952987[2] = delta_x[2] + nom_x[2];
   out_1860508488552952987[3] = delta_x[3] + nom_x[3];
   out_1860508488552952987[4] = delta_x[4] + nom_x[4];
   out_1860508488552952987[5] = delta_x[5] + nom_x[5];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_5901639540911762316) {
   out_5901639540911762316[0] = -nom_x[0] + true_x[0];
   out_5901639540911762316[1] = -nom_x[1] + true_x[1];
   out_5901639540911762316[2] = -nom_x[2] + true_x[2];
   out_5901639540911762316[3] = -nom_x[3] + true_x[3];
   out_5901639540911762316[4] = -nom_x[4] + true_x[4];
   out_5901639540911762316[5] = -nom_x[5] + true_x[5];
}
void H_mod_fun(double *state, double *out_7047678969751563333) {
   out_7047678969751563333[0] = 1.0;
   out_7047678969751563333[1] = 0;
   out_7047678969751563333[2] = 0;
   out_7047678969751563333[3] = 0;
   out_7047678969751563333[4] = 0;
   out_7047678969751563333[5] = 0;
   out_7047678969751563333[6] = 0;
   out_7047678969751563333[7] = 1.0;
   out_7047678969751563333[8] = 0;
   out_7047678969751563333[9] = 0;
   out_7047678969751563333[10] = 0;
   out_7047678969751563333[11] = 0;
   out_7047678969751563333[12] = 0;
   out_7047678969751563333[13] = 0;
   out_7047678969751563333[14] = 1.0;
   out_7047678969751563333[15] = 0;
   out_7047678969751563333[16] = 0;
   out_7047678969751563333[17] = 0;
   out_7047678969751563333[18] = 0;
   out_7047678969751563333[19] = 0;
   out_7047678969751563333[20] = 0;
   out_7047678969751563333[21] = 1.0;
   out_7047678969751563333[22] = 0;
   out_7047678969751563333[23] = 0;
   out_7047678969751563333[24] = 0;
   out_7047678969751563333[25] = 0;
   out_7047678969751563333[26] = 0;
   out_7047678969751563333[27] = 0;
   out_7047678969751563333[28] = 1.0;
   out_7047678969751563333[29] = 0;
   out_7047678969751563333[30] = 0;
   out_7047678969751563333[31] = 0;
   out_7047678969751563333[32] = 0;
   out_7047678969751563333[33] = 0;
   out_7047678969751563333[34] = 0;
   out_7047678969751563333[35] = 1.0;
}
void f_fun(double *state, double dd, double *out_5732415076332320842) {
   out_5732415076332320842[0] = dd*state[3] + state[0];
   out_5732415076332320842[1] = dd*state[4] + state[1];
   out_5732415076332320842[2] = dd*state[5] + state[2];
   out_5732415076332320842[3] = state[3];
   out_5732415076332320842[4] = state[4];
   out_5732415076332320842[5] = state[5];
}
void F_fun(double *state, double dd, double *out_6137822031242167206) {
   out_6137822031242167206[0] = 1;
   out_6137822031242167206[1] = 0;
   out_6137822031242167206[2] = 0;
   out_6137822031242167206[3] = dd;
   out_6137822031242167206[4] = 0;
   out_6137822031242167206[5] = 0;
   out_6137822031242167206[6] = 0;
   out_6137822031242167206[7] = 1;
   out_6137822031242167206[8] = 0;
   out_6137822031242167206[9] = 0;
   out_6137822031242167206[10] = dd;
   out_6137822031242167206[11] = 0;
   out_6137822031242167206[12] = 0;
   out_6137822031242167206[13] = 0;
   out_6137822031242167206[14] = 1;
   out_6137822031242167206[15] = 0;
   out_6137822031242167206[16] = 0;
   out_6137822031242167206[17] = dd;
   out_6137822031242167206[18] = 0;
   out_6137822031242167206[19] = 0;
   out_6137822031242167206[20] = 0;
   out_6137822031242167206[21] = 1;
   out_6137822031242167206[22] = 0;
   out_6137822031242167206[23] = 0;
   out_6137822031242167206[24] = 0;
   out_6137822031242167206[25] = 0;
   out_6137822031242167206[26] = 0;
   out_6137822031242167206[27] = 0;
   out_6137822031242167206[28] = 1;
   out_6137822031242167206[29] = 0;
   out_6137822031242167206[30] = 0;
   out_6137822031242167206[31] = 0;
   out_6137822031242167206[32] = 0;
   out_6137822031242167206[33] = 0;
   out_6137822031242167206[34] = 0;
   out_6137822031242167206[35] = 1;
}
void h_18(double *state, double *unused, double *out_526534197004359629) {
   out_526534197004359629[0] = state[0];
   out_526534197004359629[1] = state[1];
   out_526534197004359629[2] = state[2];
}
void H_18(double *state, double *unused, double *out_8752775213417897399) {
   out_8752775213417897399[0] = 1;
   out_8752775213417897399[1] = 0;
   out_8752775213417897399[2] = 0;
   out_8752775213417897399[3] = 0;
   out_8752775213417897399[4] = 0;
   out_8752775213417897399[5] = 0;
   out_8752775213417897399[6] = 0;
   out_8752775213417897399[7] = 1;
   out_8752775213417897399[8] = 0;
   out_8752775213417897399[9] = 0;
   out_8752775213417897399[10] = 0;
   out_8752775213417897399[11] = 0;
   out_8752775213417897399[12] = 0;
   out_8752775213417897399[13] = 0;
   out_8752775213417897399[14] = 1;
   out_8752775213417897399[15] = 0;
   out_8752775213417897399[16] = 0;
   out_8752775213417897399[17] = 0;
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

void lane_update_18(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<3, 3, 0>(in_x, in_P, h_18, H_18, NULL, in_z, in_R, in_ea, MAHA_THRESH_18);
}
void lane_err_fun(double *nom_x, double *delta_x, double *out_1860508488552952987) {
  err_fun(nom_x, delta_x, out_1860508488552952987);
}
void lane_inv_err_fun(double *nom_x, double *true_x, double *out_5901639540911762316) {
  inv_err_fun(nom_x, true_x, out_5901639540911762316);
}
void lane_H_mod_fun(double *state, double *out_7047678969751563333) {
  H_mod_fun(state, out_7047678969751563333);
}
void lane_f_fun(double *state, double dd, double *out_5732415076332320842) {
  f_fun(state,  dd, out_5732415076332320842);
}
void lane_F_fun(double *state, double dd, double *out_6137822031242167206) {
  F_fun(state,  dd, out_6137822031242167206);
}
void lane_h_18(double *state, double *unused, double *out_526534197004359629) {
  h_18(state, unused, out_526534197004359629);
}
void lane_H_18(double *state, double *unused, double *out_8752775213417897399) {
  H_18(state, unused, out_8752775213417897399);
}
void lane_predict(double *in_x, double *in_P, double *in_Q, double dt) {
  predict(in_x, in_P, in_Q, dt);
}
}

const EKF lane = {
  .name = "lane",
  .kinds = { 18 },
  .feature_kinds = {  },
  .f_fun = lane_f_fun,
  .F_fun = lane_F_fun,
  .err_fun = lane_err_fun,
  .inv_err_fun = lane_inv_err_fun,
  .H_mod_fun = lane_H_mod_fun,
  .predict = lane_predict,
  .hs = {
    { 18, lane_h_18 },
  },
  .Hs = {
    { 18, lane_H_18 },
  },
  .updates = {
    { 18, lane_update_18 },
  },
  .Hes = {
  },
  .sets = {
  },
  .extra_routines = {
  },
};

ekf_init(lane);
