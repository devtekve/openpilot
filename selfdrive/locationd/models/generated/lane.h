#pragma once
#include "rednose/helpers/common_ekf.h"
extern "C" {
void lane_update_18(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void lane_err_fun(double *nom_x, double *delta_x, double *out_1860508488552952987);
void lane_inv_err_fun(double *nom_x, double *true_x, double *out_5901639540911762316);
void lane_H_mod_fun(double *state, double *out_7047678969751563333);
void lane_f_fun(double *state, double dd, double *out_5732415076332320842);
void lane_F_fun(double *state, double dd, double *out_6137822031242167206);
void lane_h_18(double *state, double *unused, double *out_526534197004359629);
void lane_H_18(double *state, double *unused, double *out_8752775213417897399);
void lane_predict(double *in_x, double *in_P, double *in_Q, double dt);
}