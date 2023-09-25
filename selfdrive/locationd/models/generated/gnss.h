#pragma once
#include "rednose/helpers/common_ekf.h"
extern "C" {
void gnss_update_6(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_update_20(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_update_7(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_update_21(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_err_fun(double *nom_x, double *delta_x, double *out_4015664806136119694);
void gnss_inv_err_fun(double *nom_x, double *true_x, double *out_4638289788108739971);
void gnss_H_mod_fun(double *state, double *out_3854302207848908237);
void gnss_f_fun(double *state, double dt, double *out_3320937052910667211);
void gnss_F_fun(double *state, double dt, double *out_7811766921854687646);
void gnss_h_6(double *state, double *sat_pos, double *out_5613198087574821366);
void gnss_H_6(double *state, double *sat_pos, double *out_9160780538084901745);
void gnss_h_20(double *state, double *sat_pos, double *out_3416209643607972244);
void gnss_H_20(double *state, double *sat_pos, double *out_1494989141823676808);
void gnss_h_7(double *state, double *sat_pos_vel, double *out_1775211612282646777);
void gnss_H_7(double *state, double *sat_pos_vel, double *out_6317595068337282390);
void gnss_h_21(double *state, double *sat_pos_vel, double *out_1775211612282646777);
void gnss_H_21(double *state, double *sat_pos_vel, double *out_6317595068337282390);
void gnss_predict(double *in_x, double *in_P, double *in_Q, double dt);
}