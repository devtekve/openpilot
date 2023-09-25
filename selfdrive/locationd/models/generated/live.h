#pragma once
#include "rednose/helpers/common_ekf.h"
extern "C" {
void live_update_4(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_9(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_10(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_12(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_35(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_32(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_13(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_14(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_33(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_H(double *in_vec, double *out_882580126896942295);
void live_err_fun(double *nom_x, double *delta_x, double *out_5996174494959834025);
void live_inv_err_fun(double *nom_x, double *true_x, double *out_6459459844882907304);
void live_H_mod_fun(double *state, double *out_8080057036640925045);
void live_f_fun(double *state, double dt, double *out_5988242100613668006);
void live_F_fun(double *state, double dt, double *out_4629418675209333035);
void live_h_4(double *state, double *unused, double *out_6919725180684590112);
void live_H_4(double *state, double *unused, double *out_7649182230504147241);
void live_h_9(double *state, double *unused, double *out_6579658735120463831);
void live_H_9(double *state, double *unused, double *out_7890371877133737886);
void live_h_10(double *state, double *unused, double *out_7199017713804032144);
void live_H_10(double *state, double *unused, double *out_5379501964687391970);
void live_h_12(double *state, double *unused, double *out_5558218791521407497);
void live_H_12(double *state, double *unused, double *out_8270281255551740908);
void live_h_35(double *state, double *unused, double *out_5935994973679120654);
void live_H_35(double *state, double *unused, double *out_7430899785832796999);
void live_h_32(double *state, double *unused, double *out_8756716565681570076);
void live_H_32(double *state, double *unused, double *out_4647241621414379406);
void live_h_13(double *state, double *unused, double *out_9075639493952822068);
void live_H_13(double *state, double *unused, double *out_6314434707637418687);
void live_h_14(double *state, double *unused, double *out_6579658735120463831);
void live_H_14(double *state, double *unused, double *out_7890371877133737886);
void live_h_33(double *state, double *unused, double *out_4423886799982408917);
void live_H_33(double *state, double *unused, double *out_4280342781193939395);
void live_predict(double *in_x, double *in_P, double *in_Q, double dt);
}