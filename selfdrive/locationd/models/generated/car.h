#pragma once
#include "rednose/helpers/common_ekf.h"
extern "C" {
void car_update_25(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_24(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_30(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_26(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_27(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_29(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_28(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_31(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_err_fun(double *nom_x, double *delta_x, double *out_6405586198045064763);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_3316869061539667119);
void car_H_mod_fun(double *state, double *out_818954866575854649);
void car_f_fun(double *state, double dt, double *out_9199471104524812554);
void car_F_fun(double *state, double dt, double *out_8094123874431399722);
void car_h_25(double *state, double *unused, double *out_301876459492834287);
void car_H_25(double *state, double *unused, double *out_1314426040298649704);
void car_h_24(double *state, double *unused, double *out_7349785640378725628);
void car_H_24(double *state, double *unused, double *out_6103965593378277189);
void car_h_30(double *state, double *unused, double *out_6889704385969129450);
void car_H_30(double *state, double *unused, double *out_5842122370426257902);
void car_h_26(double *state, double *unused, double *out_2491752304687500438);
void car_H_26(double *state, double *unused, double *out_5055929359172705928);
void car_h_27(double *state, double *unused, double *out_6097165057921014859);
void car_H_27(double *state, double *unused, double *out_3618528299242314685);
void car_h_29(double *state, double *unused, double *out_6555366420079576521);
void car_H_29(double *state, double *unused, double *out_5331891026111865718);
void car_h_28(double *state, double *unused, double *out_7040674910822677294);
void car_H_28(double *state, double *unused, double *out_3368260754546539467);
void car_h_31(double *state, double *unused, double *out_3335573068761188063);
void car_H_31(double *state, double *unused, double *out_5682137461406057404);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}