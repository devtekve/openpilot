#include "gnss.h"

namespace {
#define DIM 11
#define EDIM 11
#define MEDIM 11
typedef void (*Hfun)(double *, double *, double *);
const static double MAHA_THRESH_6 = 3.8414588206941227;
const static double MAHA_THRESH_20 = 3.8414588206941227;
const static double MAHA_THRESH_7 = 3.8414588206941227;
const static double MAHA_THRESH_21 = 3.8414588206941227;

/******************************************************************************
 *                       Code generated with SymPy 1.12                       *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                         This file is part of 'ekf'                         *
 ******************************************************************************/
void err_fun(double *nom_x, double *delta_x, double *out_4015664806136119694) {
   out_4015664806136119694[0] = delta_x[0] + nom_x[0];
   out_4015664806136119694[1] = delta_x[1] + nom_x[1];
   out_4015664806136119694[2] = delta_x[2] + nom_x[2];
   out_4015664806136119694[3] = delta_x[3] + nom_x[3];
   out_4015664806136119694[4] = delta_x[4] + nom_x[4];
   out_4015664806136119694[5] = delta_x[5] + nom_x[5];
   out_4015664806136119694[6] = delta_x[6] + nom_x[6];
   out_4015664806136119694[7] = delta_x[7] + nom_x[7];
   out_4015664806136119694[8] = delta_x[8] + nom_x[8];
   out_4015664806136119694[9] = delta_x[9] + nom_x[9];
   out_4015664806136119694[10] = delta_x[10] + nom_x[10];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_4638289788108739971) {
   out_4638289788108739971[0] = -nom_x[0] + true_x[0];
   out_4638289788108739971[1] = -nom_x[1] + true_x[1];
   out_4638289788108739971[2] = -nom_x[2] + true_x[2];
   out_4638289788108739971[3] = -nom_x[3] + true_x[3];
   out_4638289788108739971[4] = -nom_x[4] + true_x[4];
   out_4638289788108739971[5] = -nom_x[5] + true_x[5];
   out_4638289788108739971[6] = -nom_x[6] + true_x[6];
   out_4638289788108739971[7] = -nom_x[7] + true_x[7];
   out_4638289788108739971[8] = -nom_x[8] + true_x[8];
   out_4638289788108739971[9] = -nom_x[9] + true_x[9];
   out_4638289788108739971[10] = -nom_x[10] + true_x[10];
}
void H_mod_fun(double *state, double *out_3854302207848908237) {
   out_3854302207848908237[0] = 1.0;
   out_3854302207848908237[1] = 0;
   out_3854302207848908237[2] = 0;
   out_3854302207848908237[3] = 0;
   out_3854302207848908237[4] = 0;
   out_3854302207848908237[5] = 0;
   out_3854302207848908237[6] = 0;
   out_3854302207848908237[7] = 0;
   out_3854302207848908237[8] = 0;
   out_3854302207848908237[9] = 0;
   out_3854302207848908237[10] = 0;
   out_3854302207848908237[11] = 0;
   out_3854302207848908237[12] = 1.0;
   out_3854302207848908237[13] = 0;
   out_3854302207848908237[14] = 0;
   out_3854302207848908237[15] = 0;
   out_3854302207848908237[16] = 0;
   out_3854302207848908237[17] = 0;
   out_3854302207848908237[18] = 0;
   out_3854302207848908237[19] = 0;
   out_3854302207848908237[20] = 0;
   out_3854302207848908237[21] = 0;
   out_3854302207848908237[22] = 0;
   out_3854302207848908237[23] = 0;
   out_3854302207848908237[24] = 1.0;
   out_3854302207848908237[25] = 0;
   out_3854302207848908237[26] = 0;
   out_3854302207848908237[27] = 0;
   out_3854302207848908237[28] = 0;
   out_3854302207848908237[29] = 0;
   out_3854302207848908237[30] = 0;
   out_3854302207848908237[31] = 0;
   out_3854302207848908237[32] = 0;
   out_3854302207848908237[33] = 0;
   out_3854302207848908237[34] = 0;
   out_3854302207848908237[35] = 0;
   out_3854302207848908237[36] = 1.0;
   out_3854302207848908237[37] = 0;
   out_3854302207848908237[38] = 0;
   out_3854302207848908237[39] = 0;
   out_3854302207848908237[40] = 0;
   out_3854302207848908237[41] = 0;
   out_3854302207848908237[42] = 0;
   out_3854302207848908237[43] = 0;
   out_3854302207848908237[44] = 0;
   out_3854302207848908237[45] = 0;
   out_3854302207848908237[46] = 0;
   out_3854302207848908237[47] = 0;
   out_3854302207848908237[48] = 1.0;
   out_3854302207848908237[49] = 0;
   out_3854302207848908237[50] = 0;
   out_3854302207848908237[51] = 0;
   out_3854302207848908237[52] = 0;
   out_3854302207848908237[53] = 0;
   out_3854302207848908237[54] = 0;
   out_3854302207848908237[55] = 0;
   out_3854302207848908237[56] = 0;
   out_3854302207848908237[57] = 0;
   out_3854302207848908237[58] = 0;
   out_3854302207848908237[59] = 0;
   out_3854302207848908237[60] = 1.0;
   out_3854302207848908237[61] = 0;
   out_3854302207848908237[62] = 0;
   out_3854302207848908237[63] = 0;
   out_3854302207848908237[64] = 0;
   out_3854302207848908237[65] = 0;
   out_3854302207848908237[66] = 0;
   out_3854302207848908237[67] = 0;
   out_3854302207848908237[68] = 0;
   out_3854302207848908237[69] = 0;
   out_3854302207848908237[70] = 0;
   out_3854302207848908237[71] = 0;
   out_3854302207848908237[72] = 1.0;
   out_3854302207848908237[73] = 0;
   out_3854302207848908237[74] = 0;
   out_3854302207848908237[75] = 0;
   out_3854302207848908237[76] = 0;
   out_3854302207848908237[77] = 0;
   out_3854302207848908237[78] = 0;
   out_3854302207848908237[79] = 0;
   out_3854302207848908237[80] = 0;
   out_3854302207848908237[81] = 0;
   out_3854302207848908237[82] = 0;
   out_3854302207848908237[83] = 0;
   out_3854302207848908237[84] = 1.0;
   out_3854302207848908237[85] = 0;
   out_3854302207848908237[86] = 0;
   out_3854302207848908237[87] = 0;
   out_3854302207848908237[88] = 0;
   out_3854302207848908237[89] = 0;
   out_3854302207848908237[90] = 0;
   out_3854302207848908237[91] = 0;
   out_3854302207848908237[92] = 0;
   out_3854302207848908237[93] = 0;
   out_3854302207848908237[94] = 0;
   out_3854302207848908237[95] = 0;
   out_3854302207848908237[96] = 1.0;
   out_3854302207848908237[97] = 0;
   out_3854302207848908237[98] = 0;
   out_3854302207848908237[99] = 0;
   out_3854302207848908237[100] = 0;
   out_3854302207848908237[101] = 0;
   out_3854302207848908237[102] = 0;
   out_3854302207848908237[103] = 0;
   out_3854302207848908237[104] = 0;
   out_3854302207848908237[105] = 0;
   out_3854302207848908237[106] = 0;
   out_3854302207848908237[107] = 0;
   out_3854302207848908237[108] = 1.0;
   out_3854302207848908237[109] = 0;
   out_3854302207848908237[110] = 0;
   out_3854302207848908237[111] = 0;
   out_3854302207848908237[112] = 0;
   out_3854302207848908237[113] = 0;
   out_3854302207848908237[114] = 0;
   out_3854302207848908237[115] = 0;
   out_3854302207848908237[116] = 0;
   out_3854302207848908237[117] = 0;
   out_3854302207848908237[118] = 0;
   out_3854302207848908237[119] = 0;
   out_3854302207848908237[120] = 1.0;
}
void f_fun(double *state, double dt, double *out_3320937052910667211) {
   out_3320937052910667211[0] = dt*state[3] + state[0];
   out_3320937052910667211[1] = dt*state[4] + state[1];
   out_3320937052910667211[2] = dt*state[5] + state[2];
   out_3320937052910667211[3] = state[3];
   out_3320937052910667211[4] = state[4];
   out_3320937052910667211[5] = state[5];
   out_3320937052910667211[6] = dt*state[7] + state[6];
   out_3320937052910667211[7] = dt*state[8] + state[7];
   out_3320937052910667211[8] = state[8];
   out_3320937052910667211[9] = state[9];
   out_3320937052910667211[10] = state[10];
}
void F_fun(double *state, double dt, double *out_7811766921854687646) {
   out_7811766921854687646[0] = 1;
   out_7811766921854687646[1] = 0;
   out_7811766921854687646[2] = 0;
   out_7811766921854687646[3] = dt;
   out_7811766921854687646[4] = 0;
   out_7811766921854687646[5] = 0;
   out_7811766921854687646[6] = 0;
   out_7811766921854687646[7] = 0;
   out_7811766921854687646[8] = 0;
   out_7811766921854687646[9] = 0;
   out_7811766921854687646[10] = 0;
   out_7811766921854687646[11] = 0;
   out_7811766921854687646[12] = 1;
   out_7811766921854687646[13] = 0;
   out_7811766921854687646[14] = 0;
   out_7811766921854687646[15] = dt;
   out_7811766921854687646[16] = 0;
   out_7811766921854687646[17] = 0;
   out_7811766921854687646[18] = 0;
   out_7811766921854687646[19] = 0;
   out_7811766921854687646[20] = 0;
   out_7811766921854687646[21] = 0;
   out_7811766921854687646[22] = 0;
   out_7811766921854687646[23] = 0;
   out_7811766921854687646[24] = 1;
   out_7811766921854687646[25] = 0;
   out_7811766921854687646[26] = 0;
   out_7811766921854687646[27] = dt;
   out_7811766921854687646[28] = 0;
   out_7811766921854687646[29] = 0;
   out_7811766921854687646[30] = 0;
   out_7811766921854687646[31] = 0;
   out_7811766921854687646[32] = 0;
   out_7811766921854687646[33] = 0;
   out_7811766921854687646[34] = 0;
   out_7811766921854687646[35] = 0;
   out_7811766921854687646[36] = 1;
   out_7811766921854687646[37] = 0;
   out_7811766921854687646[38] = 0;
   out_7811766921854687646[39] = 0;
   out_7811766921854687646[40] = 0;
   out_7811766921854687646[41] = 0;
   out_7811766921854687646[42] = 0;
   out_7811766921854687646[43] = 0;
   out_7811766921854687646[44] = 0;
   out_7811766921854687646[45] = 0;
   out_7811766921854687646[46] = 0;
   out_7811766921854687646[47] = 0;
   out_7811766921854687646[48] = 1;
   out_7811766921854687646[49] = 0;
   out_7811766921854687646[50] = 0;
   out_7811766921854687646[51] = 0;
   out_7811766921854687646[52] = 0;
   out_7811766921854687646[53] = 0;
   out_7811766921854687646[54] = 0;
   out_7811766921854687646[55] = 0;
   out_7811766921854687646[56] = 0;
   out_7811766921854687646[57] = 0;
   out_7811766921854687646[58] = 0;
   out_7811766921854687646[59] = 0;
   out_7811766921854687646[60] = 1;
   out_7811766921854687646[61] = 0;
   out_7811766921854687646[62] = 0;
   out_7811766921854687646[63] = 0;
   out_7811766921854687646[64] = 0;
   out_7811766921854687646[65] = 0;
   out_7811766921854687646[66] = 0;
   out_7811766921854687646[67] = 0;
   out_7811766921854687646[68] = 0;
   out_7811766921854687646[69] = 0;
   out_7811766921854687646[70] = 0;
   out_7811766921854687646[71] = 0;
   out_7811766921854687646[72] = 1;
   out_7811766921854687646[73] = dt;
   out_7811766921854687646[74] = 0;
   out_7811766921854687646[75] = 0;
   out_7811766921854687646[76] = 0;
   out_7811766921854687646[77] = 0;
   out_7811766921854687646[78] = 0;
   out_7811766921854687646[79] = 0;
   out_7811766921854687646[80] = 0;
   out_7811766921854687646[81] = 0;
   out_7811766921854687646[82] = 0;
   out_7811766921854687646[83] = 0;
   out_7811766921854687646[84] = 1;
   out_7811766921854687646[85] = dt;
   out_7811766921854687646[86] = 0;
   out_7811766921854687646[87] = 0;
   out_7811766921854687646[88] = 0;
   out_7811766921854687646[89] = 0;
   out_7811766921854687646[90] = 0;
   out_7811766921854687646[91] = 0;
   out_7811766921854687646[92] = 0;
   out_7811766921854687646[93] = 0;
   out_7811766921854687646[94] = 0;
   out_7811766921854687646[95] = 0;
   out_7811766921854687646[96] = 1;
   out_7811766921854687646[97] = 0;
   out_7811766921854687646[98] = 0;
   out_7811766921854687646[99] = 0;
   out_7811766921854687646[100] = 0;
   out_7811766921854687646[101] = 0;
   out_7811766921854687646[102] = 0;
   out_7811766921854687646[103] = 0;
   out_7811766921854687646[104] = 0;
   out_7811766921854687646[105] = 0;
   out_7811766921854687646[106] = 0;
   out_7811766921854687646[107] = 0;
   out_7811766921854687646[108] = 1;
   out_7811766921854687646[109] = 0;
   out_7811766921854687646[110] = 0;
   out_7811766921854687646[111] = 0;
   out_7811766921854687646[112] = 0;
   out_7811766921854687646[113] = 0;
   out_7811766921854687646[114] = 0;
   out_7811766921854687646[115] = 0;
   out_7811766921854687646[116] = 0;
   out_7811766921854687646[117] = 0;
   out_7811766921854687646[118] = 0;
   out_7811766921854687646[119] = 0;
   out_7811766921854687646[120] = 1;
}
void h_6(double *state, double *sat_pos, double *out_5613198087574821366) {
   out_5613198087574821366[0] = sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2)) + state[6];
}
void H_6(double *state, double *sat_pos, double *out_9160780538084901745) {
   out_9160780538084901745[0] = (-sat_pos[0] + state[0])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_9160780538084901745[1] = (-sat_pos[1] + state[1])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_9160780538084901745[2] = (-sat_pos[2] + state[2])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_9160780538084901745[3] = 0;
   out_9160780538084901745[4] = 0;
   out_9160780538084901745[5] = 0;
   out_9160780538084901745[6] = 1;
   out_9160780538084901745[7] = 0;
   out_9160780538084901745[8] = 0;
   out_9160780538084901745[9] = 0;
   out_9160780538084901745[10] = 0;
}
void h_20(double *state, double *sat_pos, double *out_3416209643607972244) {
   out_3416209643607972244[0] = sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2)) + sat_pos[3]*state[10] + state[6] + state[9];
}
void H_20(double *state, double *sat_pos, double *out_1494989141823676808) {
   out_1494989141823676808[0] = (-sat_pos[0] + state[0])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_1494989141823676808[1] = (-sat_pos[1] + state[1])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_1494989141823676808[2] = (-sat_pos[2] + state[2])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_1494989141823676808[3] = 0;
   out_1494989141823676808[4] = 0;
   out_1494989141823676808[5] = 0;
   out_1494989141823676808[6] = 1;
   out_1494989141823676808[7] = 0;
   out_1494989141823676808[8] = 0;
   out_1494989141823676808[9] = 1;
   out_1494989141823676808[10] = sat_pos[3];
}
void h_7(double *state, double *sat_pos_vel, double *out_1775211612282646777) {
   out_1775211612282646777[0] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + state[7];
}
void H_7(double *state, double *sat_pos_vel, double *out_6317595068337282390) {
   out_6317595068337282390[0] = pow(sat_pos_vel[0] - state[0], 2)*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_6317595068337282390[1] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[1] - state[1], 2)*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_6317595068337282390[2] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[2] - state[2], 2)*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_6317595068337282390[3] = -(sat_pos_vel[0] - state[0])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_6317595068337282390[4] = -(sat_pos_vel[1] - state[1])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_6317595068337282390[5] = -(sat_pos_vel[2] - state[2])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_6317595068337282390[6] = 0;
   out_6317595068337282390[7] = 1;
   out_6317595068337282390[8] = 0;
   out_6317595068337282390[9] = 0;
   out_6317595068337282390[10] = 0;
}
void h_21(double *state, double *sat_pos_vel, double *out_1775211612282646777) {
   out_1775211612282646777[0] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + state[7];
}
void H_21(double *state, double *sat_pos_vel, double *out_6317595068337282390) {
   out_6317595068337282390[0] = pow(sat_pos_vel[0] - state[0], 2)*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_6317595068337282390[1] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[1] - state[1], 2)*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_6317595068337282390[2] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[2] - state[2], 2)*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_6317595068337282390[3] = -(sat_pos_vel[0] - state[0])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_6317595068337282390[4] = -(sat_pos_vel[1] - state[1])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_6317595068337282390[5] = -(sat_pos_vel[2] - state[2])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_6317595068337282390[6] = 0;
   out_6317595068337282390[7] = 1;
   out_6317595068337282390[8] = 0;
   out_6317595068337282390[9] = 0;
   out_6317595068337282390[10] = 0;
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

void gnss_update_6(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_6, H_6, NULL, in_z, in_R, in_ea, MAHA_THRESH_6);
}
void gnss_update_20(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_20, H_20, NULL, in_z, in_R, in_ea, MAHA_THRESH_20);
}
void gnss_update_7(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_7, H_7, NULL, in_z, in_R, in_ea, MAHA_THRESH_7);
}
void gnss_update_21(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_21, H_21, NULL, in_z, in_R, in_ea, MAHA_THRESH_21);
}
void gnss_err_fun(double *nom_x, double *delta_x, double *out_4015664806136119694) {
  err_fun(nom_x, delta_x, out_4015664806136119694);
}
void gnss_inv_err_fun(double *nom_x, double *true_x, double *out_4638289788108739971) {
  inv_err_fun(nom_x, true_x, out_4638289788108739971);
}
void gnss_H_mod_fun(double *state, double *out_3854302207848908237) {
  H_mod_fun(state, out_3854302207848908237);
}
void gnss_f_fun(double *state, double dt, double *out_3320937052910667211) {
  f_fun(state,  dt, out_3320937052910667211);
}
void gnss_F_fun(double *state, double dt, double *out_7811766921854687646) {
  F_fun(state,  dt, out_7811766921854687646);
}
void gnss_h_6(double *state, double *sat_pos, double *out_5613198087574821366) {
  h_6(state, sat_pos, out_5613198087574821366);
}
void gnss_H_6(double *state, double *sat_pos, double *out_9160780538084901745) {
  H_6(state, sat_pos, out_9160780538084901745);
}
void gnss_h_20(double *state, double *sat_pos, double *out_3416209643607972244) {
  h_20(state, sat_pos, out_3416209643607972244);
}
void gnss_H_20(double *state, double *sat_pos, double *out_1494989141823676808) {
  H_20(state, sat_pos, out_1494989141823676808);
}
void gnss_h_7(double *state, double *sat_pos_vel, double *out_1775211612282646777) {
  h_7(state, sat_pos_vel, out_1775211612282646777);
}
void gnss_H_7(double *state, double *sat_pos_vel, double *out_6317595068337282390) {
  H_7(state, sat_pos_vel, out_6317595068337282390);
}
void gnss_h_21(double *state, double *sat_pos_vel, double *out_1775211612282646777) {
  h_21(state, sat_pos_vel, out_1775211612282646777);
}
void gnss_H_21(double *state, double *sat_pos_vel, double *out_6317595068337282390) {
  H_21(state, sat_pos_vel, out_6317595068337282390);
}
void gnss_predict(double *in_x, double *in_P, double *in_Q, double dt) {
  predict(in_x, in_P, in_Q, dt);
}
}

const EKF gnss = {
  .name = "gnss",
  .kinds = { 6, 20, 7, 21 },
  .feature_kinds = {  },
  .f_fun = gnss_f_fun,
  .F_fun = gnss_F_fun,
  .err_fun = gnss_err_fun,
  .inv_err_fun = gnss_inv_err_fun,
  .H_mod_fun = gnss_H_mod_fun,
  .predict = gnss_predict,
  .hs = {
    { 6, gnss_h_6 },
    { 20, gnss_h_20 },
    { 7, gnss_h_7 },
    { 21, gnss_h_21 },
  },
  .Hs = {
    { 6, gnss_H_6 },
    { 20, gnss_H_20 },
    { 7, gnss_H_7 },
    { 21, gnss_H_21 },
  },
  .updates = {
    { 6, gnss_update_6 },
    { 20, gnss_update_20 },
    { 7, gnss_update_7 },
    { 21, gnss_update_21 },
  },
  .Hes = {
  },
  .sets = {
  },
  .extra_routines = {
  },
};

ekf_init(gnss);
