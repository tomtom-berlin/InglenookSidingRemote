#pragma once

#ifndef CYD_CALIBRATION
#define CYD_CALIBRATION

/******************************************************************
CYD Mac { 0xEC, 0x62, 0x60, 0x34, 0x86, 0x6C }
******************************************************************
USE THE FOLLOWING COEFFICIENT VALUES TO CALIBRATE YOUR TOUCHSCREEN
270°
Computed X:  alpha_x = 0.001, beta_x = 0.089, delta_x = -38.505
Computed Y:  alpha_y = 0.066, beta_y = 0.000, delta_y = -18.271
******************************************************************
90°
Computed X:  alpha_x = -0.001, beta_x = -0.089, delta_x = 351.939
Computed Y:  alpha_y = -0.065, beta_y = -0.000, delta_y = 255.552
******************************************************************

MAC Address: { 0xEC, 0x62, 0x60, 0x34, 0x83, 0xF0 }
******************************************************************
90°
Computed X:  alpha_x = -0.000, beta_x = -0.089, delta_x = 350.770
Computed Y:  alpha_y = -0.066, beta_y = -0.001, delta_y = 258.445
******************************************************************
270°
Computed X:  alpha_x = 0.000, beta_x = 0.089, delta_x = -30.087
Computed Y:  alpha_y = 0.065, beta_y = 0.001, delta_y = -14.959
******************************************************************/

#ifdef CYD_1

#ifdef ROT_270DEG
float alpha_x = 0.001, beta_x = 0.089, delta_x = -38.505;
float alpha_y = 0.066, beta_y = 0.000, delta_y = -18.271;
#endif

#ifdef ROT_90DEG
const float alpha_x = -0.001, beta_x = -0.089, delta_x = 351.939;
const float alpha_y = -0.065, beta_y = -0.000, delta_y = 255.552;
#endif

#endif

#ifdef CYD_2

#ifdef ROT_270DEG
const float alpha_x = 0.000, beta_x = 0.089, delta_x = -30.087;
const float  alpha_y = 0.065, beta_y = 0.001, delta_y = -14.959;
#endif

#ifdef ROT_90DEG
const float alpha_x = -0.000, beta_x = -0.089, delta_x = 350.770;
const float alpha_y = -0.066, beta_y = -0.001, delta_y = 258.445;
#endif

#endif

//---------
#endif
