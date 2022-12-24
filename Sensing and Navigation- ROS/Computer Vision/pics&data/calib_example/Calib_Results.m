% Intrinsic and Extrinsic Camera Parameters
%
% This script file can be directly executed under Matlab to recover the camera intrinsic and extrinsic parameters.
% IMPORTANT: This file contains neither the structure of the calibration objects nor the image coordinates of the calibration points.
%            All those complementary variables are saved in the complete matlab data file Calib_Results.mat.
% For more information regarding the calibration model visit http://www.vision.caltech.edu/bouguetj/calib_doc/


%-- Focal length:
fc = [ 488.171172244663865 ; 486.711704355268523 ];

%-- Principal point:
cc = [ 321.991516895328289 ; 230.598716114822622 ];

%-- Skew coefficient:
alpha_c = 0.000000000000000;

%-- Distortion coefficients:
kc = [ -0.091598084966379 ; 0.158195615122967 ; -0.002643003204609 ; 0.000658086069992 ; 0.000000000000000 ];

%-- Focal length uncertainty:
fc_error = [ 3.170177281987098 ; 2.934352068267027 ];

%-- Principal point uncertainty:
cc_error = [ 2.822675151546350 ; 2.270758530347428 ];

%-- Skew coefficient uncertainty:
alpha_c_error = 0.000000000000000;

%-- Distortion coefficients uncertainty:
kc_error = [ 0.017210323160374 ; 0.052306848002525 ; 0.001491835874363 ; 0.001812726588650 ; 0.000000000000000 ];

%pixel error
err = [ 0.33277   0.29580 ];


%-- Image size:
nx = 640;
ny = 480;


%-- Various other variables (may be ignored if you do not use the Matlab Calibration Toolbox):
%-- Those variables are used to control which intrinsic parameters should be optimized

n_ima = 9;						% Number of calibration images
est_fc = [ 1 ; 1 ];					% Estimation indicator of the two focal variables
est_aspect_ratio = 1;				% Estimation indicator of the aspect ratio fc(2)/fc(1)
center_optim = 1;					% Estimation indicator of the principal point
est_alpha = 0;						% Estimation indicator of the skew coefficient
est_dist = [ 1 ; 1 ; 1 ; 1 ; 0 ];	% Estimation indicator of the distortion coefficients


%-- Extrinsic parameters:
%-- The rotation (omc_kk) and the translation (Tc_kk) vectors for every calibration image and their uncertainties

%-- Image #1:
omc_1 = [ -2.204788e+00 ; -2.196448e+00 ; -5.357482e-03 ];
Tc_1  = [ -3.584629e+02 ; -2.317948e+02 ; 7.809749e+02 ];
omc_error_1 = [ 3.997269e-03 ; 4.087972e-03 ; 8.532847e-03 ];
Tc_error_1  = [ 4.661035e+00 ; 3.814367e+00 ; 5.412863e+00 ];

%-- Image #2:
omc_2 = [ -1.968958e+00 ; -1.951569e+00 ; 3.649720e-01 ];
Tc_2  = [ -3.562922e+02 ; -2.427766e+02 ; 1.035078e+03 ];
omc_error_2 = [ 4.878162e-03 ; 4.464399e-03 ; 8.025631e-03 ];
Tc_error_2  = [ 6.019441e+00 ; 4.896525e+00 ; 6.081413e+00 ];

%-- Image #3:
omc_3 = [ 1.997528e+00 ; 1.927008e+00 ; 6.440912e-01 ];
Tc_3  = [ -2.010799e+02 ; -2.302217e+02 ; 7.191310e+02 ];
omc_error_3 = [ 5.312474e-03 ; 4.023489e-03 ; 8.204545e-03 ];
Tc_error_3  = [ 4.328856e+00 ; 3.436045e+00 ; 5.015317e+00 ];

%-- Image #4:
omc_4 = [ 2.134743e+00 ; 2.034612e+00 ; 8.123923e-01 ];
Tc_4  = [ -2.293064e+02 ; -1.620528e+02 ; 7.721997e+02 ];
omc_error_4 = [ 6.020588e-03 ; 3.632760e-03 ; 8.963577e-03 ];
Tc_error_4  = [ 4.601925e+00 ; 3.712718e+00 ; 5.758610e+00 ];

%-- Image #5:
omc_5 = [ -2.059849e+00 ; -2.028629e+00 ; -4.171027e-01 ];
Tc_5  = [ -3.076883e+02 ; -2.106693e+02 ; 9.400427e+02 ];
omc_error_5 = [ 4.174066e-03 ; 5.423662e-03 ; 9.630860e-03 ];
Tc_error_5  = [ 5.566273e+00 ; 4.556099e+00 ; 6.837455e+00 ];

%-- Image #6:
omc_6 = [ -1.865848e+00 ; -1.860412e+00 ; 2.082076e-01 ];
Tc_6  = [ -3.199517e+02 ; -2.401757e+02 ; 1.084895e+03 ];
omc_error_6 = [ 4.606573e-03 ; 4.955687e-03 ; 7.763923e-03 ];
Tc_error_6  = [ 6.313642e+00 ; 5.127050e+00 ; 6.385436e+00 ];

%-- Image #7:
omc_7 = [ -2.084831e+00 ; -1.876894e+00 ; 6.186960e-01 ];
Tc_7  = [ -3.784644e+02 ; -2.125193e+02 ; 1.130855e+03 ];
omc_error_7 = [ 5.501078e-03 ; 3.841533e-03 ; 8.235755e-03 ];
Tc_error_7  = [ 6.591583e+00 ; 5.340324e+00 ; 6.143353e+00 ];

%-- Image #8:
omc_8 = [ 2.185645e+00 ; 2.131718e+00 ; -1.897153e-01 ];
Tc_8  = [ -3.308686e+02 ; -2.970606e+02 ; 8.511354e+02 ];
omc_error_8 = [ 3.820824e-03 ; 4.409513e-03 ; 8.519010e-03 ];
Tc_error_8  = [ 5.051022e+00 ; 4.035654e+00 ; 5.567043e+00 ];

%-- Image #9:
omc_9 = [ 2.058911e+00 ; 1.956536e+00 ; 4.347506e-01 ];
Tc_9  = [ -1.978370e+02 ; -2.536281e+02 ; 7.846832e+02 ];
omc_error_9 = [ 5.122192e-03 ; 4.201855e-03 ; 8.616986e-03 ];
Tc_error_9  = [ 4.704290e+00 ; 3.721237e+00 ; 5.238136e+00 ];

