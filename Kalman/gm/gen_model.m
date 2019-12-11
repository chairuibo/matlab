function model= gen_model

% basic parameters
model.x_dim= 4;   %dimension of state vector
model.z_dim= 2;   %dimension of observation vector

% dynamical model parameters (CV model)
model.T= 0.1;                                     %sampling period
model.A0= [ 1 model.T; 0 1 ];                      %transition matrix                     
model.F= [ model.A0 zeros(2,2); zeros(2,2) model.A0 ];  %×ªÒÆ¾ØÕó
model.B0= [ (model.T^2)/2; model.T ];
model.B= [ model.B0 zeros(2,1); zeros(2,1) model.B0 ];  %ÏµÍ³ÔëÉùÓ°Ïì¾ØÕó
model.sigma_v = 5;
model.Q= (model.sigma_v)^2* model.B*model.B';   %process noise covariance
 
% observation model parameters (noisy x/y only)
model.H= [ 1 0 0 0 ; 0 0 1 0 ];    %observation matrix
model.D= diag([ 10; 10 ]); 
model.R= model.D*model.D';         %observation noise covariance

model.x0=[0.1;0;0.1;0];
model.p0=diag([2 2 2 2]).^2;
