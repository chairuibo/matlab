function model = ModelBuild_CV
% dynamical model parameters (CA model)
model.T= 0.5;   
model.sigma_v = 2;
% initial value
model.x0=[0.1; 0];
model.p0=diag([2 2]).^2;
% basic parameters
model.x_dim= 2;   %dimension of state vector
model.z_dim= 1;   %dimension of observation vector
model.v_dim= 1;   %dimension of prSocess noise
model.w_dim= 2;   %dimension of observation noise
%dynamical model parameters         
model.F= [1 model.T
          0     1  ];  %×ªÒÆ¾ØÕó
model.B= [(model.T^2)/2; model.T];  %ÏµÍ³ÔëÉù¾ØÕó
model.Q= (model.sigma_v)^2* model.B*model.B';   %process noise covariance
% observation model parameters 
model.H= [ 1 0 ];    %observayytion matrix
model.D= diag(10); 
model.R= model.D*model.D'; %observation noise covariance

