function model = ModelBuild_Singer
% dynamical model parameters (CA model)
model.T= 1;   
model.sigma_v = 5;
% initial value
model.x0=[0.1; 0; 0];
model.p0=diag([2 2 2]).^2;
% basic parameters
model.x_dim= 3;   %dimension of state vector
model.z_dim= 1;   %dimension of observation vector
model.v_dim= 1;   %dimension of prSocess noise
model.w_dim= 3;   %dimension of observation noise
model.alpha=0.05;
model.sigma_v = sqrt(2*model.alpha*model.sigma_v);
%dynamical model parameters         
Alpha_T = model.alpha*model.T;
Exp_Alpha_T = exp(-Alpha_T);

model.F= [1 model.T  (Exp_Alpha_T+Alpha_T-1)/(model.alpha^2)
          0     1    (1-Exp_Alpha_T)/model.alpha
          0     0    Exp_Alpha_T];  %×ªÒÆ¾ØÕó

model.B= [((1-Exp_Alpha_T)/model.alpha+Alpha_T*model.T/2-model.T)/(model.alpha^2)
          (model.T-(1-Exp_Alpha_T)/model.alpha)/model.alpha
          (1-Exp_Alpha_T)/model.alpha];  %ÏµÍ³ÔëÉù¾ØÕó

model.Q= (model.sigma_v)^2* model.B*model.B';   %process noise covariance
% observation model parameters 
model.H= [ 1 0 0 ];    %observation matrix
model.D= diag(10); 
model.R= model.D*model.D'; %observation noise covariance
end

