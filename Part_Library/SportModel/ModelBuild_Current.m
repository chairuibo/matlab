function model = ModelBuild_Current
% dynamical model parameters (CA model)
%           [ 0  1   0  ]       [ 0 ]           [ 0 ]
%  x(k+1/k)=|    0   1  | x(k)+ | 0 |alpha(k) + | 0 |w(k+1)
%           [      -alpha]      [ 1 ]           [ 1 ]

model.T = 1;
model.sigma_v = 0.5;
model.D= diag(1);  
%target initial states 
model.x0=[300000; 00; 20];
model.p0=diag([0 0 0 ]);
model.ObserveBeita = 0.01;
model.ObserveStaticError=30;
% basic parameters
model.x_dim= 3;   %dimension of state vector
model.z_dim= 1;   %dimension of observation vector
model.v_dim= 1;   %dimension of prSocess noise
model.w_dim= 3;   %dimension of observation noise
model.alpha=0.1;
model.sigma_v = sqrt(2*model.alpha*model.sigma_v);

%dynamical model parameters         
Alpha_T = model.alpha*model.T;
Exp_Alpha_T = exp(-Alpha_T);
model.F= [1 model.T  (Exp_Alpha_T+Alpha_T-1)/(model.alpha^2)
          0     1    (1-Exp_Alpha_T)/model.alpha
          0     0    Exp_Alpha_T];  %×ªÒÆ¾ØÕó
model.U= [((1-Exp_Alpha_T)/model.alpha+Alpha_T*model.T/2-model.T)/model.alpha
          model.T-(1-Exp_Alpha_T)/model.alpha
          1-Exp_Alpha_T];  %ÏµÍ³ÔëÉù¾ØÕó     

q11 = (1-exp(-Alpha_T*2)+2*Alpha_T+2/3*Alpha_T^3-2*Alpha_T^2-4*Alpha_T*Exp_Alpha_T)/(2*model.alpha^5);
q12 = (exp(-Alpha_T*2)+1-2*Exp_Alpha_T+2*Alpha_T*Exp_Alpha_T-2*Alpha_T+Alpha_T^2)/(2*model.alpha^4);
q13 = (1-exp(-Alpha_T*2)-2*Alpha_T*Exp_Alpha_T)/(2*model.alpha^3);
q22 = (4*Exp_Alpha_T-3-exp(-Alpha_T*2)+2*Alpha_T)/(2*model.alpha^3);
q23 = (exp(-Alpha_T*2)+1-2*Exp_Alpha_T)/(2*model.alpha^2);
q33 = (1-exp(-Alpha_T*2))/(2*model.alpha);

model.B = [q11 q12 q13
           q12 q22 q23
           q13 q23 q33];

model.Q= 2*model.alpha*(model.sigma_v)^2* model.B;   %process noise covariance
% observation model parameters 
model.H= [ 1 0 0 ];    %observation matrix
model.R= model.D*model.D'; %observation noise covariance

