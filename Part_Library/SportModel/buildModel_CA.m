function model = buildModel_CA(d)
%参数d为量测精度
% dynamical model parameters (CA model)
    model.T= 0.5;   
    model.sigma_v =2;
    % initial
    model.x0=[30000; 0; -3];
    model.p0=diag([1 1 1]).^2;
    % basic parameters
    model.x_dim= 3;   %dimension of state vector
    model.z_dim= 1;   %dimension of observation vector
    model.v_dim= 1;   %dimension of prSocess noise
    model.w_dim= 3;   %dimension of observation noise
    %dynamical model parameters         
    model.F= [1 model.T (model.T^2)/2
              0     1   model.T
              0     0      1];  %转移矩阵
    model.B= [(model.T^3)/6; (model.T^2)/2; model.T];  %系统噪声矩阵
    model.Q= (model.sigma_v)^2* model.B*model.B';   %process noise covariance
    % observation model parameters 
    model.H= [ 1 0 0 ];    %observation matrix
    model.D= diag(d); 
    model.R= model.D*model.D'; %observation noise covariance
end