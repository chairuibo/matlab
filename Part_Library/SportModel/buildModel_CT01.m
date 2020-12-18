function model = buildModel_CT01(time,dim,x0,omiga,w_precision)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%函数名称：model = buildModel_CA(time,dim,x0,w_precision)
%程序说明：目标角速度未知的CT运动模型，模型的观测噪声矩阵需要在外部给定
%输入参数说明：1、time  采样周期
%             2、dim 模型状态变量个数
%             3、x0 系统初值
%             4、Omiga 目标的角速度
%             4、w_precision 系统误差(需要给出各个维数的参数)
%输出参数说明：model 建立的模型
%               model.T 采样周期
%               model.F 状态转移矩阵
%               model.Q 系统误差矩阵
%               model.H 观测矩阵
%               model.x0状态向量初始值
%               model.p0.^2;协方差矩阵初始值
%版本说明：1.0 （2019-12-25 CRB 18235107312）    建立文件
%版权说明：西工大精导所拥有本程序所有权，仅供学习使用
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    % dynamical model parameters (CT model)
    model.T= time;                %sampling period
    model.Sigma_v = w_precision;  
    model.Omiga = omiga;
    % initial
    model.x0=x0;                  %state initial value
    
    % basic parameters
    model.x_dim= dim;   %dimension of state vector        
    model.sinwT = sin(model.Omiga*model.T);
    model.coswT = cos(model.Omiga*model.T);
    
    %multi-state variables 
    if model.x_dim ==2
        model.p0=diag([100 1]).^2; %process noise covariance    
        model.p0 = [model.p0 zeros(3) 
                    zeros(3) model.p0 ];
        model.B0= [(model.T^2)/2; model.T];  %process noise parameters
        model.H0 = [ 1 0 ];    %observation matrix
        
        model.F = [1    model.sinwT/model.Omiga     0   -(1-model.coswT)/model.Omiga 
                   0    model.coswT                 0   -model.sinwT
                   0    (1-model.coswT)/model.Omiga 1   model.sinwT/model.Omiga
                   0    model.sinwT                 0   model.coswT];
        
        model.B = [model.Sigma_v(1)*model.B0  zeros(2,1) 
                   zeros(2,1)   model.Sigma_v(2)*model.B0  ];
        model.H = [ model.H0      zeros(1,2)
                    zeros(1,2)    model.H0 ];    %observation matrix
    elseif model.x_dim ==3
        model.F= [1     model.sinwT/model.Omiga     (1-model.sinwT)/(model.Omiga^2)
                  0     model.coswT                 model.sinwT/model.Omiga
                  0     -model.sinwT*model.Omiga    model.coswT ];  %state transfer matrix
        model.B0= [(model.T^3)/6; (model.T^2)/2; model.T];  %process noise parameters
        model.H0 = [ 1 0 0 ];    %observation matrix
        model.p0=diag([100 10 1]).^2; %process noise covariance
        model.p0=[model.p0 zeros(3) zeros(3)
                  zeros(3) model.p0   zeros(3)   
                  zeros(3) zeros(3)   model.p0];
        model.F = [model.F     zeros(3)    zeros(3)
                   zeros(3)    model.F    zeros(3)
                   zeros(3)    zeros(3)   model.F];
        model.B = [model.Sigma_v(1)*model.B0      zeros(3,1)  zeros(3,1)
                   zeros(3,1)   model.Sigma_v(2)*model.B0     zeros(3,1)
                   zeros(3,1)   zeros(3,1)  model.Sigma_v(3)*model.B0];    
        model.H = [ model.H0       zeros(1,3)    zeros(1,3)
                    zeros(1,3)    model.H0       zeros(1,3)
                    zeros(1,3)    zeros(1,3)    model.H0];    %observation matrix
    end
    model.Q= model.B*model.B';   %process noise covariance
end
