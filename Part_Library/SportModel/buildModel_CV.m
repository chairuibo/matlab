function model = buildModel_CV(time,dim,x0,w_precision)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%函数名称：model = buildModel_CV(time,dim,x0,w_precision)
%程序说明：创建CV运动模型,模型的观测噪声矩阵需要在外部给定
%参数说明：1、time  采样时间
%         2、dim 模型状态变量个数
%         3、x0 初值
%         4、w_precision 系统误差(需要给出各个维数的参数)
%版本说明   1.0 （2019-12-25 CRB）    建立文件
%           1.1 （2020-01-13 CRB）  添加量测噪声参数，修改部分结构，改单轴参数为可选轴数
%           1.2 （2020-05-24 CRB）  添加系统误差参数
%           1.3  (2020-12-12 CRB)   整理代码风格
%版权说明：西工大精导所拥有本程序所有权，仅供学习使用
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    % dynamical model parameters (CV model)
    model.T= time;   
    model.sigma_v = w_precision; %process noise parameter
    % initial value
    model.x0=x0;
    model.p0=diag([100 100]).^2;
    % basic parameters
    model.x_dim= dim;   %dimension of state vector
    %dynamical model parameters         
    model.F= [1  model.T
              0     1  ];  %state transfer matrix
    model.B= [(model.T^2)/2; model.T];  
    model.Q= (model.sigma_v)^2* model.B*model.B';   %process noise covariance
    % observation model parameters 
    model.H= [ 1 0 ];    %observation matrix

    %multi-state variables 
    if model.x_dim == 1
        model.B0= model.sigma_v*model.B;
    elseif model.x_dim ==2
        model.p0 = [model.p0 zeros(2) 
                    zeros(2) model.p0 ];
        model.F = [model.F    zeros(2)   
                    zeros(2)  model.F ];
        model.B = [model.sigma_v(1)*model.B  zeros(2,1) 
                   zeros(2,1)   model.sigma_v(2)*model.B  ];    
        model.H= [model.H   zeros(1,2) 
                  zeros(1,2)   model.H   ];
    elseif model.x_dim ==3
        model.p0=[model.p0  zeros(2) zeros(2)
                  zeros(2)  model.p0  zeros(2)   
                  zeros(2)  zeros(2)  model.p0];
        model.F = [model.F     zeros(2)    zeros(2)
                   zeros(2)    model.F    zeros(2)
                   zeros(2)    zeros(2)   model.F];
        model.B = [model.sigma_v(1)*model.B      zeros(2,1)  zeros(2,1)
                   zeros(2,1)   model.sigma_v(2)*model.B     zeros(2,1)
                   zeros(2,1)   zeros(2,1)  model.sigma_v(3)*model.B];    
        model.H= [model.H       zeros(1,2)  zeros(1,2)
                  zeros(1,2)   model.H     zeros(1,2)
                  zeros(1,2)   zeros(1,2)  model.H];
    end
    model.Q= model.B*model.B';   %process noise covariance
end
