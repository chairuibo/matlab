function model = buildModel_CA(time,dim,x0,w_precision)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%函数名称：model = buildModel_CA(time,dim,precision)
%程序说明：创建目标运动模型
%输入参数说明：1、time  采样周期
%             2、dim 模型状态变量个数
%             3、x0 系统初值
%             4、w_precision 系统误差
%输出参数说明：model 建立的模型
%               model.T 采样周期
%               model.F 状态转移矩阵
%               model.Q 系统误差矩阵
%               model.R 量测误差矩阵
%               model.H 观测矩阵
%               model.x0状态向量初始值
%               model.p0.^2;协方差矩阵初始值
%版本说明：1.0 （2019-12-25 CRB 18235107312）    建立文件
%         1.1（2019-12-29 CRB）  添加参数，修改部分结构，改单轴参数为可选轴数
%         1.2 (2020-06-23 CRB)   添加参数
%		  1.3 (2020-12-08 CRB)	 删除量测噪声方差及相关参数，改为函数外赋值
%版权说明：西工大精导所拥有本程序所有权，仅供学习使用
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    % dynamical model parameters (CA model)
    model.T= time;                %采样周期
    model.sigma_v = w_precision;  %系统噪声值
    % initial
    model.x0=x0;                  %系统初值
    model.p0=diag([100 10 1]).^2; %系统初始协方差阵
    % basic parameters
    model.x_dim= dim*3;   %dimension of state vector
    model.w_dim= dim*3;   %dimension of observation noise
    %dynamical model parameters         
    model.F= [1 model.T (model.T^2)/2
              0     1   model.T
              0     0      1];  %转移矩阵
    model.B0= [(model.T^3)/6; (model.T^2)/2; model.T];  %系统噪声矩阵
    % observation model parameters 
    model.H= [ 1 0 0 ];    %observation matrix
    %multi-state variables 
    if dim ==1
        model.B0= model.sigma_v*model.B;
    elseif dim ==2
        model.p0 = [model.p0 zeros(3) 
                    zeros(3) model.p0 ];
        model.F = [model.F    zeros(3)   
                    zeros(3)  model.F ];
        model.B = [model.sigma_v(1)*model.B0  zeros(3,1) 
                   zeros(3,1)   model.sigma_v(2)*model.B0  ];    
    elseif dim ==3
        model.p0=[model.p0 zeros(3) zeros(3)
                  zeros(3) model.p0   zeros(3)   
                  zeros(3) zeros(3)   model.p0];
        model.F = [model.F     zeros(3)    zeros(3)
                   zeros(3)    model.F    zeros(3)
                   zeros(3)    zeros(3)   model.F];
        model.B = [model.sigma_v(1)*model.B0      zeros(3,1)  zeros(3,1)
                   zeros(3,1)   model.sigma_v(2)*model.B0     zeros(3,1)
                   zeros(3,1)   zeros(3,1)  model.sigma_v(3)*model.B0];    
    end
    model.Q= model.B*model.B';   %process noise covariance
end
