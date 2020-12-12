function model = buildModel_Singer(time,dim,x0,alpha,w_precision)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%函数名称：model = buildModel_Singer(time,dim,x0,w_precision)
%程序说明：创建CV运动模型,模型的观测噪声矩阵需要在外部给定
%参数说明：1、time  采样时间
%         2、dim 模型状态变量个数
%         3、x0 初值
%         4、alpha 机动时间常数的倒数，如1/60为目标机动大转弯 1/20~30 为目标回避操作
%         5、w_precision 系统误差 sigma的平方 
%               计算公式：w_precision=A_max^2/3*(1+4*P_max-P0) A_max 目标最大加速度
%             P_max 目标最大加速的概率       P0  目标没有加速的概率
%版本说明   1.0 （2019-12-25 CRB 18235107312）    建立文件
%           1.1 （2020-01-13 CRB）    增加目标状态维数，整理代码风格
%版权说明：西工大精导所拥有本程序所有权，仅供学习使用
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    % dynamical model parameters (Singer model)
    model.T= time;   
    model.sigma_v = w_precision;
    model.alpha=alpha; 
    model.AS = model.sigma_v.*model.alpha;
    % initial value
    model.x0=x0;
    model.p0=diag([2 2 2]).^2;
    % basic parameters
    model.x_dim = dim;   %dimension of state vector

    %dynamical model parameters         
    Alpha_T = model.alpha*model.T;
    Exp_Alpha_T = exp(-Alpha_T);
    model.F = [1 model.T  (Exp_Alpha_T+Alpha_T-1)/(model.alpha^2)
              0     1    (1-Exp_Alpha_T)/model.alpha
              0     0    Exp_Alpha_T];  %state transfer matrix
          
    q11 = (1-exp(-Alpha_T*2)+2*Alpha_T+2/3*Alpha_T^3-2*Alpha_T^2-4*Alpha_T*Exp_Alpha_T)/(2*model.alpha^5);
    q12 = (exp(-Alpha_T*2)+1-2*Exp_Alpha_T+2*Alpha_T*Exp_Alpha_T-2*Alpha_T+Alpha_T^2)/(2*model.alpha^4);
    q13 = (1-exp(-Alpha_T*2)-2*Alpha_T*Exp_Alpha_T)/(2*model.alpha^3);
    q22 = (4*Exp_Alpha_T-3-exp(-Alpha_T*2)+2*Alpha_T)/(2*model.alpha^3);
    q23 = (exp(-Alpha_T*2)+1-2*Exp_Alpha_T)/(2*model.alpha^2);
    q33 = (1-exp(-Alpha_T*2))/(2*model.alpha);          
    model.B0 = [q11 q12 q13
           q12 q22 q23
           q13 q23 q33];   %process noise covariance
    model.H= [ 1 0 0 ];    %observation matrix

    %multi-state variables 
    if model.x_dim ==1
        model.B = model.AS*model.B0;   %process noise covariance
    elseif model.x_dim ==2
        model.p0 = [model.p0 zeros(3) 
                    zeros(3) model.p0 ];
        model.F = [model.F    zeros(3)   
                    zeros(3)  model.F ];
        model.B = [model.AS(1)*model.B0  zeros(3,1) 
                   zeros(3,1)   model.AS(2)*model.B0  ];
        model.H = [ model.H     zeros(1,3)
                    zeros(1,3)    model.H ];    %observation matrix
    elseif model.x_dim ==3        
        model.p0=[model.p0 zeros(3) zeros(3)
                  zeros(3) model.p0   zeros(3)   
                  zeros(3) zeros(3)   model.p0];
        model.F = [model.F     zeros(3)    zeros(3)
                   zeros(3)    model.F    zeros(3)
                   zeros(3)    zeros(3)   model.F];
        model.B = [model.AS(1)*model.B0      zeros(3,1)  zeros(3,1)
                   zeros(3,1)   mmodel.AS*model.B0     zeros(3,1)
                   zeros(3,1)   zeros(3,1)  model.AS*model.B0];    
        model.H = [ model.H       zeros(1,3)    zeros(1,3)
                    zeros(1,3)    model.H       zeros(1,3)
                    zeros(1,3)    zeros(1,3)    model.H];    %observation matrix
    end
    model.Q= 2*model.B;   %process noise covariance
end
