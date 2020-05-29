function estimate_value = Kalman_UKF(model, measure_value,X_fun,Z_fun,UT)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%程序说明:基本卡尔曼滤波方程
%参数说明:1、model  运动模型
%          model.Q 系统噪声协方差阵
%          model.R 观测噪声协方差阵
%          model.p0 P阵初始值
%          model.x0 状态向量初试值
%        2、measure_value  所有传感器的量测数据
%           measure_value.L  量测数据的长度
%           measure_value.Z  所有量测数据，格式为1xL的矩阵胞组
%                            每个胞组中是Nx1维的量测数据
%        3、X_fun 系统方程函数
%        4、Z_fun 量测方程函数
%        5、UT UT变换需要用到的参数Alpha Lamda Kappa
%            UT.Alpha UT.Lamda(可有可无，按需修改) UT.Kappa
%输出说明：est_x_update 滤波后的数据
%           est_x_update.X 所有时刻的滤波结果，格式为1xL的矩阵胞组
%                          每个胞组中是Nx1维的量测数据
%           est_x_update.P 所有时刻的误差协方差阵，格式为1xL的矩阵胞组
%                          每个胞组中是NxN维的量测数据（数据融合可能会用到）
%版本说明:1.0 （2020-5-28 CRB 18235107312）建立文件
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    L = measure_value.L;
    est_x_update = cell(1,L+1);  %转换矩阵方便
    P_update = cell(1,L+1);

    est_temp = model.x0;
    P_temp = model.p0;
    for k=1:L
         [X_temp,w]=ut(est_temp,P_temp,UT);                             %UT变换
         [X_pre,P_pre,X_] = x_time_update(X_fun,X_temp,w,model.Q);      %时间更新-状态预测
         [Z_pre,P_zz,P_xz] = z_time_update(Z_fun,X_temp,X_,w,model.R);  %时间更新-量测预测
         [est_temp,P_temp] = measure_update(measure_value.Z{k},X_pre,P_pre,Z_pre,P_zz,P_xz);%测量更新
         est_x_update{k}=est_temp;
         P_update{k}=P_temp;
    end 
    %%%输出结果
    estimate_value.X =est_x_update;
    estimate_value.P =P_update;
end

%%%UT变换  输出结果  状态变量的UT变换结果和加权系数
function [X_temp,w]= ut(x,P,UT)
    alpha =UT.Alpha;
    kappa =UT.Kappa;
%     lambda =UT.Lamda;

    n_x= length(x);         %状态的维度
    lambda= alpha^2*(n_x+kappa) - n_x;
    d=(n_x+lambda)*P;       
    Psqrtm= chol(d)';       %P阵的cholesterol分解
    X_temp= repmat(x,[1 2*n_x+1])+ [ zeros(n_x,1) Psqrtm -Psqrtm ];
    w= [ lambda 0.5*ones(1,2*n_x) ]/(n_x+lambda);   %权值系数
 end

%系统状态预测
function [X_Pre,P_Pre,X_] = x_time_update(X_fun,x_temp,w,Q)  

    X_temp =X_fun(x_temp);
    X_Pre =X_temp*w(:);        %状态向量的预测值
    X_ =X_temp- repmat(X_Pre,[1 length(w)]);   %西格玛点的状态向量预测值与均值的差
    P_Pre =X_*diag(w)*X_'+Q;  %P阵预测值
end

%量测预测
function [Z_Pre,P_zz,P_xz] = z_time_update(Z_fun,x_temp,X_,w,R)

    Z_temp =Z_fun(x_temp);
    Z_Pre = Z_temp*w(:);        %观测量预测值
    Z_ = Z_temp- repmat(Z_Pre,[1 length(w)]); %西格玛点的观测向量预测值与均值的差
    P_zz= Z_*diag(w)*Z_'+R;
    P_xz = X_*diag(w)*Z_';
end

%滤波
function [est_temp,P_temp] = measure_update(Z_meas,x_pre,P_pre,z_pre,P_zz,P_xz)

    K = P_xz/P_zz;                          %估计卡尔曼增益矩阵
    est_temp = x_pre+ K*(Z_meas - z_pre);   %估计当前时刻的状态向量
    P_temp = P_pre - K*P_zz*K';             %估计当前时刻的误差协方差矩阵
end
