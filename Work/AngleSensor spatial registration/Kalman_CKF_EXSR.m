function Estimate_value = Kalman_CKF_EXSR(model,measure_value,X_fun,Z_fun)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%程序名称: Kalman_CKF_EXSR(model,measure_value,X_fun,Z_fun)
%程序说明:容积卡尔曼滤波方程-- 适用于EXSR程序
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
%输出说明：est_x_update 滤波后的数据
%           est_x_update.X 所有时刻的滤波结果，格式为1xL的矩阵胞组
%                          每个胞组中是Nx1维的量测数据
%           est_x_update.P 所有时刻的误差协方差阵，格式为1xL的矩阵胞组
%                          每个胞组中是NxN维的量测数据（数据融合可能会用到）
%版本说明:1.0 （2020-12-08 CRB 18235107312）建立文件 继承自Kalman_CKF_10
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    L = measure_value.L;
    Est_x_update = cell(1,L);  %转换矩阵方便
    P_update = cell(1,L);
    Est_temp = model.x0;
    P_temp = model.p0;
    Other.id =12;
    
    for k=1:L
        %%% 以下程序除三个函数和Est_x_update、P_update外其他根据需要改动
        [X_pre,P_pre] = x_time_update(X_fun,Est_temp,P_temp,model);      %时间更新-状态预测
        % Other.AngleErr = measure_value.AngleErr{k};
        Other.SensorPos = measure_value.SensorPos{k};
        [Z_pre,P_zz,P_xz] = z_time_update(Z_fun,X_pre,P_pre,model.R,Other);      %时间更新-量测预测
        [Est_temp,P_temp] = measure_update(measure_value.Z{k},X_pre,P_pre,Z_pre,P_zz,P_xz);%测量更新        
        Est_x_update{k}=Est_temp;
        P_update{k}=P_temp;
    end 
    %%%输出结果
    Estimate_value.X =Est_x_update;
    Estimate_value.P =P_update;
end

%%% 以下程序经过多次使用，适应性强，基本无需改动
%系统状态预测
function [X_Pre,P_Pre] = x_time_update(X_fun,x_est,P_temp,model)  
    Q = model.Q;
    
    n_x =length(x_est);
    C = sqrt(n_x)*[eye(n_x) -eye(n_x)];  %%构造容积点集
    S_est = chol(P_temp,'lower');
    x_EstiTemp_mat = repmat(x_est,1,2*n_x);
    x_CabutPointSet = S_est*C+x_EstiTemp_mat;    %计算容积点集
    X_temp =X_fun(x_CabutPointSet,model);              %计算传导容积点集
    X_Pre = X_temp*ones(2*n_x,1)/(2*n_x);        %估计状态向量的一步预测值
    P_Pre= X_temp*X_temp'/(2*n_x)-X_Pre*X_Pre'+Q;   %估计误差协方差一步预测值
end

%量测预测
function [Z_Pre,P_zz,P_xz] = z_time_update(Z_fun,X_pre,P_pre,R,other)

    n_x =length(X_pre);
    C =sqrt(n_x)*[eye(n_x) -eye(n_x)];      %构造容积点集
    S_Pre =chol(P_pre,'lower');             
    X_Pre_mat =repmat(X_pre,1,2*n_x);       
    x_temp =S_Pre*C+X_Pre_mat;                %计算容积点集
    Z_temp =Z_fun(x_temp,other); %计算传导容积点集
    Z_Pre = Z_temp*ones(2*n_x,1)/(2*n_x);     %估计量测的一步预测值
    P_zz= Z_temp*Z_temp'/(2*n_x)-Z_Pre*Z_Pre'+R;  %估计新息协方差矩阵
    P_xz = x_temp*Z_temp'/(2*n_x)-X_pre*Z_Pre';   %估计交叉协方差矩阵的一步预测值
end

%滤波
function [est_temp,P_temp] = measure_update(Z_meas,x_pre,P_pre,z_pre,P_zz,P_xz)

    K = P_xz/P_zz;                        %计算卡尔曼增益矩阵
    est_temp = x_pre+ K*(Z_meas - z_pre); %估计当前时刻的状态向量
    P_temp = P_pre - K*P_zz*K';           %估计当前时刻的误差协方差矩阵
end
