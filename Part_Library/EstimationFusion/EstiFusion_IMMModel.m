function ExtimateValue = EstiFusion_IMMModel_00(IMMmodel,meas,others)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%函数名称：model = IMMModel(time,dim,x0,w_precision)
%程序说明：用IMM交互式模型滤波，对单个观测结果用多个模型滤波，滤波方法可修改。
%   本例程中中使用的是CKF滤波方法，若使用其他方法，需要删除原滤波方法中的一些
%   程序段。主要有：1）在滤波方法的主程序部分删除多时刻的相关代码
%                  2）在滤波子函数中添加似然函数计算的代码
%                  3）在每个模型滤波前将上一时刻的滤波结果用模型的x0，p0参数传入
%  需要注意的是：其他的融合方法是用多个传感器的量测值融合，本方法是用多个模型单个
% 传感器的数据融合，也可以根据需要用多个模型和多个传感器的数据进行融合。
%输入参数说明：1、IMMmodel IMM模型及其相关参数
%                   IMMmodel.pai = 模型转移Markov链;
%                   IMMmodel.U0 = 模型概率;
%                   IMMmodel.count模型集中模型数量
%             2、LocalEstiData 子模型的滤波数据
%                   LocalEstiData.ModelData(i).X 子模型的滤波结果
%                   LocalEstiData.ModelData(i).P 子模型滤波结果协方差阵
%输出参数说明：ExtimateValue 融合后的数据
%               Estimate_value.X 所有时刻的滤波结果，格式为1xL的矩阵胞组
%                          每个胞组中是Nx1维的量测数据
%               Estimate_value.P 所有时刻的误差协方差阵，格式为1xL的矩阵胞组
%版本说明：1.0 （2020-12-16 CRB 18235107312）    建立文件
%           1.1 (2020-12-21 CRB )  修改bug
%版权说明：西工大精导所拥有本程序所有权，仅供学习使用
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    % 转移参数
    Pai = IMMmodel.pai;
    U = IMMmodel.U0;
    Count = IMMmodel.count;
    L = meas.L;
    Z = meas.Z;
    X_fun = others.X_fun;
    Z_fun = others.Z_fun;
    % 变量预分配存储空间
    Est_P_temp = cell(1,Count);
    Est_temp = cell(1,Count);
    Model = cell(1,Count);
    Like = zeros(Count,1);
    Est_mixed = cell(1,Count);
    Est_P_mixed = cell(1,Count);
    % 给定滤波器初值
    for i=1:Count
        Est_temp{i} = IMMmodel.model{i}.x0;
        Est_P_temp{i} = IMMmodel.model{i}.p0;
        Model{i} = IMMmodel.model{i};
    end
    %% 滤波
    for k=1:L
        % 混合概率计算 
        C_ = Pai'*U; % nx1矩阵
        U = (1./C_)'.*Pai.*U;  
        
        % 混合估计
        for j = 1:Count     %重初始化的状态混合估计
            Est_mixed{j} = 0;
            for i = 1:Count
                Est_mixed{j} = Est_mixed{j}+Est_temp{i}*U(i,j);              
            end
        end

        for j = 1:Count     %重初始化的协方差阵混合估计
            Est_P_mixed{j} = 0;
            for i = 1:Count
                Xmixed_err = Est_temp{i}-Est_mixed{j};
                Est_P_mixed{j} = Est_P_mixed{j} +(Est_P_temp{i}+Xmixed_err*Xmixed_err')*U(i,j);
            end
        end
        % 模型条件滤波
        for i=1:Count
            % 给滤波方法传上一时刻的滤波结果
            other.count =i;
            Model{i}.x0 = Est_temp{i};
            Model{i}.p0 = Est_P_temp{i};
            % 使用各模型滤波（可替换成其他滤波方法，需要在各个方法中加入似然函数的公式，具体参考measure_update函数中） 
            [Est_temp{i},Est_P_temp{i},Like(i,1)] = Kalman_CKF(Model{i},Z{k}(1:3),X_fun,Z_fun,other);
        end

        % 模型概率更新
        C = C_'*Like;   
        U = Like.*C_/C; %模型匹配概率
        
        % 估计融合
        Est_x = cell2mat(Est_temp)*U;
%         x_err = repmat(Est_x,1,Count)-cell2mat(Est_temp);
        Est_P = 0;        
        for i=1:Count
            x_err = Est_x-Est_temp{i};
            Est_P = Est_P+(Est_P_temp{i}+x_err*x_err')*U(i);
        end
        % 存储结果
        ExtimateValue.X{k} = Est_x;
        ExtimateValue.P{k} = Est_P;
        ExtimateValue.U{k} = U;

    end
end

%% CKF滤波算法
function [Est_temp,P_temp,Like] = Kalman_CKF(model,measure_value,X_fun,Z_fun,others)
    
    Est_temp = model.x0;
    P_temp = model.p0;

    %%% 以下程序只需改动other参数，其他部分根据需要改动。
    [X_pre,P_pre] = x_time_update(X_fun,Est_temp,P_temp,model,others);      %时间更新-状态预测
%     others.SensorPos = measure_value.SensorPos{k};
    others.k = 1;
    [Z_pre,P_zz,P_xz] = z_time_update(Z_fun,X_pre,P_pre,model.R,others);      %时间更新-量测预测
    [Est_temp,P_temp,Like] = measure_update(measure_value,X_pre,P_pre,Z_pre,P_zz,P_xz);%测量更新     

end

%% 系统状态预测
function [X_Pre,P_Pre] = x_time_update(X_fun,x_est,P_temp,model,others)  
    Q = model.Q;
    
    n_x =length(x_est);
    C = sqrt(n_x)*[eye(n_x) -eye(n_x)];  %%构造容积点集
    S_est = chol(P_temp,'lower');
    x_EstiTemp_mat = repmat(x_est,1,2*n_x);
    x_CabutPointSet = S_est*C+x_EstiTemp_mat;    %计算容积点集
    X_temp =X_fun(x_CabutPointSet,model,others);              %计算传导容积点集
    X_Pre = X_temp*ones(2*n_x,1)/(2*n_x);        %估计状态向量的一步预测值
    P_Pre= X_temp*X_temp'/(2*n_x)-X_Pre*X_Pre'+Q;   %估计误差协方差一步预测值
end

%% 量测预测
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

%% 滤波
function [est_temp,P_temp,Like] = measure_update(Z_meas,x_pre,P_pre,z_pre,P_zz,P_xz)

    K = P_xz/P_zz;                        %计算卡尔曼增益矩阵
    Z_err = Z_meas - z_pre;
    est_temp = x_pre+ K*Z_err; %估计当前时刻的状态向量
    P_temp = P_pre - K*P_zz*K';           %估计当前时刻的误差协方差矩阵
    
    Like = det(2*3.1415926*P_zz)^(-0.5)*exp(-0.5*Z_err'/P_zz*Z_err); %似然函数，需要添加
end