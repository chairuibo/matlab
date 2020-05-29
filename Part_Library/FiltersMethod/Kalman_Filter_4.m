 function EstimateValue = Kalman_Filter_4(model,meas)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%程序说明:基本卡尔曼滤波方程
%输入参数说明:1、model  运动模型 
%               model.T 采样周期
%               model.F 状态转移矩阵
%               model.Q 系统误差矩阵
%               model.R 量测误差矩阵
%               model.H 观测矩阵
%               model.x0状态向量初始值
%               model.p0.^2;协方差矩阵初始值
%            2、meas 传感器的量测数据
%               meas.L 数据长度
%               meas.Z 传感器数据，格式为：
%                   1)每个时刻的数据放在一个元素胞组内
%                   2)传感器的数据按行排列
%输出数据说明：EstimateValue  输出滤波结果
%               EstimateValue.x 状态向量的滤波值
%               EstimateValue.x_p 状态向量的预测值
%               EstimateValue.P 估计误差协方差阵P(k/k)
%               EstimateValue.P_p 估计误差协方差阵P(k+1/k)
%     输出格式：1)每个时刻的结果放在一个元素胞组内
%              3)不同时刻数据为一行n列
%              2)每个胞组中状态向量为n行一列
%版本说明:1.0 （2019-12-25 CRB 18235107312）建立文件
%        1.1  (2019-12-26 CRB) 添加更新和预测函数，调整函数结构
%        1.2  (2019-12-29 CRB) 操作单元将矩阵元素改为胞组
%        1.3  (2020-01-14 CRB) 增加输出P阵功能   
%版权信息：西工大精导所拥有本程序所有权，仅供学习使用
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    L = meas.L;
    est_x_update = cell(1,L+1);
    P_update = cell(1,L+1);
    x_predict = cell(1,L);
    P_predict = cell(1,L);
    est_x_update{1}= model.x0;
    P_update{1}= model.p0.^2;

    for k=1:L
         %求x(k/k-1)和P(k/k-1)
         [x_predict{k},P_predict{k}] = predict_multiple(model,est_x_update{k},P_update{k});    
         %求x(k/k)和P(k/k)
         [est_x_update{k+1},P_update{k+1}] = update_multiple(meas.Z{k},model,x_predict{k},P_predict{k});
    end 
    %估计出的数值为estimate_value的第二列到最后一列，第一列为初始数据
    EstimateValue.x = est_x_update(2:k+1);
    EstimateValue.x_p = x_predict;
    EstimateValue.P = P_update(2:k+1);
    EstimateValue.P_p = P_predict;
end

%Kalman predict
function [x_predict,P_predict] = predict_multiple(model,x,P)     
    F = model.F;
    Q = model.Q;

    x_predict = F*x;
    P_predict = Q+F*P*F'; 
end

%Kalman update
function [x_update,P_update] = update_multiple(measure_Z,model,xPredict,PPredict)
    H = model.H;
    P = PPredict;
    R = model.R;
    
    Z_Predict = H*xPredict;
    S  = R+H*P*H'; %Vs= chol(S); det_S= prod(diag(Vs))^2; inv_sqrt_S= inv(Vs); iS= inv_sqrt_S*inv_sqrt_S'; 
    K  = P*H'/S;
    IKH = (eye(size(P))-K*H);
    x_update = xPredict + K*(measure_Z-Z_Predict);
    P_update = IKH*P*IKH'+K*R*K';
end
