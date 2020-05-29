 function EstimateValue = Kalman_predict(model,meas)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%程序说明:基本卡尔曼预测方程
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
%                   2)每个传感器的数据按列排列
%                   3)不同传感器按行排列
%输出数据说明：EstimateValue  输出预测结果
%     输出格式：1)每个时刻的结果放在一个元素胞组内
%              2)每个胞组中状态向量按列排列
%版本说明:1.0 （2020-4-8 CRB 18235107312）建立文件
%版权信息：西工大精导所拥有本程序所有权，仅供学习使用
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    L = meas.L;
    x_predict = cell(1,L+1);
    P_predict = cell(1,L+1);
    x_predict{1}= model.x0;
    P_predict{1}= model.p0.^2;

    for k=2:L
         %求x(k/k-1)和P(k/k-1)
         [x_predict{k},P_predict{k}] = predict_multiple(model,x_predict{k-1},P_predict{k-1},meas.Z{k});    
    end 
    %估计出的数值为estimate_value的第二列到最后一列，第一列为初始数据
    EstimateValue.x = x_predict(2:k+1);
end

%Kalman predict
function [x_predict,P_predict] = predict_multiple(model,x,P,z)     
    F = model.F;
    Q = model.Q;
    H = model.H;
    R = model.R;
    
    K = F*P*H'/(H*P*H'+R);
    x_predict = F*x+K*(z-H*x);
    P_predict = F*P*F'-K*H*P*F'+Q; 
end
