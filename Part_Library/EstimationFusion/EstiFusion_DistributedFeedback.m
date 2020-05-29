function [ kalman_x_update,estimate_data ] = EstiFusion_DistributedFeedback(model,measureData )
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%程序说明:分布式融合系统，带反馈的最优分布式估计融合算法
%参数说明:model  运动模型
%        measure_value 所有传感器的量测数据
%          -measure_value数据格式要求：包含三个部分 
%          -L为数据长度，
%          -count为传感器数量
%          -data为每个传感器的量测值
%版本说明：1.0 （2020-01-13 CRB）    建立文件
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    SensorCount = measureData.SensorCount;
    Data_L = measureData.L;
    estimate_data = cell(Data_L+1,1);
    kalman_x_update = cell(Data_L+1,SensorCount);
    kalman_x_predict = cell(1,SensorCount);
    kalman_P_predict = cell(1,SensorCount);
    kalman_P_update = cell(1,SensorCount);
    
 %%%整理数据   
    for j = 1:SensorCount 
        kalman_x_update{1,j}= model(j).x0;
        kalman_P_update{j}= model(j).p0;
%             measure_data.z{i,j} = measureData.data(j).z{i};
    end
    InvP = model(j).p0;
    estimate_data{1} = model(j).x0;
    
 %%%滤波主程序  
    for k=1:Data_L
        for j = 1:SensorCount
            [kalman_x_predict{j},kalman_P_predict{j}] = predict_multiple(model(j),kalman_x_update{k,j},kalman_P_update{j});    
            [kalman_x_update{k+1,j},kalman_P_update{j}] = update_multiple(model(j),measureData.data(j).Z{k},kalman_x_predict{j},kalman_P_predict{j});
        end
        SensorData.kalman_x_predict = kalman_x_predict;
        SensorData.kalman_P_predict = kalman_P_predict;
        SensorData.kalman_x_update = kalman_x_update(k+1,:);
        SensorData.kalman_P_update = kalman_P_update;
        [estimate_data{k+1},InvP ]= SingleStepMultiple(model(j),estimate_data{k},InvP,SensorData);
        for j = 1:SensorCount
            kalman_x_update{k+1,j}= estimate_data{k+1};
            kalman_P_update{j} = InvP;
        end
    end
    estimate_data = estimate_data(2:k+1);
    kalman_x_update = kalman_x_update(2:k+1);
end

%%%单步计算,算法基本单元
function  [x_out,P_out]= SingleStepMultiple(model,x_Global_update,P_Global_update,SensorData)
 %%%整理数据   
    x_Gu = x_Global_update;
    P_Gu = P_Global_update;
    x_2 = SensorData.kalman_x_update;
    invP_2 = SensorData.kalman_P_update;
    SensorCount = length(x_2);
    invP_temp = 0;
    invPx_temp = 0;
    
 %%%kernel     
    invP_Gp = inv(model.F/P_Gu*model.F'+model.Q);
    x_Gp =  model.F * x_Gu;
    for j=1:SensorCount
        invP_temp = invP_temp + invP_2{j};
        invPx_temp = invPx_temp + invP_2{j}*x_2{j};
    end
    P_out = invP_temp - (SensorCount-1).*invP_Gp ;
    x_out = P_out\invPx_temp - (SensorCount-1).*P_out\invP_Gp*x_Gp ;
end

%Kalman predict
function [x_predict,inv_P_predict] = predict_multiple(model,x,P)     
    F = model.F;
    Q = model.Q;

    x_predict = F*x;
    inv_P_predict = inv(Q+F/P*F'); 
end

%Kalman update
function [x_update,inv_P_update] = update_multiple(model,measure_Z,xPredict,inv_PPredict)
    H = model.H;
    R = model.R;
    
    Z_Predict = H*xPredict;
    inv_P = inv_PPredict + H'/R*H;
    K  = inv_P\H'/R;
    x_update = xPredict + K*(measure_Z-Z_Predict);
    inv_P_update = inv_P;
end
