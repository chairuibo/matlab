function EstimationValue = AsynChroFusion_highAndlowFrequencies(model,measure_value,sensor_count)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%程序说明   异步融合程序
%参数说明   model  运动模型
%           measure_value 所有传感器的量测数据
%           sensor_count  传感器数量
%版本说明   1.0 （2019-12-29 CRB）    建立文件
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    %initial value
    count = sensor_count;
    L = measure_value.HighFrequence.L;
    est_x_update = cell(1,L+1);
    P_update = cell(1,L+1);
    est_x_update{1}= zeros(length(model(1).x0),1);  
    x_perDimCount = model(1).x_dim/model(1).z_dim;  %x每个维度状态个数
    for i = 1:model(1).z_dim
        est_x_update{1}((i-1)*x_perDimCount+1) = measure_value.HighFrequence.Z{1}(i);
    end
    P_update{1}= model.p0;
    model_R = cell(count,1);
    model_H = cell(count,1);  
    for i = 1:count
        model_R{i} = model(i,1).R;
        model_H{i} = model(i,1).H;
    end
    
%%%%滤波主程序  
    for k=1:L
        [x_predict,P_predict] = Predict_multiple(model,est_x_update{k},P_update{k});   
        if mod(k,10) == 0
            [est_temp,P_temp] = update_multiple(measure_value.LowFrequence.Z{k/10},model_R{2},model_H{2},x_predict,P_predict);
        else
            [est_temp,P_temp] = update_multiple(measure_value.HighFrequence.Z{k},model_R{1},model_H{1},x_predict,P_predict);
        end
        est_x_update{k+1}=est_temp;
        P_update{k+1}=P_temp;
    end 
    %估计出的数值为estimate_value的第二列到最后一列，第一列为初始数据
    EstimationValue = est_x_update(2:k+1);
end

%Kalman predict
function [x_predict,P_predict] = Predict_multiple(model,x,P)      
    F = model.F;
    Q = model.Q;

    x_predict = F*x;
    P_predict = Q+F*P*F'; 
end

%Kalman update
function [x_update,P_update] = update_multiple(measre_Z,model_R,model_H,x_predict,P_predict) 
    Z_Predict = model_H * x_predict;
    S  = model_R+model_H * P_predict * model_H'; %Vs= chol(S); det_S= prod(diag(Vs))^2; inv_sqrt_S= inv(Vs); iS= inv_sqrt_S*inv_sqrt_S'; 
    K  = P_predict*model_H'/S;
    IKH = (eye(size(P_predict))-K*model_H);
    
    x_update = x_predict + K*(measre_Z - Z_Predict);
    P_update = IKH*P_predict*IKH'+K*model_R*K';
end
