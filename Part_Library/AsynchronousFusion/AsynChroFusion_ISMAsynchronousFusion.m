function EstimationValue = AsynChroFusion_ISMAsynchronousFusion(model,measure_value,sensor_count)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%程序说明   异步融合程序
%参数说明   model  运动模型
%           measure_value 所有传感器的量测数据
%           sensor_count  传感器数量
%版本说明   1.0 （2019-12-29 CRB）    建立文件
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%数据整理
    count = sensor_count;
    L = measure_value.LowFrequence.L;
    est_x_update = cell(L+1,1);
    P_update = cell(L+1,1);
    measureValue.Z = cell(L,1);
    model_H = cell(1,1);  
    est_x_update{1}= model.x0;
    P_update{1}= model.p0;
    
    for i = 1:10
        model_H{1} = [model_H{1};model(1).H/model(1).F'];
    end
    model_H{1} = [model_H{1};model(2).H/model(2).F'];
    
    for j = 1:L 
        for i=1:10
            measureValue.Z{j} = [measureValue.Z{j},measure_value.HighFrequence.Z{i}'];
        end
        measureValue.Z{j} = [measureValue.Z{j},measure_value.LowFrequence.Z{j}'];
        measureValue.Z{j} = measureValue.Z{j}';
    end
    
    model_R1 = model(1).H/model(1).F'*model(1).Q*(model(1).H/model(1).F')';
    model_R2 = model(2).H/model(2).F'*model(2).Q*(model(1).H/model(1).F')';
    model_R = model_R1;
    for i=1:9
        model_R = [model_R model_R1];
    end
    for i=1:9
        model_R(2*i+1:2*i+2,:) = model_R(1:2,:);
    end
    for i=1:10
        model_R(21:22,2*i-1:2*i) = model_R2;
        model_B(1:6,2*i-1:2*i) = [model(1).B];
%         Cov_wyita(1:2,2*i-1:2*i) = -model(1).Q*(model(1).H/model(1).F')';
    end
    model_R(1:20,21:22) = model_R(21:22,1:20)';
    model_R(21:22,21:22) = model(2).H/model(2).F'*model(2).Q*(model(2).H/model(2).F')'; 
%     Cov_wyita(1:2,11:12) = -model(2).Q*(model(2).H/model(2).F')';
    model_B(1:6,21:22) = [model(2).B];
%%%%%滤波主程序  
    for k=1:L
        [x_predict,P_predict] = Predict_multiple(model(2),est_x_update{k},P_update{k});   
        [est_temp,P_temp] = update_multiple(measureValue.Z{k},model_R,model_H{1},model_B,x_predict,P_predict);
        est_x_update{k+1}=est_temp;
        P_update{k+1}=P_temp;
    end 
    %估计出的数值为estimate_value的第二列到最后一列，第一列为初始数据
    EstimationValue = est_x_update(2:k+1);
end

%%%%%%Kalman predict
function [x_predict,P_predict] = Predict_multiple(model,x,P)      
    F = model.F;
    Q = model.Q;

    x_predict = F*x;
    P_predict = Q+F*P*F'; 
end

%%%%%%Kalman update
function [x_update,P_update] = update_multiple(measre_Z,model_R,model_H,model_B,x_predict,P_predict) 
    Z_Predict = model_H * x_predict;
%     aaaa=P_predict*model_H';
    L = P_predict*model_H' + model_B;
    S  = model_R+model_H * P_predict * model_H'+model_H*model_B+(model_H*model_B)'; %Vs= chol(S); det_S= prod(diag(Vs))^2; inv_sqrt_S= inv(Vs); iS= inv_sqrt_S*inv_sqrt_S'; 
    K  = L/S;
 
    x_update = x_predict + K*(measre_Z - Z_Predict);
    P_update = P_predict-L/S*L';
end
