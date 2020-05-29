function EstimationValue = AsynChroFusion_PredictFusion( model,estimate_value,fusion_value,sensor_num )
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%程序说明   异步融合程序，低频率传感器做预测,采用不带反馈的最优分布式估计融合
%参数说明   model  运动模型
%           measure_value 所有传感器的量测数据
%           sensor_num  传感器数量
%版本说明   1.0 （2019-12-30 CRB）    建立文件
%           1.1 （2019-01-23 CRB）    将集中式融合改为分布式
%           1.2 （2019-02-10 CRB）    选择不带反馈的最优分布式融合
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%初始化数据
    SensorCount = sensor_num;
    DataLength = length(estimate_value.HighFrequence.x); 
    Sensor_1Data = estimate_value.LowFrequence;
    Sensor_2Data = estimate_value.HighFrequence;
    SensorData = cell(DataLength,1);
    est_x_update_G = cell(DataLength,1);
    P_update_G = cell(DataLength,1);
%     x_predict_G = cell(DataLength,1);
%     P_predict_G = cell(DataLength,1);
%     measureValue.Z = cell(DataLength,1);
%     est_x_update{1}= zeros(length(model(1).x0),1);
%     x_perDimCount = model(1).x_dim/model(1).z_dim;  %x每个维度状态个数
%     for i = 1:model.z_dim
%         est_x_update{1}((i-1)*x_perDimCount+1) = estimate_value.HighFrequence.Z{1}(i);
%     end
%     P_update{1}= model.p0;

%%%%滤波主程序 
%前100秒传感器进行简单并行融合滤波
    est_x_update_G(1:100) = fusion_value.x(1:100);
    P_update_G(1:100) = fusion_value.P(1:100);
    x_predict_G(1:100) = fusion_value.x_p(1:100);
    P_predict_G(1:100) = fusion_value.P_p(1:100);
%100秒后 每个周期内，低频信息进行预测
    x_predict_LP = cell(DataLength,1);     %低频传感器 x
    P_predict_LP = cell(DataLength,1);     %低频传感器 P
    est_x_update_temp = cell(DataLength,1);
    P_update_temp = cell(DataLength,1);
    est_x_update_temp{100} = Sensor_1Data.x_p{10};
    P_update_temp{100} = Sensor_1Data.P_p{10};
    for k=101:DataLength
        if mod(k,10)==0 
           %%%%124用
%             x_predict_LP{k} = Sensor_1Data.x_p{(k+9)/10};     %低频传感器 x
%             P_predict_LP{k} = Sensor_1Data.P_p{(k+9)/10};     %低频传感器 P
%             est_x_update_temp{k} = Sensor_1Data.x{(k+9)/10};
%             P_update_temp{k} = Sensor_1Data.P{(k+9)/10};
            %%%3用
            x_predict_LP{k} = Sensor_1Data.x_p{k/10};     %低频传感器 x
            P_predict_LP{k} = Sensor_1Data.P_p{k/10};     %低频传感器 P
            est_x_update_temp{k} = Sensor_1Data.x{k/10};
            P_update_temp{k} = Sensor_1Data.P{k/10};
        else
            [x_predict_LP{k},P_predict_LP{k}] = Predict_multiple(model,est_x_update_temp{k-1},P_update_temp{k-1});    
            [est_x_update_temp{k},P_update_temp{k}] = update_multiple(model,x_predict_LP{k},P_predict_LP{k});
        end
    end
    Sensor_1Data.x_p = x_predict_LP;
    Sensor_1Data.P_p = P_predict_LP;
    Sensor_1Data.x = est_x_update_temp;
    Sensor_1Data.P = P_update_temp;
%     save('Work/SatEstimate.mat','x_predict_LP');
%100秒后进行融合滤波，不带反馈的最优式分布融合滤波算法    
    for k=101:DataLength
        SensorData{k}.kalman_x_predict{1} = Sensor_1Data.x_p{k};
        SensorData{k}.kalman_P_predict{1} = Sensor_1Data.P_p{k};
        SensorData{k}.kalman_x_update{1} =  Sensor_1Data.x{k};
        SensorData{k}.kalman_P_update{1} =  Sensor_1Data.P{k};
        SensorData{k}.kalman_x_predict{2} = Sensor_2Data.x_p{k};
        SensorData{k}.kalman_P_predict{2} = Sensor_2Data.P_p{k};
        SensorData{k}.kalman_x_update{2} =  Sensor_2Data.x{k};
        SensorData{k}.kalman_P_update{2} =  Sensor_2Data.P{k};
    end
    InvP = inv(P_update_G{100});
    for k=101:DataLength    
       [est_x_update_G{k},InvP ]=SingleStepMultiple(model,est_x_update_G{k-1},InvP,SensorData{k},SensorCount);
    end 
    %估计出的数值为estimate_value的第二列到最后一列，第一列为初始数据
    EstimationValue = est_x_update_G;
end
%%%单步计算,算法基本单元
function  [x_out,P_out]= SingleStepMultiple(model,x_Global_update,P_Global_update,SensorData,SensorCount)
 %%%整理数据   
    x_Gu = x_Global_update;
    P_Gu = P_Global_update;
    x_1 = SensorData.kalman_x_predict;
    P_1 = SensorData.kalman_P_predict;
    x_2 = SensorData.kalman_x_update;
    P_2 = SensorData.kalman_P_update;
    invP_temp = 0;
    invPx_temp = 0;
    
 %%%kernel     
    invP_Gp = inv(model.F/P_Gu*model.F'+model.Q);
    x_Gp =  model.F * x_Gu;
    for j=1:SensorCount
        invP_temp = invP_temp + inv(P_2{j})-inv(P_1{j});
        invPx_temp = invPx_temp + P_2{j}\x_2{j}-P_1{j}\x_1{j};
    end
    P_out = invP_Gp + invP_temp;
    x_out = P_out\invP_Gp*x_Gp + P_out\invPx_temp;
end
%Kalman predict
function [x_predict,P_predict] = Predict_multiple(model,x,P)      
    F = model.F;
    Q = model.Q;

    x_predict = F*x;
    P_predict = Q+F*P*F'; 
end
%Kalman update
function [x_update,P_update] = update_multiple(model,xPredict,PPredict)
    H = model.H;
    P = PPredict;
    R = model.R;
    Z_Estimate =H*xPredict+model.D*randn(size(model.D,2),size(xPredict,2));
    
    Z_Predict = H*xPredict;
    S  = R+H*P*H'; %Vs= chol(S); det_S= prod(diag(Vs))^2; inv_sqrt_S= inv(Vs); iS= inv_sqrt_S*inv_sqrt_S'; 
    K  = P*H'/S;
    IKH = (eye(size(P))-K*H);
    x_update = xPredict + K*(Z_Estimate-Z_Predict);
    P_update = IKH*P*IKH'+K*R*K';
end
