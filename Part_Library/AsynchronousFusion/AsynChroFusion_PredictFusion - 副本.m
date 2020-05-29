function EstimationValue = AsynChroFusion_PredictFusion( model,estimate_value,,sensor_num )
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
    count = sensor_num;
    DataLength = length(estimate_value.HighFrequence.x); 
    est_x_update = cell(DataLength+1,1);
    P_update = cell(DataLength+1,1);
    measureValue.Z = cell(DataLength,1);
    est_x_update{1}= zeros(length(model(1).x0),1);
    x_perDimCount = model(1).x_dim/model(1).z_dim;  %x每个维度状态个数
    for i = 1:model.z_dim
        est_x_update{1}((i-1)*x_perDimCount+1) = estimate_value.HighFrequence.Z{1}(i);
    end
    P_update{1}= model.p0;
    model_R = model.R;
    model_H = model.H;

%%%%滤波主程序 
%前100秒各传感器独立滤波，不进行融合
    for k=1:100
        %如果高精度信息到来选用高精度信息
        if mod(k,10) == 1 
            est_x_update{k} = estimate_value.LowFrequence.x{(k+9)/10};
        %否则选择低精度信息
        else
            est_x_update{k} = estimate_value.HighFrequence.x{k};
        end
    end
%100秒后 每个周期内，低频信息进行预测
    x_predict_LP = cell(DataLength,1);     %低频传感器 x滤波器
    P_predict_LP = cell(DataLength,1);     %低频传感器 P滤波器
    est_x_update_temp = cell(DataLength,1);
    P_update_temp = cell(DataLength,1);
    
    for k=101:DataLength
        [x_predict,P_predict] = Predict_multiple(model(1),est_x_update_temp{k},P_update_temp{k});    
        [est_temp,P_temp] = update_multiple_02(measureValue.Z{k},model_R,model_H,x_predict,P_predict,count);
        est_x_update_temp{k+1}=est_temp;
        P_update_temp{k+1}=P_temp;
    end
%100秒后进行融合滤波，不带反馈的最优式分布融合滤波算法    
    for k=101:DataLength
         [x_predict,P_predict] = Predict_multiple(model(1),est_x_update{k},P_update{k});    
         [est_temp,P_temp] = update_multiple_02(measureValue.Z{k},model_R,model_H,x_predict,P_predict,count);
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
%Kalman update_2
function [x_update,P_update] = update_multiple_02(measure_Z,model_R,model_H,x_predict,P_predict,sensor_num)
    xUpdate_and_temp = 0;
    pUpdate_and_temp = 0;
    
    for j = 1:sensor_num
        xUpdate_and_temp = xUpdate_and_temp + model_H{j}'/model_R{j}*(measure_Z(1,j)-model_H{j}*x_predict);
        pUpdate_and_temp = pUpdate_and_temp + model_H{j}'/model_R{j}*model_H{j};
    end  
    P_update =inv(P_predict)+pUpdate_and_temp; 
    x_update = x_predict + P_update\xUpdate_and_temp;
    P_update = inv(P_update);
end
