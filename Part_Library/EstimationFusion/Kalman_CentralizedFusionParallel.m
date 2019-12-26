function [estimate_value ] = Kalman_CentralizedFusionParallel(model,measure_value,model_count)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%程序说明   集中式融合系统，并行滤波程序
%参数说明   model  运动模型
%           measure_value 所有传感器的量测数据
%           model_count  模型数量，即传感器数量
%版本说明   1.0 （2019-12-25 CRB）    建立文件
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    %初始化数据
    count = model_count;
    est_x_update(:,1)= model(1,1).x0;
    P_update(:,:,1)= model(1,1).p0.^2;
    model_R = cell(count,1);
    model_H = cell(count,1);
    zlength = measure_value(1,1).K;
    measure_Z = cell(zlength,1);
    for i = 1:count
        model_R{i} = model(i,1).R;
        model_H{i} = model(i,1).H;
        for j = 1:zlength
            measure_Z{j} = [measure_Z{j} measure_value(i,1).Z{j}];   %行为每一次的量测数据，列为每个传感器的数据
%             measure_Z{j,i} =[measure_Z{j,i} measure_value(j,1).Z{j}];
        end
    end
%%%%滤波主程序  
    for k=1:zlength
         [x_predict,P_predict] = Predict_multiple(model(1,:),est_x_update(:,k),P_update(:,:,k));    
         [est_temp,P_temp] = update_multiple(measure_Z{k},model_R,model_H,x_predict,P_predict,count);
         est_x_update(:,k+1)=est_temp;
         P_update(:,:,k+1)=P_temp;
    end 
    %估计出的数值为estimate_value的第二列到最后一列，第一列为初始数据
    estimate_value = est_x_update(:,2:k+1);
end

%Kalman predict
function [x_predict,P_predict] = Predict_multiple(model,x,P)      
    F = model.F;
    Q = model.Q;

    x_predict = F*x;
    P_predict = Q+F*P*F'; 
end

%Kalman update
function [x_update,P_update] = update_multiple(measure_Z,model_R,model_H,x_predict,P_predict,sensor_num)
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
