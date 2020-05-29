function estimate_value = EstiFusion_CentralizedParallel_2(model,measure_value,sensor_count)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%����˵��   ����ʽ�ں�ϵͳ�������˲�����
%����˵��   model  �˶�ģ��
%           measure_value ���д���������������
%           model_count  ģ��������������������
%�汾˵��   1.0 ��2019-12-25 CRB��    �����ļ�
%           1.1 (2019-12-31 CRB)    �޸ı������
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%��������
    count = sensor_count;   
    zlength = measure_value.LowFrequence.L;
    model_R = cell(1,count);
    model_H = cell(1,count);
    est_x_update = cell(1,zlength+1);
    P_update = cell(1,zlength+1);
    est_x_update{1}= model(1,1).x0;
    P_update{1}= model(1,1).p0;
    measure.Z = cell(1,zlength);
    for j=1:zlength
        measure.Z{j} =[ measure_value.HighFrequence.Z{j} measure_value.LowFrequence.Z{j}];
    end
    for i = 1:count
        model_R{i} = model(i,1).R;
        model_H{i} = model(i,1).H;
    end
%%%%�˲�������  
    for k=1:zlength
         [x_predict,P_predict] = Predict_multiple(model(1),est_x_update{k},P_update{k});    
         [est_temp,P_temp] = update_multiple(measure.Z{k},model_R,model_H,x_predict,P_predict,count);
         est_x_update{k+1}=est_temp;
         P_update{k+1}=P_temp;
    end 
    %���Ƴ�����ֵΪestimate_value�ĵڶ��е����һ�У���һ��Ϊ��ʼ����
    estimate_value = est_x_update(2:k+1);
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
        xUpdate_and_temp = xUpdate_and_temp + model_H{j}'/model_R{j}*(measure_Z(:,j)-model_H{j}*x_predict);
        pUpdate_and_temp = pUpdate_and_temp + model_H{j}'/model_R{j}*model_H{j};
    end  
    P_update =inv(P_predict)+pUpdate_and_temp; 
    x_update = x_predict + P_update\xUpdate_and_temp;
    P_update = inv(P_update);
end
