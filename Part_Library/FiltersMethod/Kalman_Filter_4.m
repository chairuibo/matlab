 function EstimateValue = Kalman_Filter_4(model,meas)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%����˵��:�����������˲�����
%�������˵��:1��model  �˶�ģ�� 
%               model.T ��������
%               model.F ״̬ת�ƾ���
%               model.Q ϵͳ������
%               model.R ����������
%               model.H �۲����
%               model.x0״̬������ʼֵ
%               model.p0.^2;Э��������ʼֵ
%            2��meas ����������������
%               meas.L ���ݳ���
%               meas.Z ���������ݣ���ʽΪ��
%                   1)ÿ��ʱ�̵����ݷ���һ��Ԫ�ذ�����
%                   2)�����������ݰ�������
%�������˵����EstimateValue  ����˲����
%               EstimateValue.x ״̬�������˲�ֵ
%               EstimateValue.x_p ״̬������Ԥ��ֵ
%               EstimateValue.P �������Э������P(k/k)
%               EstimateValue.P_p �������Э������P(k+1/k)
%     �����ʽ��1)ÿ��ʱ�̵Ľ������һ��Ԫ�ذ�����
%              3)��ͬʱ������Ϊһ��n��
%              2)ÿ��������״̬����Ϊn��һ��
%�汾˵��:1.0 ��2019-12-25 CRB 18235107312�������ļ�
%        1.1  (2019-12-26 CRB) ��Ӹ��º�Ԥ�⺯�������������ṹ
%        1.2  (2019-12-29 CRB) ������Ԫ������Ԫ�ظ�Ϊ����
%        1.3  (2020-01-14 CRB) �������P����   
%��Ȩ��Ϣ�������󾫵���ӵ�б���������Ȩ������ѧϰʹ��
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    L = meas.L;
    est_x_update = cell(1,L+1);
    P_update = cell(1,L+1);
    x_predict = cell(1,L);
    P_predict = cell(1,L);
    est_x_update{1}= model.x0;
    P_update{1}= model.p0.^2;

    for k=1:L
         %��x(k/k-1)��P(k/k-1)
         [x_predict{k},P_predict{k}] = predict_multiple(model,est_x_update{k},P_update{k});    
         %��x(k/k)��P(k/k)
         [est_x_update{k+1},P_update{k+1}] = update_multiple(meas.Z{k},model,x_predict{k},P_predict{k});
    end 
    %���Ƴ�����ֵΪestimate_value�ĵڶ��е����һ�У���һ��Ϊ��ʼ����
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
