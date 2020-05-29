 function EstimateValue = Kalman_predict(model,meas)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%����˵��:����������Ԥ�ⷽ��
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
%                   2)ÿ�������������ݰ�������
%                   3)��ͬ��������������
%�������˵����EstimateValue  ���Ԥ����
%     �����ʽ��1)ÿ��ʱ�̵Ľ������һ��Ԫ�ذ�����
%              2)ÿ��������״̬������������
%�汾˵��:1.0 ��2020-4-8 CRB 18235107312�������ļ�
%��Ȩ��Ϣ�������󾫵���ӵ�б���������Ȩ������ѧϰʹ��
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    L = meas.L;
    x_predict = cell(1,L+1);
    P_predict = cell(1,L+1);
    x_predict{1}= model.x0;
    P_predict{1}= model.p0.^2;

    for k=2:L
         %��x(k/k-1)��P(k/k-1)
         [x_predict{k},P_predict{k}] = predict_multiple(model,x_predict{k-1},P_predict{k-1},meas.Z{k});    
    end 
    %���Ƴ�����ֵΪestimate_value�ĵڶ��е����һ�У���һ��Ϊ��ʼ����
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
