function out = output_temp(index,model,truth,meas,est)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%����˵����  ���������
%����˵��   index ͼ�����
%           model �˶�ģ��
%           truth ��ֵ
%           meas ����ֵ
%           est ����ֵ
%�汾˵��   1.0 ��2019-12-29 CRB��    �����ļ�
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
   
    L = length(est);
    tick=(1:L)*model.T;
    pos_truth_x=zeros(1,L);
    pos_truth_y=zeros(1,L);
    pos_obv1_x=zeros(1,L);
    pos_obv1_y=zeros(1,L);
    pos_est_x =zeros(1,L);
    vel_est_x =zeros(1,L);
    acc_est_x =zeros(1,L);
    pos_est_y =zeros(1,L);
    vel_est_y =zeros(1,L);
    acc_est_y =zeros(1,L);
    for i=1:L
        pos_truth_x(i)=truth{i}(1);
        pos_truth_y(i)=truth{i}(3);
        pos_obv1_x(i)=meas.Z{i}(1);
        pos_obv1_y(i)=meas.Z{i}(2);
        pos_est_x(i) =est{i}(1);
        vel_est_x(i) =est{i}(2);
        acc_est_x(i) =est{i}(3);
        pos_est_y(i) =est{i}(4);
        vel_est_y(i) =est{i}(5);
        acc_est_y(i) =est{i}(6);
    end

    %%λ�ù��ƱȽ�
    figure(index);
    out=index;
    title('Position picture');
    posErr_obv1_x = pos_obv1_x-pos_truth_x;
    posErr_est_x = pos_est_x-pos_truth_x;
    subplot(221);
    plot(tick,pos_truth_x,'r',tick,pos_obv1_x,'g',tick,pos_est_x,'k');
    legend('λ����ֵ','����ֵ','����ֵ');
    subplot(223);
    plot(tick,posErr_est_x(1,:),'r',tick,posErr_obv1_x(1,:),'b');
    legend('�������','������1���');
    
    posErr_obv1_y = pos_obv1_y-pos_truth_y;
    posErr_est_y = pos_est_y-pos_truth_y;
    subplot(222);
    plot(tick,pos_truth_y,'r',tick,pos_obv1_y,'g',tick,pos_est_y,'k');
    legend('λ����ֵ','����ֵ','����ֵ');
    subplot(224);
    plot(tick,posErr_est_y(1,:),'r',tick,posErr_obv1_y(1,:),'b');
    legend('�������','������1���');

    %%�ٶȹ��ƱȽ�
%     figure(i+1);
%     title('Velocity picture');
%     velErr_est_x = vel_est_x-vel_Truth_x;
%     subplot(211);
%     plot(tick,vel_Truth_x,'r',tick,vel_est_x,'g');
%     legend('�ٶ���ֵ','����ֵ');
%     subplot(212);
%     plot(tick,velErr_est_x,'g');
%     legend('�������');
%     
%     %%���ٶȹ��ƱȽ�
%     figure(i+2);
%     title('Acceleration picture');
%     accErr_est_x = acc_est_x-acc_Truth_x;
%     subplot(211);
%     plot(tick,acc_Truth_x,'r',tick,acc_est_x,'b');
%     legend('���ٶ���ֵ','����ֵ');
%     subplot(212);
%     plot(tick,accErr_est_x,'g');
%     legend('�������');
end

