function output_work(i,K,truth,meas,est)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%����˵����  ���������
%����˵��   K ���ݳ���
%           truth ��ֵ
%           meas ����ֵ
%           est ����ֵ
%�汾˵��   1.0 ��2019-12-29 CRB��    �����ļ�
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    tick=1:K;
    
    pos_Truth_x=truth(1,:);
    pos_obv1_x=meas(1,:);
    pos_est_x=est(1,:);

    %%λ�ù��ƱȽ�
    figure(i);
    title('Position picture');
    posErr_obv1_x = pos_obv1_x-pos_Truth_x;
    posErr_est_x = pos_est_x-pos_Truth_x;
    subplot(211);
    plot(tick,pos_Truth_x,'r',tick,pos_obv1_x,'g',tick,pos_est_x,'k');
    legend('λ����ֵ','����ֵ','����ֵ');
    subplot(212);
    plot(tick,posErr_est_x(1,:),'r',tick,posErr_obv1_x(1,:),'b');
    legend('�������','������1���');

%     %%�ٶȹ��ƱȽ�
%     figure(2);
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
%     figure(3);
%     title('Acceleration picture');
%     accErr_est_x = acc_est_x-acc_Truth_x;
%     subplot(211);
%     plot(tick,acc_Truth_x,'r',tick,acc_est_x,'b');
%     legend('���ٶ���ֵ','����ֵ');
%     subplot(212);
%     plot(tick,accErr_est_x,'g');
%     legend('�������');
end

