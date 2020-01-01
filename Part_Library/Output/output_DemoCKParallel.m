function outindex = output_DemoCKParallel(index,model,truth,meas,est)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%����˵����  ���������
%����˵��   model �˶�ģ��
%           truth ��ֵ
%           meas ����ֵ
%           est ����ֵ
%�汾˵��   1.0 ��2019-12-26 CRB��    �����ļ�
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    zlength = length(est);
    tick=(1:zlength).*model.T;
    TruthValue=truth;
    sensor1 = meas.LowFrequence;
    sensor2 = meas.HighFrequence;
    pos_Truth_x=zeros(1,zlength);
    vel_Truth_x=zeros(1,zlength);
    acc_Truth_x=zeros(1,zlength);
    pos_obv1_x=zeros(1,zlength);
    pos_obv2_x=zeros(1,zlength);
    pos_est_x=zeros(1,zlength);
    vel_est_x=zeros(1,zlength);
    acc_est_x=zeros(1,zlength);
    for i=1:zlength
        pos_Truth_x(i)=TruthValue{i}(1);
        vel_Truth_x(i)=TruthValue{i}(2);
        acc_Truth_x(i)=TruthValue{i}(3);
        pos_obv1_x(i)=sensor1.Z{i}(1);
        pos_obv2_x(i)=sensor2.Z{i*10}(1);
        pos_est_x(i) =est{i}(1);
        vel_est_x(i) =est{i}(2);
        acc_est_x(i) =est{i}(3);
    end 
    
    %%λ�ù��ƱȽ�
    figure(index);
    outindex = index;
    title('Position picture');
    posErr_obv1_x = pos_obv1_x-pos_Truth_x;
    posErr_obv2_x = pos_obv2_x-pos_Truth_x;
    posErr_est_x = pos_est_x-pos_Truth_x;
    subplot(211);
    plot(tick,pos_Truth_x,'r',tick,pos_obv1_x,'g',tick,pos_obv2_x,'b',tick,pos_est_x,'k');
    legend('λ����ֵ','������1','������2','����ֵ');
    subplot(212);
    plot(tick,posErr_est_x(1,:),'r',tick,posErr_obv1_x(1,:),'b',tick,posErr_obv2_x(1,:),'g');
    legend('�������','������1���','������2���');

    %%�ٶȹ��ƱȽ�
    index = index+1;
    figure(index);
    outindex = index;
    title('Velocity picture');
    velErr_est_x = vel_est_x-vel_Truth_x;
    subplot(211);
    plot(tick,vel_Truth_x,'r',tick,vel_est_x,'g');
    legend('�ٶ���ֵ','����ֵ');
    subplot(212);
    plot(tick,velErr_est_x,'g');
    legend('�������');
    
    %%���ٶȹ��ƱȽ�
%     index = index+1;
%     figure(index);
%     outindex = index;
%     title('Acceleration picture');
%     accErr_est_x = acc_est_x-acc_Truth_x;
%     subplot(211);
%     plot(tick,acc_Truth_x,'r',tick,acc_est_x,'b');
%     legend('���ٶ���ֵ','����ֵ');
%     subplot(212);
%     plot(tick,accErr_est_x,'g');
%     legend('�������');
end

