function output_work(i,K,truth,meas,est)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%程序说明：  仿真结果输出
%参数说明   K 数据长度
%           truth 真值
%           meas 量测值
%           est 估计值
%版本说明   1.0 （2019-12-29 CRB）    建立文件
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    tick=1:K;
    
    pos_Truth_x=truth(1,:);
    pos_obv1_x=meas(1,:);
    pos_est_x=est(1,:);

    %%位置估计比较
    figure(i);
    title('Position picture');
    posErr_obv1_x = pos_obv1_x-pos_Truth_x;
    posErr_est_x = pos_est_x-pos_Truth_x;
    subplot(211);
    plot(tick,pos_Truth_x,'r',tick,pos_obv1_x,'g',tick,pos_est_x,'k');
    legend('位置真值','量测值','估计值');
    subplot(212);
    plot(tick,posErr_est_x(1,:),'r',tick,posErr_obv1_x(1,:),'b');
    legend('估计误差','传感器1误差');

%     %%速度估计比较
%     figure(2);
%     title('Velocity picture');
%     velErr_est_x = vel_est_x-vel_Truth_x;
%     subplot(211);
%     plot(tick,vel_Truth_x,'r',tick,vel_est_x,'g');
%     legend('速度真值','估计值');
%     subplot(212);
%     plot(tick,velErr_est_x,'g');
%     legend('估计误差');
%     
%     %%加速度估计比较
%     figure(3);
%     title('Acceleration picture');
%     accErr_est_x = acc_est_x-acc_Truth_x;
%     subplot(211);
%     plot(tick,acc_Truth_x,'r',tick,acc_est_x,'b');
%     legend('加速度真值','估计值');
%     subplot(212);
%     plot(tick,accErr_est_x,'g');
%     legend('估计误差');
end

