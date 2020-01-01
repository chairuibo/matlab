function out = output_temp(index,model,truth,meas,est)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%程序说明：  仿真结果输出
%参数说明   index 图表序号
%           model 运动模型
%           truth 真值
%           meas 量测值
%           est 估计值
%版本说明   1.0 （2019-12-29 CRB）    建立文件
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

    %%位置估计比较
    figure(index);
    out=index;
    title('Position picture');
    posErr_obv1_x = pos_obv1_x-pos_truth_x;
    posErr_est_x = pos_est_x-pos_truth_x;
    subplot(221);
    plot(tick,pos_truth_x,'r',tick,pos_obv1_x,'g',tick,pos_est_x,'k');
    legend('位置真值','量测值','估计值');
    subplot(223);
    plot(tick,posErr_est_x(1,:),'r',tick,posErr_obv1_x(1,:),'b');
    legend('估计误差','传感器1误差');
    
    posErr_obv1_y = pos_obv1_y-pos_truth_y;
    posErr_est_y = pos_est_y-pos_truth_y;
    subplot(222);
    plot(tick,pos_truth_y,'r',tick,pos_obv1_y,'g',tick,pos_est_y,'k');
    legend('位置真值','量测值','估计值');
    subplot(224);
    plot(tick,posErr_est_y(1,:),'r',tick,posErr_obv1_y(1,:),'b');
    legend('估计误差','传感器1误差');

    %%速度估计比较
%     figure(i+1);
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
%     figure(i+2);
%     title('Acceleration picture');
%     accErr_est_x = acc_est_x-acc_Truth_x;
%     subplot(211);
%     plot(tick,acc_Truth_x,'r',tick,acc_est_x,'b');
%     legend('加速度真值','估计值');
%     subplot(212);
%     plot(tick,accErr_est_x,'g');
%     legend('估计误差');
end

