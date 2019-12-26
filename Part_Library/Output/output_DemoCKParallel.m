function output_DemoCKParallel(model,truth,meas,est)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%程序说明：  仿真结果输出
%参数说明   model 运动模型
%           truth 真值
%           meas 量测值
%           est 估计值
%版本说明   1.0 （2019-12-26 CRB）    建立文件
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    tick=(1:truth.K).*model.T;
    length = truth.K;
    TruthValue=truth.X;
    sensor1 = meas(1,1);
    sensor2 = meas(2,1);
    pos_Truth_x=zeros(1,length);
    vel_Truth_x=zeros(1,length);
    acc_Truth_x=zeros(1,length);
    pos_obv1_x=zeros(1,length);
    pos_obv2_x=zeros(1,length);
    pos_est_x=zeros(1,length);
    vel_est_x=zeros(1,length);
    acc_est_x=zeros(1,length);
    for i=1:length
        pos_Truth_x(i)=TruthValue{i}(1);
        vel_Truth_x(i)=TruthValue{i}(2);
        acc_Truth_x(i)=TruthValue{i}(3);
        pos_obv1_x(i)=sensor1.Z{i}(1);
        pos_obv2_x(i)=sensor2.Z{i}(1);
        pos_est_x(i) =est(1,i);
        vel_est_x(i) =est(2,i);
        acc_est_x(i) =est(3,i);
    end 
    
    %%位置估计比较
    figure(1);
    title('Position picture');
    posErr_obv1_x = pos_obv1_x-pos_Truth_x;
    posErr_obv2_x = pos_obv2_x-pos_Truth_x;
    posErr_est_x = pos_est_x-pos_Truth_x;
    subplot(211);
    plot(tick,pos_Truth_x,'r',tick,pos_obv1_x,'g',tick,pos_obv2_x,'b',tick,pos_est_x,'k');
    legend('位置真值','传感器1','传感器2','估计值');
    subplot(212);
    plot(tick,posErr_est_x(1,:),'r',tick,posErr_obv1_x(1,:),'b',tick,posErr_obv2_x(1,:),'g');
    legend('估计误差','传感器1误差','传感器2误差');

    %%速度估计比较
    figure(2);
    title('Velocity picture');
    velErr_est_x = vel_est_x-vel_Truth_x;
    subplot(211);
    plot(tick,vel_Truth_x,'r',tick,vel_est_x,'g');
    legend('速度真值','估计值');
    subplot(212);
    plot(tick,velErr_est_x,'g');
    legend('估计误差');
    
    %%加速度估计比较
    figure(3);
    title('Acceleration picture');
    accErr_est_x = acc_est_x-acc_Truth_x;
    subplot(211);
    plot(tick,acc_Truth_x,'r',tick,acc_est_x,'b');
    legend('加速度真值','估计值');
    subplot(212);
    plot(tick,accErr_est_x,'g');
    legend('估计误差');
end

