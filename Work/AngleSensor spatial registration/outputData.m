function out = outputData(index,model,meas,truth,est)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%程序说明：  仿真结果输出
%参数说明   index 图表序号
%           model 运动模型
%           truth 真值
%           meas 量测值
%           est 估计值
%版本说明   1.0 （2020-01-12 CRB）    建立文件
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
   
    L = length(est);
    Tick=(1:L)*model.T;
    
%% 转换数据
    EstValue = cell2mat(est);
    Pos_truth_x = truth(1,:);   %目标X轴真实数据
    Pos_meas_x = meas(1,:);     %目标X轴量测数据
    Pos_est_x = EstValue(1,:);  %目标X轴估计数据
    Pos_truth_y=truth(2,:);     %目标X轴真实数据
    Pos_meas_y=meas(2,:);       %目标X轴量测数据
    Pos_est_y =EstValue(4,:);   %目标X轴估计数据
    
%% 画图
    %%位置估计比较
    %y轴方向
    out = index+1;
    F=figure(out);
    set(F,'name','目标X轴运动曲线','position',[250 200 500 400]);
    PosErr_est_x = Pos_est_x-Pos_truth_x;
    s(1) = subplot(211);
    plot(Tick,Pos_truth_x,'r',Tick,Pos_est_x,'b');
%     title(s(1),Title.y);
    legend('位置真值','滤波值');
    xlabel('时间 t/s');
    ylabel('X位置 /m');
    subplot(212);
    plot(Tick,PosErr_est_x,'g');
    legend('滤波误差');
    xlabel('时间 t/s');
    ylabel('误差 /m');
    
    out = out+1;
    F=figure(out);
    set(F,'name','目标Y轴运动曲线','position',[350 100 500 400]);
    PosErr_est_y = Pos_est_y-Pos_truth_y;
    s(1) = subplot(211);
    plot(Tick,Pos_truth_y,'r',Tick,Pos_est_y,'b');
%     title(s(1),Title.y);
    legend('位置真值','滤波值');
    xlabel('时间 t/s');
    ylabel('X位置 /m');
    subplot(212);
    plot(Tick,PosErr_est_y,'g');
    legend('滤波误差');
    xlabel('时间 t/s');
    ylabel('误差 /m');
end
