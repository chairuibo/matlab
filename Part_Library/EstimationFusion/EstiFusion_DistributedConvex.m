function [ estimate_data ] = EstiFusion_DistributedConvex( localEstimate_data )
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%程序说明:分布式融合系统，简单凸组合融合算法
%参数说明:measure_value 所有传感器的量测数据
%           -measure_value数据格式要求：包含三个部分 
%           -L为数据长度，
%           -count为传感器数量
%           -data为每个传感器局部估计数据（包括x和P）
%版本说明：1.0 （2020-01-10 CRB）    建立文件
%         1.1 （2020-01-13 CRB）    修改SingleStepMultiple函数
%                   中的错误，P的逆错误的使用了P，导致结果刚好相反
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    count = localEstimate_data.count;
    Data_L = localEstimate_data.L;
    estimate_data = cell(Data_L,1);
    measure_value.x = cell(Data_L,count);
    measure_value.P = cell(Data_L,count);
  
 %%%整理数据   
    for i=1:Data_L
        for j = 1:count 
            measure_value.z{i,j} = localEstimate_data.data(j).x{i};
            measure_value.P{i,j} = localEstimate_data.data(j).P{i};
        end
    end
    
 %%%滤波主程序  
    for i=1:Data_L
        estimate_data{i} = SingleStepMultiple(measure_value.z(i,:),measure_value.P(i,:),count);
    end
end

%单步计算,算法基本单元
function  x_temp = SingleStepMultiple( x_PartEstimateValue,P_PartEstimateValue,SensorCount)
    x_temp = 0;
    P_temp = 0;
    Px_temp = 0;
    for j=1:SensorCount
        P_temp = P_temp + inv(P_PartEstimateValue{j});
        Px_temp = Px_temp + P_PartEstimateValue{j}\x_PartEstimateValue{j};
    end
    x_temp = P_temp\Px_temp;
end

