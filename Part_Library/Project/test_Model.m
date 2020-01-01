%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%程序说明：  异步滤波
%           运动模型采用CA模型
%版本说明   1.0 （2019-12-29 CRB）    在序贯滤波工程的基础上修改得到
%           1.1（2019-12-29 CRB）    解决多维无法出错问题
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
close all
clear
clc

% Model_CA_1 = buildModel_CA(1,3,200);        %精度差
Model_CA_2 = buildModel_CA(10,2,2);        %精度高
TargetRealData = load('work/TargetRealData.mat');     %精度高
TargetRealData.L = length(TargetRealData.TPos);
TargetRealData.Z = cell(TargetRealData.L,1);
TargetRealData.Z2 = cell(TargetRealData.L/10,1);
for i=1:TargetRealData.L
    TargetRealData.Z{i} = TargetRealData.TPos(:,i);
end
for i=10:10:TargetRealData.L
    TargetRealData.Z2{i/10} = TargetRealData.TPos(:,i);
end
TargetRealData.L2 =length(TargetRealData.Z2);
TargetRealData = rmfield(TargetRealData,'TPos');

% RadarMasureData = load('work/RadarMasureData.mat'); %精度差
% RadarMasureData.L = length(RadarMasureData.RMPos);
% RadarMasureData.Z = cell(RadarMasureData.L,1);
% for i=1:RadarMasureData.L
%     SatMasureData.Z{i} = SatMasureData.SMPos(:,i);
% end
% RadarMasureData = rmfield(RadarMasureData,'RMPos');

SatMasureData = load('work/SatMasureData.mat');     %精度高
SatMasureData.L = length(SatMasureData.SMPos);
SatMasureData.Z = cell(SatMasureData.L,1);
for i=1:SatMasureData.L
    SatMasureData.Z{i} = SatMasureData.SMPos(:,i);
end
SatMasureData = rmfield(SatMasureData,'SMPos');

%%%单个传感器各自滤波
% MeasureValue = [RadarMasureData.Z(10:10:end)',SatMasureData.Z'];
% MeasureValue = [RadarMasureData.RMPos;SatMasureData.SMPos];
% EstimateRadarValue = Filter_Kalman(Model_CA_1,RadarMasureData);
EstimateSatValue = Filter_Kalman(Model_CA_2,SatMasureData);

%%%两个个传感器融合滤波  频率低的传感器时刻上进行融合
% EstimateFusionValue = Kalman_CentralizedFusionParallel_2(Model,MeasureValue,2);

% output_work(1,RadarMasureData.K,TargetRealData.TPos,RadarMasureData.Z,EstimateRadarValue);
output_work(2,Model_CA_2,TargetRealData,SatMasureData,EstimateSatValue);
% output_temp(3,SatMasureData.K,TargetRealData.TPos(1,10:10:end),MeasureValue,EstimateFusionValue);
% output_DemoCKParallel(Model_CA_1,TargetRealData,MeasureValue,EstimateValue);
