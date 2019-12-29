%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%程序说明：  异步滤波
%           运动模型采用CA模型
%版本说明   1.0 （2019-12-29 CRB）    在序贯滤波工程的基础上修改得到
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
close all
clear
clc

Model_CA_1 = buildModel_CA(1,200);        %精度差
Model_CA_2 = buildModel_CA(10,2);        %精度高
Model = [Model_CA_1;Model_CA_2];
TargetRealData = load('work/TargetRealData.mat');

RadarMasureData = load('work/RadarMasureData.mat'); %精度差
RadarMasureData.K = length(RadarMasureData.RMPos);
RadarMasureData.Z = RadarMasureData.RMPos(1,:);
RadarMasureData = rmfield(RadarMasureData,'RMPos');

SatMasureData = load('work/SatMasureData.mat');     %精度高
SatMasureData.K = length(SatMasureData.SMPos);
SatMasureData.Z = SatMasureData.SMPos(1,:);
SatMasureData = rmfield(SatMasureData,'SMPos');

%%%单个传感器各自滤波
MeasureValue = [RadarMasureData.Z(10:10:end)',SatMasureData.Z'];
% MeasureValue = [RadarMasureData.RMPos;SatMasureData.SMPos];
EstimateRadarValue = Filter_Kalman(Model_CA_1,RadarMasureData);
EstimateSatValue = Filter_Kalman(Model_CA_2,SatMasureData);

%%%两个个传感器融合滤波  频率低的传感器时刻上进行融合
EstimateFusionValue = Kalman_CentralizedFusionParallel_2(Model,MeasureValue,2);

output_work(1,RadarMasureData.K,TargetRealData.TPos,RadarMasureData.Z,EstimateRadarValue);
output_work(2,SatMasureData.K,TargetRealData.TPos(1,10:10:end),SatMasureData.Z,EstimateSatValue);
output_temp(3,SatMasureData.K,TargetRealData.TPos(1,10:10:end),MeasureValue,EstimateFusionValue);
% output_DemoCKParallel(Model_CA_1,TargetRealData,MeasureValue,EstimateValue);
