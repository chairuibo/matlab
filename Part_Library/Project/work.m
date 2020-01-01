%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%程序说明：  异步滤波
%           运动模型采用CA模型
%版本说明   1.0 （2019-12-29 CRB）    在序贯滤波工程的基础上修改得到
%           1.1（2019-12-29 CRB）    解决多维无法出错问题
%           1.2 (2019-12-30 CRB)     添加两种融合方法，highAndLow...的方法效果比较好
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
close all
clear
clc

Model_CA_1 = buildModel_CA(1,2,2000);        %精度差,频率高
Model_CA_2 = buildModel_CA(10,2,15);         %精度高,频率低
Model_CA = [Model_CA_1;Model_CA_2];                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           
%%%%%目标真实数据
TargetRealData = load('work/TargetRealData.mat');     %精度高
TargetRealData.L = length(TargetRealData.TPos);
TargetRealData.Z = cell(1,TargetRealData.L);
TargetRealData.Z2 = cell(1,TargetRealData.L/10);
for i=1:TargetRealData.L
    TargetRealData.Z{i} = TargetRealData.TPos(:,i);
end
for i=10:10:TargetRealData.L
    TargetRealData.Z2{i/10} = TargetRealData.TPos(:,i);
end
TargetRealData.L2 =length(TargetRealData.Z2);
TargetRealData = rmfield(TargetRealData,'TPos');

%%%%%雷达测量数据
RadarMasureData = load('work/RadarMasureData.mat'); %精度差
RadarMasureData.L = length(RadarMasureData.RMPos);
RadarMasureData.Z = cell(1,RadarMasureData.L);
for i=1:RadarMasureData.L
    RadarMasureData.Z{i} = RadarMasureData.RMPos(1,i);
    RadarMasureData.Z{i} = [RadarMasureData.Z{i};RadarMasureData.RMPos(3,i);];
end
RadarMasureData = rmfield(RadarMasureData,'RMPos');

%%%%%卫星测量数据
SatMasureData = load('work/SatMasureData.mat');     %精度高,频率低
SatMasureData.L = length(SatMasureData.SMPos);
SatMasureData.Z = cell(1,SatMasureData.L);
for i=1:SatMasureData.L
    SatMasureData.Z{i} = SatMasureData.SMPos(:,i);
end
SatMasureData = rmfield(SatMasureData,'SMPos');

%%%单个传感器各自滤波
% MeasureValue = [RadarMasureData.Z(10:10:end)',SatMasureData.Z'];
% MeasureValue = [RadarMasureData.RMPos;SatMasureData.SMPos];
EstimateRadarValue = Filter_Kalman(Model_CA_1,RadarMasureData);
EstimateSatValue = Filter_Kalman(Model_CA_2,SatMasureData);

%%%两个个传感器融合滤波  频率低的传感器进行校正
MeasureValue.LowFrequence = SatMasureData;
MeasureValue.HighFrequence= RadarMasureData;

EstimateFusionValue = Kalman_CentralizedFusionParallel_2(Model_CA,MeasureValue,2);
EstimateFusionValue_01 = Kalman_highAndlowFrequencies(Model_CA,MeasureValue,2);

EstimateFusionValue_temp = cell2mat(EstimateFusionValue);
EstimateFusionValue_temp(2,:) = EstimateFusionValue_temp(4,:);
MeasureValue.LowFrequence.Z = num2cell(EstimateFusionValue_temp(1:2,:),1);
EstimateFusionValue_02 = Kalman_highAndlowFrequencies(Model_CA,MeasureValue,2);
% EstimateFusionValue_02 = Kalman_ISMAsynchronousFusion(Model_CA,MeasureValue,2);
% EstimateFusionValue_03 = Kalman_PredictFusion(Model_CA,MeasureValue,2);


%%%%%仿真结果输出
q = output_work(1,Model_CA_1,TargetRealData.Z,RadarMasureData,EstimateRadarValue);
q = output_work(q+1,Model_CA_2,TargetRealData.Z2,SatMasureData,EstimateSatValue);
q = output_DemoCKParallel(q+1,Model_CA_1,TargetRealData.Z2,MeasureValue,EstimateFusionValue);
q = output_work(q+1,Model_CA_1,TargetRealData.Z,RadarMasureData,EstimateFusionValue_02);
MeasureValue.LowFrequence = SatMasureData;
MeasureValue.HighFrequence= RadarMasureData;
q = output_work(q+1,Model_CA_1,TargetRealData.Z,RadarMasureData,EstimateFusionValue_01);
% q = output_work(q+1,Model_CA_1,TargetRealData.Z,RadarMasureData,EstimateFusionValue_03);
% output_work(2,SatMasureData.K,TargetRealData.TPos(1,10:10:end),SatMasureData.Z,EstimateSatValue);
% output_temp(3,SatMasureData.K,TargetRealData.TPos(1,10:10:end),MeasureValue,EstimateFusionValue);
