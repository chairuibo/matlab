%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%程序说明：异步滤波
%           运动模型采用CA模型
%版本说明 1.0 （2020-01-12 CRB）    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
close all
clear
clc

%%%建立模型
Model_CA_1 = buildModel_CA(1,3,2000);        %精度差,频率高
Model_CA_11 = buildModel_CA(1,2,2000);       %精度差,频率高
Model_CA_2 = buildModel_CA(10,2,5);         %精度高,频率低
Model_CA_21 = buildModel_CA(1,2,5);         %精度高,频率低  70比并行滤波精度提高一倍
Model_CA_22 = buildModel_CA(10,3,5);         %精度高,频率低
Model_CA = [Model_CA_11;Model_CA_2];                           

%%%整理数据
%目标真实数据
TargetRealData = load('work_RSDF/TargetRealData3.mat');     %精度高
TargetRealData.L = length(TargetRealData.TPos);
TargetRealData.Z = cell(1,TargetRealData.L/10);
TargetRealData.Z2 = cell(1,TargetRealData.L/100);
for i=10:10:TargetRealData.L
    TargetRealData.Z{i/10} = TargetRealData.TPos(:,i);
end
for i=1:100:TargetRealData.L
    TargetRealData.Z2{(i+99)/100} = TargetRealData.TPos(:,i);
end
TargetRealData.L2 =length(TargetRealData.Z2);
TargetRealData.L = TargetRealData.L/10;
% TargetRealData = rmfield(TargetRealData,'TPos');

%雷达测量数据
RadarMasureData = load('work_RSDF/RadarMasureData3.mat'); %精度差
RadarRealData = load('work_RSDF/RadarRealData3.mat');
RadarMasureData.L = length(RadarMasureData.RMPos);
RadarMasureData.Z = cell(1,RadarMasureData.L/10);
for i=10:10:RadarMasureData.L
    RadarMasureData.Z{i/10} = RadarMasureData.RMPos(1,i);
    RadarMasureData.Z{i/10} = [RadarMasureData.Z{i/10};RadarMasureData.RMPos(3,i)];
end
RadarMasureData.L = RadarMasureData.L/10;
% RadarMasureData = rmfield(RadarMasureData,'RMPos');

%卫星测量数据
SatMasureData = load('work_RSDF/SatMasureData3.mat');     %精度高,频率低
SatRealData = load('work_RSDF/SatRealData3.mat');
% SatMasureData3_add = load('work_RSDF/SatMasureData3_add.mat');卫星的数据加雷达的Y轴数据
SatMasureData.L = length(SatMasureData.SMPos);
% SatMasureData3_add.L = length(SatMasureData3_add.SMPos);      卫星的数据加雷达的Y轴数据
SatMasureData.Z = cell(1,SatMasureData.L);
for i=1:SatMasureData.L
    SatMasureData.Z{i} = SatMasureData.SMPos(:,i);
    SatRealData.Z{i} = SatRealData.RealSMPos(1,i);
    SatRealData.Z{i} = [SatRealData.Z{i};0;SatRealData.RealSMPos(2,i)];
%     SatMasureData3_add.Z{i} = SatMasureData3_add.SMPos(:,i);  卫星的数据加雷达的Y轴数据
end
% SatMasureData = rmfield(SatMasureData,'SMPos');

%%%滤波算法
%单个传感器各自滤波
EstimateRadarValue = Kalman_Filter_4(Model_CA_11,RadarMasureData);
EstimateSatValue = Kalman_Filter_4(Model_CA_2,SatMasureData);
EstimateValue.LowFrequence = EstimateSatValue;
EstimateValue.HighFrequence = EstimateRadarValue;

%两个个传感器融合滤波  频率低的传感器进行校正
MeasureValue.LowFrequence = SatMasureData;
MeasureValue.HighFrequence= RadarMasureData;
%直接使用并行滤波器
% EstimateFusionValue_01 = EstiFusion_CentralizedParallel_2(Model_CA,MeasureValue,2);
% 并行滤波算法（其实不是真正的并行滤波算法，上边才是，这个方法是用低频准确的信息代替相应位置高频的信息）
EstimateFusionValue_02 = AsynChroFusion_highAndlowFrequencies_02(Model_CA,MeasureValue,2);
% 分布式低频传感器做预测
EstimateFusionValue_03 = AsynChroFusion_PredictFusion(Model_CA_21,EstimateValue,EstimateFusionValue_02,2);

%%%仿真结果输出
%并行滤波融合结果
% Title_01.x = '并行滤波融合结果(基本方法) x轴方向';
% Title_01.z = '并行滤波融合结果(基本方法) z轴方向';
% q = output_DemoCKParallel(0,Model_CA_1,TargetRealData.Z2,MeasureValue,EstimateFusionValue_01,Title_01);
%简单并行融合结果和单独的雷达数据比较
Title_02.x = 'Parallel Filtering Result X-axis';
Title_02.z = 'Parallel Filtering Result Z-axis';
Title_02.picture = 'Parallel Filtering Results';
q = output_workRSDF(0,Model_CA_1,TargetRealData.Z,RadarMasureData,EstimateFusionValue_02.x,2,Title_02);
%不带反馈的最优分布式融合结果和单独的雷达数据比较
Title_03.x = 'Distributed Filtering Result X-axis';
Title_03.z = 'Distributed Filtering Result Z-axis';
Title_03.picture = 'Distributed Filtering Results';
q = output_workRSDF(q,Model_CA_1,TargetRealData.Z,RadarMasureData,EstimateFusionValue_03,2,Title_03);

%各个传感器单独使用kalman滤波器结果
for i=10:10:RadarMasureData.L*10
    RadarMasureData.Z{i/10} = RadarMasureData.RMPos(1,i);
    RadarMasureData.Z{i/10} = [RadarMasureData.Z{i/10};RadarMasureData.RMPos(3,i)];
    RadarMasureData.Z{i/10} = [RadarMasureData.Z{i/10};RadarMasureData.RMPos(2,i)];
end
EstimateRadarValue.x = Kalman_Filter(Model_CA_1,RadarMasureData);
Title_04.x = 'Radar Data Filtering Result x-axis';
Title_04.z = 'Radar Data Filtering Result Z-axis';
Title_04.y = 'Radar Data Filtering Result Y-axis';
Title_04.picture = 'Radar Data Filtering Results';
q = output_workRSDF(q,Model_CA_1,TargetRealData.Z,RadarMasureData,EstimateRadarValue.x,3,Title_04);
Title_05.x = 'Sat Data Filtering Result x-axis';
Title_05.z = 'Sat Data Filtering Result z-axis';
Title_05.picture = 'Sat Data Filtering Results';
q = output_workRSDF(q,Model_CA_2,SatRealData.Z,SatMasureData,EstimateSatValue.x,2,Title_05);

%测试用卫星的数据加雷达的Y轴数据进行滤波
% Title_05.x = 'Sat Data Filtering Result x-axis';
% Title_05.z = 'Sat Data Filtering Result z-axis';
% Title_05.y = 'Radar Data Filtering Result Y-axis';
% Title_05.picture = 'Sat Data Filtering Results';
% EstimateSat3Value.x = Kalman_Filter(Model_CA_22,SatMasureData3_add);
% q = output_workRSDF(0,Model_CA_22,TargetRealData.Z2,SatMasureData3_add,EstimateSat3Value.x,3,Title_05);
