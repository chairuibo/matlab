%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%程序说明：  集中式估计融合并行滤波法演示程序
%           运动模型采用CV模型和CA模型模拟两种不同精度的传感器
%           假设CA模型的传感器的精度较高、CV模型的传感器精度较差
%版本说明   1.0 （2019-12-25 CRB）    建立运动模型和融合滤波模型 
%               (2019-12-26 CRB）    仿真
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
close all
clear
clc

Model_CA_1 = buildModel_CA(10);       %精度差
Model_CA_2 = buildModel_CA(2);        %精度高
Model = [Model_CA_1;Model_CA_2];
TruthValue = getTruthData(Model_CA_1);
MeasureValue_1 = getMeasureData(Model_CA_1,TruthValue);
MeasureValue_2 = getMeasureData(Model_CA_2,TruthValue);
MeasureValue = [MeasureValue_1;MeasureValue_2];
EstimateValue = Kalman_CentralizedFusionParallel(Model,MeasureValue,2);
output_DemoCKParallel(Model_CA_1,TruthValue,MeasureValue,EstimateValue);
