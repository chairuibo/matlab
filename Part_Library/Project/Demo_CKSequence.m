%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%����˵����  ����ʽ�����ں�����˲�����ʾ����
%           �˶�ģ�Ͳ���CVģ�ͺ�CAģ��ģ�����ֲ�ͬ���ȵĴ�����
%           ����CAģ�͵Ĵ������ľ��Ƚϸߡ�CVģ�͵Ĵ��������Ƚϲ�
%�汾˵��   1.0 ��2019-12-27 CRB��    �ڲ����˲����̵Ļ������޸ĵõ�
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
close all
clear
clc

Model_CA_1 = buildModel_CA(1,4);       %���Ȳ�
Model_CA_2 = buildModel_CA(1,2);        %���ȸ�
Model = [Model_CA_1;Model_CA_2];
TruthValue = getTruthData(Model_CA_1);
MeasureValue_1 = getMeasureData(Model_CA_1,TruthValue);
MeasureValue_2 = getMeasureData(Model_CA_2,TruthValue);
MeasureValue = [MeasureValue_1;MeasureValue_2];
EstimateValue = Kalman_CentralizedFusionParallel(Model,MeasureValue,2);
output_DemoCKParallel(Model_CA_1,TruthValue,MeasureValue,EstimateValue);
