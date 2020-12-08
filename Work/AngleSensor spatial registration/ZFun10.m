%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% �������ƣ�����ģ�ͺ���
% ˵    �������ռ���׼����10����  SpaceRegistration10
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function z = ZFun10(xTemp,AngleErr,SensorPos)
    x = xTemp(1,:);     %Ŀ���X��λ��
    y = xTemp(4,:);     %Ŀ���Y��λ��    
    
    ObvX_A = SensorPos(1);      %����վ1 X��λ��
    ObvY_A = SensorPos(2);      %����վ1 Y��λ��
    ObvX_B = SensorPos(3);      %����վ2 X��λ��
    ObvY_B = SensorPos(4);      %����վ2 Y��λ��
    ObvX_C = SensorPos(5);      %����վ2 X��λ��
    ObvY_C = SensorPos(6);      %����վ2 Y��λ��
    
    A1 = AngleErr(1,1);
    A2 = AngleErr(2,1);
    A3 = AngleErr(3,1);
    
    z(1,:) = atan2(x-ObvX_A, y-ObvY_A)+A1;      %����վ1��Ŀ��ĽǶȹ۲�
    z(2,:) = atan2(ObvX_B-x, ObvY_B-y)+A2;      %����վ2��Ŀ��ĽǶȹ۲�
    z(3,:) = atan2(x-ObvX_C, y-ObvY_C)+A3;      %����վ3��Ŀ��ĽǶȹ۲�
end
