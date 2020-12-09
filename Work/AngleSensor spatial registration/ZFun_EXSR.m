%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% �������ƣ�����ģ�ͺ���
% ˵    �������ռ���׼����10����  SpaceRegistration10
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function z = ZFun_EXSR(xTemp,others)
    x = xTemp(1,:);     %Ŀ���X��λ��
    y = xTemp(4,:);     %Ŀ���Y��λ�� 
    ID = others.id;
    SensorPos = others.SensorPos;
    ObvX_A = SensorPos(1);      %����վ1 X��λ��
    ObvY_A = SensorPos(2);      %����վ1 Y��λ��
    ObvX_B = SensorPos(3);      %����վ2 X��λ��
    ObvY_B = SensorPos(4);      %����վ2 Y��λ��
    ObvX_C = SensorPos(5);      %����վ2 X��λ��
    ObvY_C = SensorPos(6);      %����վ2 Y��λ��
    
    switch ID  %�������������µ��˲�
        case 12
            z(1,:) = atan((x-ObvX_A)./(y-ObvY_A));      %����վ1��Ŀ��ĽǶȹ۲�
            z(2,:) = atan((x-ObvX_B)./(y-ObvY_B));      %����վ2��Ŀ��ĽǶȹ۲�
        case 13
            z(1,:) = atan((x-ObvX_A)./(y-ObvY_A));      %����վ1��Ŀ��ĽǶȹ۲�
            z(3,:) = atan((x-ObvX_C)./(y-ObvY_C));      %����վ3��Ŀ��ĽǶȹ۲�
        case 23
            z(2,:) = atan((x-ObvX_B)./(y-ObvY_B));      %����վ2��Ŀ��ĽǶȹ۲�
            z(3,:) = atan((x-ObvX_C)./(y-ObvY_C));      %����վ3��Ŀ��ĽǶȹ۲�  
        otherwise 
            z(1,:) = atan((x-ObvX_A)./(y-ObvY_A));      %����վ1��Ŀ��ĽǶȹ۲�
            z(2,:) = atan((x-ObvX_B)./(y-ObvY_B));      %����վ2��Ŀ��ĽǶȹ۲�
            z(3,:) = atan((x-ObvX_C)./(y-ObvY_C));      %����վ3��Ŀ��ĽǶȹ۲�
    end
end
