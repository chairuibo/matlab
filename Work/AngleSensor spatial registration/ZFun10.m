%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 程序名称：量测模型函数
% 说    明：被空间配准函数10调用  SpaceRegistration10
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function z = ZFun10(xTemp,AngleErr,SensorPos)
    x = xTemp(1,:);     %目标的X轴位置
    y = xTemp(4,:);     %目标的Y轴位置    
    
    ObvX_A = SensorPos(1);      %空中站1 X轴位置
    ObvY_A = SensorPos(2);      %空中站1 Y轴位置
    ObvX_B = SensorPos(3);      %空中站2 X轴位置
    ObvY_B = SensorPos(4);      %空中站2 Y轴位置
    ObvX_C = SensorPos(5);      %空中站2 X轴位置
    ObvY_C = SensorPos(6);      %空中站2 Y轴位置
    
    A1 = AngleErr(1,1);
    A2 = AngleErr(2,1);
    A3 = AngleErr(3,1);
    
    z(1,:) = atan2(x-ObvX_A, y-ObvY_A)+A1;      %空中站1对目标的角度观测
    z(2,:) = atan2(ObvX_B-x, ObvY_B-y)+A2;      %空中站2对目标的角度观测
    z(3,:) = atan2(x-ObvX_C, y-ObvY_C)+A3;      %空中站3对目标的角度观测
end
