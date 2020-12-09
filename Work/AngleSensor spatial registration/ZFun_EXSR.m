%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 程序名称：量测模型函数
% 说    明：被空间配准函数10调用  SpaceRegistration10
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function z = ZFun_EXSR(xTemp,others)
    x = xTemp(1,:);     %目标的X轴位置
    y = xTemp(4,:);     %目标的Y轴位置 
    ID = others.id;
    SensorPos = others.SensorPos;
    ObvX_A = SensorPos(1);      %空中站1 X轴位置
    ObvY_A = SensorPos(2);      %空中站1 Y轴位置
    ObvX_B = SensorPos(3);      %空中站2 X轴位置
    ObvY_B = SensorPos(4);      %空中站2 Y轴位置
    ObvX_C = SensorPos(5);      %空中站2 X轴位置
    ObvY_C = SensorPos(6);      %空中站2 Y轴位置
    
    switch ID  %不考虑误差情况下的滤波
        case 12
            z(1,:) = atan((x-ObvX_A)./(y-ObvY_A));      %空中站1对目标的角度观测
            z(2,:) = atan((x-ObvX_B)./(y-ObvY_B));      %空中站2对目标的角度观测
        case 13
            z(1,:) = atan((x-ObvX_A)./(y-ObvY_A));      %空中站1对目标的角度观测
            z(3,:) = atan((x-ObvX_C)./(y-ObvY_C));      %空中站3对目标的角度观测
        case 23
            z(2,:) = atan((x-ObvX_B)./(y-ObvY_B));      %空中站2对目标的角度观测
            z(3,:) = atan((x-ObvX_C)./(y-ObvY_C));      %空中站3对目标的角度观测  
        otherwise 
            z(1,:) = atan((x-ObvX_A)./(y-ObvY_A));      %空中站1对目标的角度观测
            z(2,:) = atan((x-ObvX_B)./(y-ObvY_B));      %空中站2对目标的角度观测
            z(3,:) = atan((x-ObvX_C)./(y-ObvY_C));      %空中站3对目标的角度观测
    end
end
