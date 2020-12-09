%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 程序名称：目标运动模型函数（CA模型）
% 说    明：被空间配准函数5调用  SpaceRegistration5
%           被空间配准函数6调用  SpaceRegistration6
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function x = XFun_EXSR(xTemp,model)
    T=model.T;
    
    x = model.F*xTemp;
%     x(1,:) = xTemp(1,:)+T*xTemp(2,:)+T^2/2*xTemp(3,:);  %目标 X轴运动的位置
%     x(2,:) = xTemp(2,:)+T*xTemp(3,:);                   %目标 X轴运动的速度
%     x(3,:) = xTemp(3,:);                                %目标 X轴运动的加速度
%     x(4,:) = xTemp(4,:)+T*xTemp(5,:)+T^2/2*xTemp(6,:);  %目标 Y轴运动的位置
%     x(5,:) = xTemp(5,:)+T*xTemp(6,:);                   %目标 Y轴运动的速度
%     x(6,:) = xTemp(6,:);                                %目标 Y轴运动的加速度

end
