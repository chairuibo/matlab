function est_x_update= Filter_Kalman(model,meas)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%程序说明   基本卡尔曼滤波方程
%参数说明   model  运动模型
%           measure_value 所有传感器的量测数据
%版本说明   1.0 （2019-12-25 CRB）    建立文件
%          1.1  (2019-12-26 CRB)     添加更新和预测函数，调整函数结构
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    est_x_update(:,1)= model.x0;
    P_update(:,:,1)= model.p0.^2;

    for k=1:meas.K
         [x_predict,P_predict] = predict_multiple(model,est_x_update(:,k),P_update(:,:,k));    
         [est_temp,P_temp] = update_multiple(meas.Z{k},model,x_predict,P_predict);
         est_x_update(:,k+1)=est_temp;
         P_update(:,:,k+1)=P_temp;
    end 
    %估计出的数值为estimate_value的第二列到最后一列，第一列为初始数据
    est_x_update=est_x_update(:,2:k+1);
end

%Kalman predict
function [x_predict,P_predict] = predict_multiple(model,x,P)     
    F = model.F;
    Q = model.Q;

    x_predict = F*x;
    P_predict = Q+F*P*F'; 
end

%Kalman update
function [x_update,P_update] = update_multiple(measure_Z,model,xPredict,PPredict)
    H = model.H;
    P = PPredict;
    R = model.R;
    
    Z_Predict = H*xPredict;
    S  = R+H*P*H'; %Vs= chol(S); det_S= prod(diag(Vs))^2; inv_sqrt_S= inv(Vs); iS= inv_sqrt_S*inv_sqrt_S'; 
    K  = P*H'/S;
    IKH = (eye(size(P))-K*H);
    x_update = xPredict + K*(measure_Z-Z_Predict);
    P_update = IKH*P*IKH'+K*R*K';
end
