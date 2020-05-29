function est_x_update = Kalman_Smooth(model,meas )
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%程序说明   基本卡尔曼平滑方程--固定区间平滑
%参数说明   model  运动模型
%           measure_value 所有传感器的量测数据
%版本说明   1.0 （2019-12-30 CRB）    建立文件
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    L = meas.L*10;
    est_x_update = cell(L+1,1);
    P_update = cell(L+1,1);
    est_x_update{1}= model.x0;
    P_update{1}= model.p0;
    count = 1;
    for k=1:L
        if mod(k,10) ~= 1
            [x_predict,P_predict] = predict_multiple(model,est_x_update{k},P_update{k});  
            est_x_update{k+1} = x_predict;
            P_update{k+1} = P_predict;
        else
            [x_predict,P_predict] = predict_multiple(model,est_x_update{k},P_update{k});   
            [est_temp,P_temp] = update_multiple(meas.Z{count},model,x_predict,P_predict);
            est_x_update{k+1}=est_temp;
            P_update{k+1}=P_temp;
            if k > 1
                for j=1:9
                    %%%%%%%%%%%%%%%%%smooth_multiple(model,xPredict,xNFilter,xFilter,PPredict,PFilter,PNsmooth)
                    [x_temp,P_temp] = smooth_multiple(model,est_x_update{k-j},est_x_update{k},est_x_update{k-j+1},P_update{k-j},P_update{k-j+1},P_update{k});
                    est_x_update{k-j}=x_temp;
                    P_update{k-j}=P_temp;
                end
            end
            count = count+1;
        end
    end
    
    L_x = L;
    t=1:L_x+1;
    figure(1);
    x = cell2mat(est_x_update);
    P = cell2mat(P_update);
    plot(t,x,'r');
    legend('x平滑值');
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

%Kalman smooth
function [x_smooth,P_smooth] = smooth_multiple(model,xPredict,xNFilter,xFilter,PPredict,PFilter,PNsmooth)
    F = model.F;
    xN = xNFilter;
    xF = xFilter;
    xP = xPredict;
    PP = PPredict;
    PF = PFilter;
    PN = PNsmooth;
    
    As = PF*F'/PP;
    x_smooth = xF + As*(xN - xP);
    P_smooth = PF + As*(PN-PP)*As';
end
