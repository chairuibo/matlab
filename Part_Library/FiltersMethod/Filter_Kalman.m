function est_x_update= Filter_Kalman(model,meas)
est_x_update(:,1)= model.x0;
P_update(:,:,1)= model.p0.^2;

for k=1:meas.K
     [x_predict,P_predict] = kalman_predict_multiple(model,est_x_update(:,k),P_update(:,:,k));    
     [est_temp,P_temp] = kalman_update_multiple(meas.Z{k},model,x_predict,P_predict);
     est_x_update(:,k+1)=est_temp;
     P_update(:,:,k+1)=P_temp;
end 
est_x_update=est_x_update(:,2:k+1);

function [m_predict,P_predict] = kalman_predict_multiple(model,m,P)     
F = model.F;
Q = model.Q;

lenth = length(m);
m_predict = F*m;
P_predict = Q+F*P*F'; 