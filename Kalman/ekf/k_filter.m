function m_update= k_filter(model,meas)

m_update(:,1)= model.x0;
P_update(:,:,1)= model.p0.^2;

for k=1:meas.K
    
     [m_predict,P_predict] = ekf_predict_multiple(model,m_update(:,k),P_update(:,:,k));    
     [qz_temp,m_temp,P_temp] = ekf_update_multiple(meas.Z{k},model,m_predict,P_predict);
     m_update(:,k+1)=m_temp;
     P_update(:,:,k+1)=P_temp;
     
end 
m_update=m_update(:,2:k+1);