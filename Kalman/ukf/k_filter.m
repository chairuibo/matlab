function m_update= k_filter(model,meas)

m_update(:,1)= model.x0;
P_update(:,:,1)= model.p0.^2;

filter.ukf_alpha= 1;                %scale parameter for UKF - choose alpha=1 ensuring lambda=beta and offset of first cov weight is beta for numerical stability
filter.ukf_beta= 2;                 %scale parameter for UKF
filter.ukf_kappa= 2;   

for k=1:meas.K
    
       [m_predict,P_predict] = ukf_predict_multiple(model,m_update(:,k),P_update(:,:,k),filter.ukf_alpha,filter.ukf_kappa,filter.ukf_beta); 
       [qz_temp,m_temp,P_temp] = ukf_update_multiple(meas.Z{k},model,m_predict,P_predict,filter.ukf_alpha,filter.ukf_kappa,filter.ukf_beta);
     m_update(:,k+1)=m_temp;
     P_update(:,:,k+1)=P_temp;
     
end 
m_update=m_update(:,2:k+1);