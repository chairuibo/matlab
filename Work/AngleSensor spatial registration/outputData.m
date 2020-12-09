function out = outputData(index,model,meas,truth,est)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%����˵����  ���������
%����˵��   index ͼ�����
%           model �˶�ģ��
%           truth ��ֵ
%           meas ����ֵ
%           est ����ֵ
%�汾˵��   1.0 ��2020-01-12 CRB��    �����ļ�
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
   
    L = length(est);
    Tick=(1:L)*model.T;
    
%% ת������
    EstValue = cell2mat(est);
    Pos_truth_x = truth(1,:);   %Ŀ��X����ʵ����
    Pos_meas_x = meas(1,:);     %Ŀ��X����������
    Pos_est_x = EstValue(1,:);  %Ŀ��X���������
    Pos_truth_y=truth(2,:);     %Ŀ��X����ʵ����
    Pos_meas_y=meas(2,:);       %Ŀ��X����������
    Pos_est_y =EstValue(4,:);   %Ŀ��X���������
    
%% ��ͼ
    %%λ�ù��ƱȽ�
    %y�᷽��
    out = index+1;
    F=figure(out);
    set(F,'name','Ŀ��X���˶�����','position',[250 200 500 400]);
    PosErr_est_x = Pos_est_x-Pos_truth_x;
    s(1) = subplot(211);
    plot(Tick,Pos_truth_x,'r',Tick,Pos_est_x,'b');
%     title(s(1),Title.y);
    legend('λ����ֵ','�˲�ֵ');
    xlabel('ʱ�� t/s');
    ylabel('Xλ�� /m');
    subplot(212);
    plot(Tick,PosErr_est_x,'g');
    legend('�˲����');
    xlabel('ʱ�� t/s');
    ylabel('��� /m');
    
    out = out+1;
    F=figure(out);
    set(F,'name','Ŀ��Y���˶�����','position',[350 100 500 400]);
    PosErr_est_y = Pos_est_y-Pos_truth_y;
    s(1) = subplot(211);
    plot(Tick,Pos_truth_y,'r',Tick,Pos_est_y,'b');
%     title(s(1),Title.y);
    legend('λ����ֵ','�˲�ֵ');
    xlabel('ʱ�� t/s');
    ylabel('Xλ�� /m');
    subplot(212);
    plot(Tick,PosErr_est_y,'g');
    legend('�˲����');
    xlabel('ʱ�� t/s');
    ylabel('��� /m');
end
