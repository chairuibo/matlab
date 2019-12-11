function k_plot(model,truth,est)

temp=(1:truth.K).*model.T;
a=truth.X;
pos_x=zeros(1,truth.K);
pos_y=zeros(1,truth.K);
vel_x=zeros(1,truth.K);
vel_y=zeros(1,truth.K);
for i=1:truth.K
    pos_x(i)=a{i}(1);
    pos_y(i)=a{i}(3);
    vel_x(i)=a{i}(2);
    vel_y(i)=a{i}(4);
end 
%%位置估计比较
figure;
subplot(211);
plot(temp,pos_x,'b');
hold on;
plot(temp,est(1,:),'r');
subplot(212);
plot(temp,pos_y,'b');
hold on;
plot(temp,est(3,:),'r');

%%速度估计比较
figure;
subplot(211);
plot(temp,vel_x,'b');
hold on;
plot(temp,est(2,:),'r');
subplot(212);
plot(temp,vel_y,'b');
hold on;
plot(temp,est(4,:),'r');
