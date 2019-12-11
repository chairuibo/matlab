clc;
clear all;

L=10;
outIndex=zeros(1,L);
u=unifrnd(0,1,1,L);
u1=sort(u);
A=[2,8,2,7,3,5,5,1,4,6];
w=A./sum(A);
cdf=cumsum(w);
 i=1;
 for j=1:L
     while (i<=L) && (u1(i)<=cdf(j))
         outIndex(i)=j;
         i=i+1;
     end
 end 