%modified_GRNN controller


clear;
clc
close all;

ts=0.01;
u_2=0;
steps=1500;
y=ones(1,steps);
u=y;
t=0;
up=10;
down=-10;

sigma=0.1;
p1=0;
iw=rand(2,5);

x1(1:4)=0;
x2(1:4)=0;

ninput=2;

b11=0.002;
b2=0.055;
c1=0.01;
c2=0.01;

parameters(1)=b11;
parameters(2)=b2;
parameters(3)=c1;
parameters(4)=c2;


fix_the_model=steps;



big=3;
for k=4:1500
    
    %%% ======Reference Signals===========
%     yd(k)=0.5*sin(pi*k/100)+0.5*sin(pi*k/200)+2;
%     yd(k)=sign(sin(2*k/100))+3;
%     yd(k)=sawtooth(pi*k/100,1/2)+3;    
    yd(k)=3;   
%     yd=[2.0*ones(1,300) 3*ones(1,300)  4*ones(1,300) 3*ones(1,300) 2.0*ones(1,300)];
    
        
    % ========= Plant==============
    y2(k)=1.5*(u_2+1*y(k-1)-y(k-2)-1.0*sin(y(k-1))); %time varying robot manipulator model obtained from
    % http://dx.doi.org/10.1016/j.isatra.2015.09.003
    % ========= Plant==============
    
    
    e(k)=y2(k)-yd(k);
    de(k)=e(k)-e(k-1);
    
    %%% Simp_NN part
    
    d1=dist([e(k);de(k)],iw);

    hi=exp(-d1/2*sigma.^2);
    h1=norm(hi,2);
    p1=p1-0.1*e(k);
    
    u(k)=h1*p1;
    
    
    
    
    %%%====== Simp_NFS part========
    Data=[e(k-big+2:k);de(k-big+2:k); u(:,k-big+2:k)]';
    [ur,Weight,rule,time]=Simp_NFS(Data,ninput,parameters);
    
    u_NFS=ur(:,end);
    
    u_SNN=u(:,k);
    
    %%%%control singal of HAC (u)
            u(:,k)=1*u_SNN+1*u_NFS;
    
    u(:,k)=min(up, max(down, u(:,k)));
    
    
    
    u_2=u(k);
    
    
    kk(k)=k;
    t=t+ts;
end
RMSE_siso=sqrt(mse(yd(1,1:end),y2(1,1:end)));
% plot(kk,yd,kk,y2,':','linewidth',2);


clearvars -except RMSE_siso yd y2 u



