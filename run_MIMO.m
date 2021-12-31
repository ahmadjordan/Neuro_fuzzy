clc
clear
steps=1500;
y=ones(2,steps);
yg=y;
y1=ones(1,steps);
y2=y1;
u1=yg;
u2=yg;
u=[u1 u2];
yd1=y1;
yd2=y1;
yd=y;
e=y;
de=e;

dt=0.01;
t=0;
h=2;

b1=zeros(2,1);

ninput=4;

b11=0.001;
b2=0.055;
c1=0.01;
c2=0.01;

parameters(1)=b11;
parameters(2)=b2;
parameters(3)=c1;
parameters(4)=c2;


% fix_the_model=steps;




up=4;
down=-4;
big=3;
p1=[0;0];
iw=0.1*ones(5,4);
sigma=1;


kp=0.2;
kd=0.2;
ki=0.2;

load noise_mimo
Ts=0.01;
for k=big:steps-1
    % ========= Plant==============
    y1(k)=0.5*y1(k-1)+u1(k-2)+0.1*y2(k-1)*u1(k-1)+0.5*noise(k-1)+0.2*y1(k-2)*noise(k-2)+noise(k);
    y2(k)=0.9*y2(k-2)+u2(k-1)+0.2*y2(k-1)*u2(k-2)+0.5*noise(k-1)+0.1*y2(k-1)*noise(k-2)+noise(k);
    % ========= Plant==============
    
    %%% Ref 1
%             yd1(k)=3*sin(2*k*Ts);
%             yd2(k)=2*cos(3*k*Ts);
    
    %%% Ref 2
    yd1(k)=sawtooth(1*k*Ts,1/2)+3;
    yd2(k)=sawtooth(1.5*k*Ts,1/2)+3;
    
    yd(:,k)=[yd1(k);yd2(k)];
    
    e1(k)=yd1(k)-y1(k);
    e2(k)=yd2(k)-y2(k);
    e(:,k)=[e1(k);e2(k)];
    
    de1(k)=y1(k)-y1(k-1);
    de2(k)=e2(k)-e2(k-1);
    de(:,k)=[de1(k);de2(k)];
    
    
    %%% Simp_NN part
    
    d1=dist(iw,[e(:,k);de(:,k)]);
    
    hi=exp(-d1/2*sigma.^2);
    h1=abs(mean(mean(hi)));
    p1=p1+0.07*e(:,k);
    
    u(:,k)=p1*h1;
    
    
    
    %%%====== Simp_NFS part ========
    Data=[e1(k-big+2:k);e2(k-big+2:k);de1(k-big+2:k);de2(k-big+2:k); u(:,k-big+2:k)]';
    [ur,Weight,rule,time]=Simp_NFS(Data,ninput,parameters);
    
    
    u_NFS=ur(:,end);
    
    u_SNN=u(:,k);
    
    %%%%control singal of HAC (u)
    
%     u(:,k)=1*u_SNN+1*u_NFS;
    
    u(:,k)=min(up, max(down, u(:,k)));
    
    u1(k)=u(1,k);
    u2(k)=u(2,k);
    
    
    
    %%% ====SMC/ PID======
    %     if (abs(e1(k))>0.1)
    %         kp=kp+0.001;
    %         kd=kd+0.001;
    %         ki=ki+0.001;
    %     end
    %     u(k)=1.0*(kp*e1(k)+kd*de1(k));
    
    tt(k)=t;
    kk(k)=k;
    
    y(:,k)=[y1(k);y2(k)];
    
    % ====Error calculation=====
    err(k)=y(1,k)-yd(1,k);
    err2(k)=err(k).^2;
    %
    
    t=t+dt;
    
end

RMSE_1=sqrt(mse(yd(1,1:end),y(1,1:end)));
RMSE_2=sqrt(mse(yd(2,1:end),y(2,1:end)));

clearvars -except RMSE_1 RMSE_2 yd y 
% fig1=figure(1);
% plot(kk, yd(1,1:k),kk,y(1,1:k),'--r',kk,yd(2,1:k),kk,y(2,1:k),':b','LineWidth',2);



% set(gca,'FontName','Times New Roman','fontweight','bold','FontSize',29);
% set(gcf,'units','normalized','outerposition',[0 0 1 1])
% %legend('y_d','y_{HPEC}','y_{Simp\_NN}',);
% legend('y_{d_1}','y_{{Simp\_NN}_1}','y_{d_2}','y_{{Simp\_NN}_2}','Orientation','horizontal');
% xlabel('number of steps (k)');
% ylabel('y');
% savefig('simp_nn_mimo.fig')
% saveas(fig1,'mimo_simp_nn','epsc')
%
% fig2=figure(2)
% plot(kk, yd(1,1:k),kk,y(1,1:k),'--r',kk,yd(2,1:k),kk,y(2,1:k),':b','LineWidth',2);
% set(gca,'FontName','Times New Roman','fontweight','bold','FontSize',29);
% set(gcf,'units','normalized','outerposition',[0 0 1 1])
% %legend('y_d','y_{HPEC}','y_{Simp\_NN}',);
% legend('y_{d_1}','y_{{HPEC}_1}','y_{d_2}','y_{{HPEC}_2}','Orientation','horizontal');
% xlabel('number of steps (k)');
% ylabel('y');
% savefig('hpec_mimo.fig')
% saveas(fig2,'mimo_comb1','epsc')
