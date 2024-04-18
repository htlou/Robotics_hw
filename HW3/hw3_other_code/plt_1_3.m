clc;
clear;
%% 计算两段一共8个系数
a10=60;
a11=0;
a12=180*9/8;
a13=-180*19/24;
a20=120; 
a21=-180/8;
a22=-180*5/4;
a23=180*7/8;
%% 计算各个时刻的角度角加速度等
i=1;
for t = 0:0.01:1
    theta_t1(i)=a10+a11*t+a12*t^2+a13*t^3; % 角度
    dtheta_t1(i)=a11+2*a12*t+3*a13*t^2; % 角速度
    ddtheta_t1(i)=2*a12+6*a13*t; % 角加速度
    i=i+1;
end
i=1;
for t = 0:0.01:1
    theta_t2(i)=a20+a21*t+a22*t^2+a23*t^3; % 角度
    dtheta_t2(i)=a21+2*a22*t+3*a23*t^2; % 角速度
    ddtheta_t2(i)=2*a22+6*a23*t; % 角加速度
    i=i+1;
end
% 把两段接上
theta_t=[theta_t1 theta_t2(2:101)]; %theta_t2需要去掉第一项
dtheta_t=[dtheta_t1 dtheta_t2(2:101)];
ddtheta_t=[ddtheta_t1 ddtheta_t2(2:101)];
%% 绘图
subplot(4,1,1),plot(0:0.01:2,theta_t,'r','LineWidth',1);
xlabel('时间'),ylabel('角度');
subplot(4,1,2),plot(0:0.01:2,dtheta_t,'r','LineWidth',1);
xlabel('时间'),ylabel('角速度');
subplot(4,1,3),plot(0:0.01:2,ddtheta_t,'r','LineWidth',1);
xlabel('时间'),ylabel('角加速度');

% 保存图片
saveas(gcf, 'plot_1-3.png');