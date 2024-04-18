clc
clear
cd /Users/louhantao/Desktop/code/semester_4/Robotics/hw/HW3/hw3_other_code
%轨迹定义条件
%时间
t0=0;
t1=1;
%位置和速度（a）
%利用公式（1-25）求系数
a0=120;
a1=0;
a2=0;
a3=-600;
a4=900;
a5=-360;
%轨迹生成
t=t0:0.1:t1;
%位置
q=a0+a1*power((t-t0),1)+a2*power((t-t0),2)+a3*power((t-t0),3)+...
    a4*power(t-t0,4)+a5*power(t-t0,5);
%速度
v=a1+2*a2*power((t-t0),1)+3*a3*power((t-t0),2)+4*a4*power(t-t0,3)+...
    5*a5*power(t-t0,4);
%加速度
acc=2*a2+6*a3*power((t-t0),1)+12*a4*power(t-t0,2)+20*a5*power(t-t0,3);
%绘图
figure; % 使用figure命令新开一个窗口，避免subplot影响保存
subplot(3,2,1)
plot(t,q,'r');
ylabel('position')
grid on
subplot(3,2,3)
plot(t,v,'b');
ylabel('velocity')
grid on
subplot(3,2,5)
plot(t,acc,'g');
xlabel('(a)');
ylabel('acceleration')
grid on


% 保存图片
saveas(gcf, 'plot_1-2.png');
% 与1-1相同 这也是合理的