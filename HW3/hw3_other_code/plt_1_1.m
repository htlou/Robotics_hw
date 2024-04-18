clc
clear
t0=0;
t1=1;
t=t0:0.1:t1;
a0=120; % 方便起见以度为单位
a1=0;
a2=-180;
a3=120;

q=a0+a1*power((t-t0),1)+a2*power((t-t0),2)+a3*power((t-t0),3);
%速度
v=a1+2*a2*power((t-t0),1)+3*a3*power((t-t0),2);
%加速度
acc=2*a2+6*a3*power((t-t0),1);
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
saveas(gcf, 'plot_1-1.png');