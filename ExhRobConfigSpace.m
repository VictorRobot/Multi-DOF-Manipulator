clear;
clc;
D1=30;
D2=30;
D3=20;
% [theta d a alpha]
L(1)=Link([0 D1 0 -pi/2],'modified');
L(2)=Link([0 0 0 pi/2 ],'modified');
L(3)=Link([0 0 0 -pi/2],'modified');
L(4)=Link([0 0 D2 0],'modified');
L(5)=Link([0 D3 0 pi/2],'modified');

RehExo=SerialLink(L,'name','RehExo');
RehExo.plot([pi/2 0 0 pi 0]);
q0=[pi/6 -pi/2 pi/2 pi 0];
q1=[pi/6 -pi/2 pi/2 pi/2 0];
q2=[pi/6 -pi/2 pi pi/2 0];

step=0:.04:1;
traj1=jtraj(q0,q1,step);
traj2=jtraj(q0,q2,step);
traj3=jtraj(q2,q0,step);

%把目标点转换成轴坐标轨迹点，Inverse Kinematics
hold on
atj=zeros(4,4);
%视角和坐标范围
view(-110,30)
xlim([-60,60])
ylim([-30,90])
zlim([-60,60])

%10°间隔 pi/180会死机
%m=0:pi/8:pi;
stepDgr=pi/12;
m1=-pi/6:stepDgr:2*pi/3;%肩部外展/内收 -30° 到 120°
m2=-pi/4:stepDgr:pi/2;%肩部外旋/内旋 -45°到90°
m3=-pi/6:stepDgr:17*pi/18;%肩部弯曲/伸展 -30°到170°
m4=-5*pi/18:stepDgr:pi/2;%肘部弯曲 0°到140°

for i=1:1:size(m1,2)
    theta1=m1(i);
    for j=1:1:size(m2,2)
        theta2=m2(j);
        for k=1:1:size(m3,2)
            theta3=m3(k);
            for l=1:1:size(m4,2)
                theta4=m4(l);
                qs=[theta1 theta2 theta3 theta4 0];
                atj=RehExo.fkine(qs);
                jta=transpose(atj);%把4*4旋转矩阵转置
                %取第4行的前三个元素为位置坐标x,y,z；
                plot2(jta(4,1:3),'r.');
                %RehExo.plot(q0);
                %RehExo.plot(qs);
                %plot2(jta2,'b')
            end
        end
    end
end

