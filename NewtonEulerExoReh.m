% 4自由度康复外骨骼机器人 Newton-Euler 递归算法
clear all;
clc;
syms g L0 L1 L2 Lc1 Lc2 real;
syms c1 s1 c2 s2 c3 s3 c4 s4 real;
syms kr1 kr2 kr3 kr4 kr5 m1 m2 m3 m4 m5 Im1 Im2 Im3 Im4 Im5 real;
syms q1 q2 q3 q4 dq1 dq2 dq3 dq4 dq5 ddq1 ddq2 ddq3 ddq4 ddq5 real;
z0=[0;0;1];
zm=[0;0;1];
ddp0=[0;g;0]; % ddp0 = g
w=sym(zeros(3,6)); % 连杆的角速度
f=sym(zeros(3,6)); % 连杆i-1对连杆i施加的作用力
u=sym(zeros(3,6)); % 连杆i-1对连杆i关于坐标系i-1原点的力矩
Tau=sym(zeros(6,1)); % 关节转矩
dw=sym(zeros(3,6)); % 连杆的角加速度
dwm=sym(zeros(3,6)); % 转子的角加速度
ddp=sym(zeros(3,6));
ddp(2,1)=g;          % 坐标系i原点的线加速度
ddpc=sym(zeros(3,6)); % 质心C_i的线加速度
kr=[0 kr1 kr2 kr3 kr4 kr5];
m=[0 m1 m2 m3 m4 m5 0];
q=[0;q1;q2;q3;q4;0];
dq=[0;dq1;dq2;dq3;dq4;0];
ddq=[0;ddq1;ddq2;ddq3;ddq4;0];
Im=[0 Im1 Im2 Im3 Im4 Im5];
R(:,:,1)=[cos(q1) -sin(q1) 0;
          0 0 1;
          -sin(q1) -cos(q1) 0];
R(:,:,2)=[cos(q2) -sin(q2) 0;
          0 0 1;
          -sin(q2) cos(q2) 0];
R(:,:,3)=[cos(q3) -sin(q3) 0;
          0 0 1;
         -sin(q3) -cos(q3) 0];
R(:,:,4)=[cos(q4) -sin(q4) 0;
          sin(q4) cos(q4) 0;
          0 0 1];
R(:,:,5)=[1 0 0;
          0 1 0;
          0 0 1];
r11=[0 0 0 0 Lc1 Lc2;
     0 0 0 0 0 0;
     0 0 0 0 0 0];
r01=[0 L0 0 0 L1 L2;
     0 0 0 0 0 0;
     0 0 0 0 0 0];
% 正向运算 角速度，角加速度，速度，加速度，转子的角加速度
for i=2:1:6
    w(:,i)    = R(:,:,i-1)'*(w(:,i-1)+dq(i)*z0);
    dw(:,i)   = R(:,:,i-1)'*(dw(:,i-1)+ddq(i)*z0+cross(dq(i)*w(:,i-1),z0));
    ddp(:,i)  = R(:,:,i-1)'*ddp(:,i-1)+cross(dw(:,i),r01(:,i))+...
                cross(w(:,i),cross(w(:,i),r01(:,i)));
    ddpc(:,i) = ddp(:,i)+cross(w(:,i),r11(:,i))+...
                cross(w(:,i),cross(w(:,i),r11(:,i)));
    dwm(:,i)  = dw(:,i-1)+kr(i)*ddq(i)*zm+cross(kr(i)*dq(i)*w(:,i-1),zm);
end
% 逆向运算 力，转矩，关节扭矩
for j=5:-1:1
    f(:,j) = R(:,:,j)*f(:,j+1)+m(j)*ddpc(:,j);
    u(:,j) = cross(-f(:,j),(r01(:,j)+r11(:,j)))+R(:,:,j)*u(:,j+1)+...
             cross(R(:,:,j)*f(:,j+1),r11(:,j))+kr(j+1)*ddq(j+1)*Im(j+1)*zm+...
             cross(kr(j+1)*dq(j+1)*Im(j+1)*w(:,j),zm);
    Tau(j) = u(:,j)'*z0+kr(j)*Im(j)*dwm(:,j)'*zm;
end
     


