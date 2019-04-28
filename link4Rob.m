clear;
clc;
L(1)=Link([0 30 0 -pi/2],'modified');
L(2)=Link([0 0 0 pi/2 ],'modified');
L(3)=Link([0 0 0 -pi/2],'modified');
L(4)=Link([0 0 30 0],'modified');

Rbt=SerialLink(L,'name','RehExo');
Rbt.tool=[1 0 0 0;
             0 1 0 -20;
             0 0 1 0;
             0 0 0 1];
Rbt.plot([pi/2 0 0 pi]);
q0=[pi/2 0 0 pi];
q1=[pi/2 0 pi/2 pi/2];
q2=[pi/2 0 pi/2 pi/2];

step=0:.04:1;
traj1=jtraj(q0,q1,step);
traj2=jtraj(q0,q2,step);
traj3=jtraj(q2,q0,step);
hold on
atj=zeros(4,4);
view(-50,30)
ylim([-40,40])
xlim([-100,20])
zlim([-60,60])
%{
for i=1:1:51
    atj=Rbt.fkine(traj1(i,:));
    jta=transpose(atj);
    jta1(i,:)=jta(4,1:3);
    jta=jta1;
    plot2(jta(i,:),'r.')
    Rbt.plot(traj1(i,:))
    plot2(jta1,'b')
    end
%}
a=0;
while a<2
for i=1:1:size(step,2)
    atj=Rbt.fkine(traj2(i,:));
    jta=transpose(atj);
    jta2(i,:)=jta(4,1:3);
    jta=jta2;
    plot2(jta(i,:),'r.')
    Rbt.plot(traj2(i,:))
    plot2(jta2,'b')
end
for i=1:1:size(step,2)
    atj=Rbt.fkine(traj3(i,:));
    jta=transpose(atj);
    jta3(i,:)=jta(4,1:3);
    jta=jta3;
    plot2(jta(i,:),'r.')
    Rbt.plot(traj3(i,:))
    plot2(jta3,'b')
end
a=a+1;
end