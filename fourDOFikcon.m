clear;
clc;
%%
% create links using Modified D-H parameters
L(1)=Link([0 30 0 -pi/2],'modified');
L(2)=Link([0 0 0 pi/2 ],'modified');
L(3)=Link([0 0 0 -pi/2],'modified');
L(4)=Link([0 0 30 0],'modified');
%%
%define link mass
L(1).m = 4.43;
L(2).m = 10.2;
L(3).m = 4.8;
L(4).m = 1.18;
%%
%define center of gravity
L(1).r = [ 0 0 -0.08];
L(2).r = [ -0.216 0 0.026];
L(3).r = [ 0 0 0.216];
L(4).r = [ 0 0.02 0];
%%
%define link inertial as a 6-element vector
%interpreted in the order of [Ixx Iyy Izz Ixy Iyz Ixz]
L(1).I = [ 0.195 0.195 0.026 0 0 0];
L(2).I = [ 0.588 1.886 1.470 0 0 0];
L(3).I = [ 0.324 0.324 0.017 0 0 0];
L(4).I = [ 3.83e-3 2.5e-3 3.83e-3 0 0 0];
%%
% set limits for joints
L(1).qlim=[deg2rad(-30) deg2rad(120)];
L(2).qlim=[deg2rad(-45) deg2rad(90)];
L(3).qlim=[deg2rad(-30) deg2rad(170)];
L(4).qlim=[deg2rad(0) deg2rad(180)];
%%
% build the robot model,add tool
RehExo = SerialLink(L, 'name','RehExo');
RehExo.tool=[1 0 0 0;
             0 1 0 -20;
             0 0 1 0;
             0 0 0 1];
tool.m=2;
tool.r=[0 0 0];
tool.I=[1 1 1 0 0 0];
RehExo.gravity=[0 0 9.81];
%%
qInit = [pi/2 0 0 pi];
q1=[pi/2 0 pi/2 pi/2];
RehExo.plot(qInit);
%T0 = RehExo.fkine(qInit);
step = 0:.04:1;
[traj1,traj1d,traj1dd]=jtraj(qInit,q1,step);
[traj2,traj2d,traj2dd]=jtraj(q1,qInit,step);
%%
a=0;
qReach(4,4)=0;
while a<2
    q0=qInit;
for i=1:1:size(step,2)
    atj=RehExo.fkine(traj1(i,:));% Target Transform Matrix
    qReach=RehExo.ikcon(atj,q0);%robot.ikcon(T, q0) specify the initial joint coordinates q0 used for the minimisation. 
    q0=qReach;
    jta=transpose(atj);
    jta1(i,:)=jta(4,1:3);
    jta=jta1;
    %plot2(jta(i,:),'r.')
    RehExo.plot(qReach)
    %plot2(jta1,'b')
end
    q0=q1;
for i=1:1:size(step,2)
    atj=RehExo.fkine(traj2(i,:));
    qReach=RehExo.ikcon(atj,q0);
    q0=qReach;
    jta=transpose(atj);
    jta2(i,:)=jta(4,1:3);
    jta=jta2;
    %plot2(jta(i,:),'r.')
    RehExo.plot(qReach)
    %plot2(jta2,'b')
end
a=a+1;
end
%%
    % compute inverse dynamics using recursive Newton-Euler algorithm
    % TAU = R.rne(Q, QD, QDD) is the joint torque required for the robot R to
    % achieve the specified joint position Q (1xN), velocity QD (1xN) and
    % acceleration QDD (1xN), where N is the number of robot joints.
    Tauf = rne(RehExo,traj1,traj1d,traj1dd);
    % Integrate forward dynamics
    % [T,q,qd] = R.fdyn(T, torqfun) integrates the dynamics of the robot 
    % over the time interval 0 to T and returns vectors of time T, 
    % joint position q and joint velocity qd. The initial joint position and velocity are zero.
    % The torque applied to the joints is computed by the user-supplied control function torqfun: 
    % TAU = TORQFUN(T, Q, QD)
    % where q and qd are the manipulator joint coordinate and velocity state respectively, and T is the current time. 
    % [ti,q,qd] = R.fdyn(T, torqfun, q0, qd0) as above but allows the initial joint position and velocity to be specified. 
    [t1f,Qf,Qdf] = RehExo.fdyn(2,Tauf(1,:),traj1(1,:), traj1d(1,:));
    Taub = rne(RehExo,traj2,traj2d,traj2dd);
    [t1b,Qb,Qdb] = RehExo.fdyn(2,Taub(5,:),traj2(5,:), traj2d(5,:));
