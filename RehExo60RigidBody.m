%% Build Manipulator Robot Using Modified Denavit-Hartenberg Parameters
%%
clear;
clc;
D1=30;
D2=30;
D3=20;
% [a alpha d theta]
mdhparams = [0     -pi/2	    D1   	0;
            0	   -pi/2     0       0; 
            0	  pi/2	    0      	0;
            D2       0	    0   	0;
            0      0  	D3  	0];   
%%
% Create a rigid body tree object to build the robot.
robot = robotics.RigidBodyTree;
robot.Gravity=[0 0 -9.81];

%%
% Create the first rigid body and add it to the robot. To add a rigid body:
%
% # Create a |RigidBody| object and give it a unique name.
% # Create a |Joint| object and give it a unique name.
% # Use |setFixedTransform| to specify the body-to-body transformation
% using DH parameters. The last element of the DH parameters, |theta|, is
% ignored because the angle is dependent on the joint position.
% # Call |addBody| to attach the first body joint to the base frame of the
% robot.
%{
body1 = robotics.RigidBody('body1');
jnt1 = robotics.Joint('jnt1','revolute');
body1.Mass=2;
body1.CenterOfMass=[0 0 0];
body1.Inertia=[1 1 1 0 0 0];

setFixedTransform(jnt1,mdhparams(1,:),'mdh');
body1.Joint = jnt1;

addBody(robot,body1,'base')
%}

%%
% Create and add other rigid bodies to the robot. Specify the
% previous body name when calling |addBody| to attach it. Each fixed
% transform is relative to the previous joint coordinate frame.
body1 = robotics.RigidBody('body1');
jnt1 = robotics.Joint('jnt1','revolute');
body2 = robotics.RigidBody('body2');
jnt2 = robotics.Joint('jnt2','revolute');
body3 = robotics.RigidBody('body3');
jnt3 = robotics.Joint('jnt3','revolute');
body4 = robotics.RigidBody('body4');
jnt4 = robotics.Joint('jnt4','revolute');
body5 = robotics.RigidBody('body5');
jnt5 = robotics.Joint('jnt5','revolute');
jnt1.HomePosition = pi/2;
jnt2.HomePosition = 0;
jnt3.HomePosition = 0;
jnt4.HomePosition = 0;
jnt5.HomePosition = 0;
setFixedTransform(jnt1,mdhparams(1,:),'mdh');
setFixedTransform(jnt2,mdhparams(2,:),'mdh');
setFixedTransform(jnt3,mdhparams(3,:),'mdh');
setFixedTransform(jnt4,mdhparams(4,:),'mdh');
setFixedTransform(jnt5,mdhparams(5,:),'mdh');

body1.Joint = jnt1;
body2.Joint = jnt2;
body3.Joint = jnt3;
body4.Joint = jnt4;
body5.Joint = jnt5;
%%
body1.Mass=2;
body1.CenterOfMass=[0 0 0];
body1.Inertia=[1 1 1 0 0 0];
body2.Mass=1;
body2.CenterOfMass=[0 0 0];
body2.Inertia=[1 1 1 0 0 0];
body3.Mass=1;
body3.CenterOfMass=[0 0 0];
body3.Inertia=[1 1 1 0 0 0];
body4.Mass=1;
body4.CenterOfMass=[0 0 0];
body4.Inertia=[1 1 1 0 0 0];
body5.Mass=1;
body5.CenterOfMass=[0 0 0];
body5.Inertia=[1 1 1 0 0 0];
%%
addBody(robot,body1,'base')
addBody(robot,body2,'body1')
addBody(robot,body3,'body2')
addBody(robot,body4,'body3')
addBody(robot,body5,'body4')

%%
% Verify that your robot was built properly by using the |showdetails| or
% |show| function. |showdetails| lists all the bodies in the
% MATLAB(R) command window. |show| displays the robot with a given
% configuration (home by default). Calls to |axis| modify the axis limits
% and hide the axis labels.
%%
%showdetails(robot);
q0=[pi/2 0 0 0 0];
q1=[pi/2 -pi/2 0 0 0];
qinit=[0 50 -30];
qInitial = homeConfiguration(robot); % Use home configuration as the initial guess
step=0:1:200;
%qs=zeros(size(step,2),length(qInitial));
points=zeros(size(step,2), 3);
for i=1:1:size(step,2)
    vx=10*sin(step(i));
    points(i,:)=qinit+[vx 0 0];
    %qinit=qs(i,:);
end
ik = robotics.InverseKinematics('RigidBodyTree', robot,'SolverAlgorithm','LevenbergMarquardt');
%default SolverAlgorithm BFGS shows wrong solution??????
gik= robotics.GeneralizedInverseKinematics;
weights = [0, 0, 0, 1, 1, 0];
endEffector = 'body5';
%%
for i = 1:size(step,2)
    % Solve for the configuration satisfying the desired end effector
    % position
    point = points(i,:);
    qSol = ik(endEffector,trvec2tform(point),weights,qInitial);
    % Store the configuration
    qs(i,:) = qSol;
    % Start from prior solution
    qInitial = qSol;
end
%%
%{
figure
show(robot,qs(1,:));
view(3)
ax = gca;
ax.Projection = 'orthographic';
hold on
%plot(points(:,1),points(:,2),'k')
%}
framesPerSecond = 150;
r = robotics.Rate(framesPerSecond);
for i = 1:size(step,2)
    show(robot,qs(i,:));%,'PreservePlot',false);
    drawnow
    waitfor(r);
end

%show(robot);

%axis([-50,50,-50,50,-50,50])
%axis on
