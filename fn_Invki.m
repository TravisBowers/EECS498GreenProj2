%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% EECS 498: Hands-On Robotics
% Project 2 - Robotic Manipulator
% Kurt Lundeen, Bharadwaj Mantha
% 2014-11-25
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function th_d=fn_Invki(p)
%close all;clear all;clc;
syms th1 th2 th3 th4 th5 L1 L2 L3 L4
th = [th1;th2;th3;th4;th5];
numJoints = size(th,1);
spaDim = 3;
gst0 = [eye(spaDim) [0;L2+L3+L4;L1];zeros(1,spaDim) 1];

w{1} = [0;0;1];
w{2} = [-1;0;0];
w{3} = w{2};
w{4} = [0;1;0];
w{5} = w{2};

q = cell(1,numJoints);
q{1} = [0;0;L1];
q{2} = q{1};
q{3} = [0;L2;L1];
q{4} = [0;L2+L3;L1];
q{5} = q{4};

expwth = cell(1,numJoints);
expwth{1} = [cos(th1) -sin(th1) 0
             sin(th1)  cos(th1) 0
                    0         0 1];
expwth{2} = [1         0        0
             0  cos(th2) sin(th2)
             0 -sin(th2) cos(th2)];
expwth{3} = [1         0        0
             0  cos(th3) sin(th3)
             0 -sin(th3) cos(th3)];
expwth{4} = [ cos(th4) 0 sin(th4)
                     0 1        0
             -sin(th4) 0 cos(th4)];
expwth{5} = [1         0        0
             0  cos(th5) sin(th5)
             0 -sin(th5) cos(th5)];          

v = cell(1,numJoints);
xi = cell(1,numJoints);
for i = 1:numJoints
    v{i} = cross(-w{i},q{i});
    xi{i} = [v{i};w{i}];
    expxith{i} = [expwth{i} (eye(spaDim)-expwth{i})*cross(w{i},v{i})+w{i}*transpose(w{i})*v{i}*th(i);zeros(1,spaDim) 1];
end

gst = simplify(expxith{1}*expxith{2}*expxith{3}*expxith{4}*expxith{5}*gst0);

% Inverse kinematics
Tr = gst(1:3,4); % the translational matrix of the end effector
% %partial derivaties with respective to each joint rotation th
j1=diff(Tr,th1); %w.r.t th1
j2=diff(Tr,th2); %w.r.t th2
j3=diff(Tr,th3); %w.r.t th3
j4=diff(Tr,th4); %w.r.t th4
j5=diff(Tr,th5); %w.r.t th5
Jac=[j1 j2 j3 j4 j5]; % generalized Jacobian matrix

th1=0;th2=-pi/4;th3=pi/2;th4=0;th5=pi/2;
L1=150;L2=240;L3=300;L4=80; % beginning point (current location of the robot)
subs(Tr); %initial coordinates of the robot arm
% Jac=subs(Jac); % jacobian matrix 
% Jac_inv=pinv(Jac); % inverse jacobian matrix
x=2;
a=0.001;
while x>0.2
l_d=p;%[0;400;35]; %end point (desired location)
l_c=subs(Tr);% current coordinates of the robot arm 
e=l_d-l_c; %error b/w the desired and actual position
Jac=subs(Jac); % jacobian matrix 
Jac_inv=pinv(Jac); % inverse jacobian matrix
th_d=[th1;th2;th3;th4;th5]+Jac_inv*e+a*randn(5,1); %desired joint angles
th1=th_d(1);th2=th_d(2);th3=th_d(3);th4=th_d(4);th5=th_d(5);
subs(Tr); %to check for intermediate coordinate points
x=norm(l_d-l_c); % calculating the norm for running the loop
end 
%subs(Tr)
end 


