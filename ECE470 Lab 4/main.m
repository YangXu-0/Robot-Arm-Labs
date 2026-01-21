% Main file: Prelab, plots, and KUKA commands

close; clearvars -except udpObj;

% Lab 4 Prelab:
d1 = 400;
a1 = 25;
a2 = 315;
a3 = 35;
d4 = 365;
a6 = 156; %a6=156 for inv kin, =0 for forces
d6 = 161.44;

% format for DH inputs: [theta_i d_i a_i alpha_i]
DH = [0 d1 a1 pi/2; 
      0 0 a2 0; 
      0 0 a3 pi/2; 
      0 d4 0 -pi/2; 
      0 0 0 pi/2; 
      0 d6 -a6 0];

kuka = mykuka(DH);

DH_forces = DH;
DH_forces(6,3) = 0;
kuka_forces = mykuka(DH_forces);

% rep function test:
setupobstacle_lab4prep;
tau = rep([pi/10,pi/12,pi/6,pi/2,pi/2,-pi/6],kuka_forces,prepobs{1})

% motion planning test:
% NOTE:
%   Setting alpha_rep = 0.013 makes the motion
%   planning work much better.

%   Decreasing rho0 appears to make the robot links
%   stay closer to the obstacle (cylinder).

p1 = [620 375 50];
p2 = [620 -375 50];
R  = [0 0 1; 0 -1 0; 1 0 0];
H1 = [R p1'; zeros(1,3) 1];
H2 = [R p2'; zeros(1,3) 1];
q1 = inverse(H1, kuka);
q2 = inverse(H2, kuka);

hold on
axis([-100 100 -100 100 0 100])
view(-32,50)
plotobstacle(prepobs);
qref = motionplan(q1, q2, 0, 10, kuka_forces, prepobs, 0.01)
t=linspace(0,10,300);
q=ppval(qref,t)';
plot(kuka,q);
hold off

%% Lab 4
%%%%%%%% Part 4.1 %%%%%%%%%
% setting up plane and 2 cylinders
setupobstacle;

%slightly increased to correctly grab cube
z_grid = 50;

p0 = [370 -440 150];
p1 = [370 -440 z_grid];
p2 = [750 -220 225];
p3 = [620 350 225];

R  = [0 0 1; 0 -1 0; 1 0 0];

% p0->p1
H0 = [R p0'; zeros(1,3) 1];
H1 = [R p1'; zeros(1,3) 1];
q0 = inverse(H0, kuka);
q1 = inverse(H1, kuka);

% p2->p3
H2 = [R p2'; zeros(1,3) 1];
H3 = [R p3'; zeros(1,3) 1];
q2 = inverse(H2, kuka);
q3 = inverse(H3, kuka);

%plot for motion planned
hold on
axis([-1000 1000 -1000 1000 0 1000])
view(-32,50)
plotobstacle(obs);
qref2 = motionplan(q1, q2, 0, 10, kuka_forces, obs, 0.01);
qref3 = motionplan(q2, q3, 0, 10, kuka_forces, obs, 0.01);
t=linspace(0,10,100);
q_pp2=ppval(qref2,t)';
q_pp3=ppval(qref3,t)';
plot(kuka,q_pp2);
hold on;
plot(kuka,q_pp3);
hold off

%%
%%%%%%%% Part 4.2 %%%%%%%%%
%KUKA commands for motion planned
setGripper(0)
setAngles(q0, 0.03);
setAngles(q1, 0.03);
setGripper(1)

for i = 1:length(q_pp2)
    setAngles(q_pp2(i, 1:6), 0.03);
end

for i = 1:length(q_pp3)
    setAngles(q_pp3(i, 1:6), 0.03);
end

setGripper(0)

%%%%%%%% Part 4.3 %%%%%%%%%
% dropping block into second cylinder

z_grid = 50;

p0 = [370 -440 150];
p1 = [370 -440 z_grid];
p2 = [750 -220 225];
% changing p3 to centre and above cyl2
p3 = [620 5 450];

R  = [0 0 1; 0 -1 0; 1 0 0];

% p0->p1
H0 = [R p0'; zeros(1,3) 1];
H1 = [R p1'; zeros(1,3) 1];
q0 = inverse(H0, kuka);
q1 = inverse(H1, kuka);

% p2->p3
H2 = [R p2'; zeros(1,3) 1];
H3 = [R p3'; zeros(1,3) 1];
q2 = inverse(H2, kuka);
q3 = inverse(H3, kuka);

%plot for motion planned
hold on
axis([-1000 1000 -1000 1000 0 1000])
view(-32,50)
plotobstacle(obs);
qref2 = motionplan(q1, q2, 0, 10, kuka_forces, obs, 0.01);
qref3 = motionplan(q2, q3, 0, 10, kuka_forces, obs, 0.01);
t=linspace(0,10,300);
q_pp2=ppval(qref2,t)';
q_pp3=ppval(qref3,t)';
plot(kuka,q_pp2);
hold on;
plot(kuka,q_pp3);
hold off

%KUKA commands for motion planned
setGripper(0)
setAngles(q0, 0.03);
setAngles(q1, 0.03);
setGripper(1)

for i = 1:length(q_pp2)
    setAngles(q_pp2(i, 1:6), 0.03);
end

for i = 1:length(q_pp3)
    setAngles(q_pp3(i, 1:6), 0.03);
end

setGripper(0)
