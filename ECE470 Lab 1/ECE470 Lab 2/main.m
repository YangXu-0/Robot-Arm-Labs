% file to run the robot simulations

close; clear;

% format for DH inputs: [theta_i d_i a_i alpha_i]
DH = [0 76 0 pi/2; 0 -23.65 43.23 0; 0 0 0 pi/2; 0 43.18 0 -pi/2; 0 0 0 pi/2; 0 20 0 0];

% as stated in lab manual
theta_1 = linspace(0,pi,200);
theta_2 = linspace(0,pi/2,200);
theta_3 = linspace(0,pi,200);
theta_4 = linspace(0,pi/4,200);
theta_5 = linspace(-pi/3,pi/3,200);
theta_6 = linspace(0,2*pi,200);

% create robot structure
myrobot = mypuma560(DH);

%% 4.2: Plot a sample joint space trajectory

q = [theta_1; theta_2; theta_3; theta_4; theta_5; theta_6]';

%plot(myrobot, q)

%% 4.3: Forward Kinematics

% to trace the end effector origin path
o = zeros(200,3);

for i = 1:200
    % H_06 for each timestep
    H = forward(q(i,:), myrobot);
    o(i,:) = H(1:3, 4);
end

%plot3(o(:,1),o(:,2),o(:,3),'r')
%hold on;
%plot(myrobot,q)

%% 4.4: Inverse Kinematics

%H = [cos(pi/4) -sin(pi/4) 0 20; sin(pi/4) cos(pi/4) 0 23; 0 0 1 15; 0 0 0 1];
%q = inverse(H, myrobot)

% pick up and put object down----------------------------
o_x = linspace(10,30, 100);
o_y = linspace(23, 30, 100);
o_z = linspace(15, 100, 100);

% defining R_z,pi/4 as specified
R = [cos(pi/4) -sin(pi/4) 0; sin(pi/4) cos(pi/4) 0; 0 0 1];

q = zeros(100,6);

H = zeros(4,4);
H(4,4) = 1;

for i = 1:100
    d = [o_x(i); o_y(i); o_z(i)];
    H(1:3,1:3) = R;
    H(1:3, 4) = d;
    q(i,:) = inverse(H, myrobot);
end

plot3(o_x,o_y,o_z,'r')
hold on;
plot(myrobot,q)