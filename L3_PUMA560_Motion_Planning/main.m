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


%%%%%% Section 3.1 %%%%%%%
H1 = eul2tr([0 pi pi/2]);
H1(1:3,4) = 100*[-1; 3; 3;]/4;
q1 = inverse(H1,myrobot);
H2 = eul2tr([0 pi -pi/2]);
H2(1:3,4) = 100*[3;-1;2]/4;
q2 = inverse(H2,myrobot);
tau = att(q1,q2,myrobot)


%%%%%% Section 3.2 %%%%%%%
qref = motionplan(q1,q2,0,10,myrobot,[],0.01);
t=linspace(0,10,300);
q = ppval(qref,t)';
plot(myrobot,q)


%%%%%% Section 3.3 %%%%%%%
setupobstacle
q3 = 0.9*q1+0.1*q2;
tau = rep(q3,myrobot,obs{1})

q = [pi/2 pi 1.2*pi 0 0 0];
tau = rep(q,myrobot,obs{6})

setupobstacle
hold on
axis([-100 100 -100 100 0 200])
view(-32,50)
plotobstacle(obs);
qref = motionplan(q1,q2,0,10,myrobot,obs,0.01);
t=linspace(0,10,300);
q=ppval(qref,t)';
plot(myrobot,q);
hold off