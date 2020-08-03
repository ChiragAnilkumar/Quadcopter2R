
% Desired set of generalized coordinates
% Pick Up
zeta_des =[0;0;0;0;0;0;pi/2;0]; 
% Move
% zeta_des =[1;0;1;0;0;0;pi/2;0];
%Drop
% zeta_des =[1;0;1;0;0;0;pi/2;-pi/4];


% tspan =[0 2];

% Pick Up
IC = [0 ;0 ;0 ;0 ;0 ;0 ;0; 0; 0; 0 ;0 ;0; pi/2; -pi/4; 0; 0];
% Move
% IC = [0 ;0 ;0 ;0 ;0 ;0 ;0; 0; 0; 0 ;0 ;0; pi/2; 0; 0; 0];
%Drop
% IC = [1 ;0 ;1 ;0 ;0 ;0 ;0; 0; 0; 0 ;0 ;0; pi/2; 0; 0; 0];

options = odeset('RelTol',1e-3,'AbsTol',1e-4);
[T, S] = ode45(@eom_quad,tspan,IC,options);

% Generalized system of coordinates
x =S(:,1);
y =S(:,2);
z =S(:,3);
psi=S(:,7);
theta=S(:,8);
phi = S(:,9);
q1 = S(:,13);
q2 = S(:,14);

% Qyadcopter parameters
l = 0.1;
g = 9.81;
d = 0.1;

% Plotting of values of generalized system of coordinates vs desired values

% subplot(4,2,1);
% plot(T,zeta_des(1),'r.');
% hold on
% plot(T,x);
% xlabel('t');
% ylabel('x','Rotation',0);
% subplot(4,2,2);
% plot(T,zeta_des(2),'r.');
% hold on
% plot(T,y);
% xlabel('t');
% ylabel('y','Rotation',0);
% subplot(4,2,3);
% ylim([-1 1]);
% plot(T,zeta_des(3),'r.');
% hold on
% plot(T,z);
% xlabel('t');
% ylabel('z','Rotation',0);
% subplot(4,2,4);
% plot(T,zeta_des(4),'r.');
% hold on
% plot(T,psi);
% xlabel('t');
% ylabel('\psi','Rotation',0);
% subplot(4,2,5);
% plot(T,zeta_des(5),'r.');
% hold on
% plot(T,theta);
% xlabel('t');
% ylabel('\theta','Rotation',0);
% subplot(4,2,6);
% ylim([-1 1]);
% plot(T,zeta_des(6),'r.');
% hold on
% plot(T,phi);
% xlabel('t');
% ylabel('\phi','Rotation',0);
% subplot(4,2,7);
% plot(T,zeta_des(7),'r.');
% hold on
% plot(T,q1);
% xlabel('t');
% ylabel('q_1','Rotation',0);
% subplot(4,2,8);
% plot(T,zeta_des(8),'r.');
% hold on
% plot(T,q2);
% xlabel('t');
% ylabel('q_2','Rotation',0);


% Animation

for i=1:length(T)

    % Plotting of quadcoter center of mass
    plot3(x(i),y(i),z(i),'ko');
    hold on
    grid on
   axis equal
    
    % Roation matrix
Rb = [cos(psi(i))*cos(theta(i)), cos(psi(i))*sin(theta(i))*sin(phi(i))-sin(psi(i))*cos(phi(i)), cos(psi(i))*sin(theta(i))*cos(phi(i))+sin(psi(i))*sin(phi(i));
    sin(psi(i))*cos(theta(i)), sin(psi(i))*sin(theta(i))*sin(phi(i))+cos(psi(i))*cos(phi(i)), sin(psi(i))*sin(theta(i))*cos(phi(i))-cos(psi(i))*sin(phi(i));
    -sin(theta(i)), cos(theta(i))*sin(phi(i)), cos(theta(i))*cos(phi(i))];

    % Positions of quadcopter arms 
    P1  = [x(i);y(i);z(i)] + Rb*[d;0;0];
    P2 = [x(i);y(i);z(i)] + Rb*[-d; 0;0];
    P3 = [x(i);y(i);z(i)] + Rb*[0; d; 0];
    P4 = [x(i);y(i);z(i)] + Rb*[0; -d; 0];
    % Positions of manipulator arms 
    L1 = [x(i);y(i);z(i)] +  Rb*[l*cos(q1(i));0;-l*sin(q1(i))];
    L2 = [x(i);y(i);z(i)] +  Rb*[l*cos(q1(i))+l*cos(q1(i)+q2(i));0;-l*sin(q1(i))-l*sin(q1(i)+q2(i))];
    
   % Plotting of quadcopter arms
    plot3([x(i) P1(1)],[y(i) P1(2)],[z(i) P1(3)],'.-', 'MarkerSize', 20, 'LineWidth', 2,'Color','b');
    plot3([x(i) P2(1)],[y(i) P2(2)],[z(i) P2(3)],'.-', 'MarkerSize', 20, 'LineWidth', 2,'Color','b');
    plot3([x(i) P3(1)],[y(i) P3(2)],[z(i) P3(3)],'.-', 'MarkerSize', 20, 'LineWidth', 2,'Color','b');
    plot3([x(i) P4(1)],[y(i) P4(2)],[z(i) P4(3)],'.-', 'MarkerSize', 20, 'LineWidth', 2,'Color','b');
    
    % Plotting of manipulator links
    plot3([x(i) L1(1) L2(1)],[y(i) L1(2) L2(2)],[z(i) L1(3) L2(3)],'.-', 'MarkerSize', 20, 'LineWidth', 2,'Color','r');
    
    % Pickup of object
    plot3(0,0,-0.2,'ko'); 
    % Movement of object
%   plot3(L2(1),L2(2),L2(3),'ko');
%    plot3(1,0,0.8,'b*'); % Desired final position
    % Drop of object
%     plot3(1,0,0.8,'ko'); 
    
    hold off
    title('Quadcopter Plot');
    drawnow
end

