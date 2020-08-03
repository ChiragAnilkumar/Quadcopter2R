
% Declaring syms variables

syms x y z phi theta psi xdot ydot zdot psidot thetadot phidot  mb  mlnk l d g q1 q2 q1dot q2dot R T real


% Rotation matrix
R = [cos(psi)*cos(theta), cos(psi)*sin(theta)*sin(phi)-sin(psi)*cos(phi), cos(psi)*sin(theta)*cos(phi)+sin(psi)*sin(phi);
    sin(psi)*cos(theta), sin(psi)*sin(theta)*sin(phi)+cos(psi)*cos(phi), sin(psi)*sin(theta)*cos(phi)-cos(psi)*sin(phi);
    -sin(theta), cos(theta)*sin(phi), cos(theta)*cos(phi)];
% Transformation matrix
T = [0,-sin(psi),cos(psi)*cos(theta);
    0, cos(psi),sin(psi)*cos(theta);
    1, 0,-sin(theta)];

zeta = [x;y;z;psi;theta;phi;q1;q2];
zetadot = [xdot;ydot;zdot;psidot;thetadot;phidot;q1dot;q2dot];

P2 = [l*cos(q1)+l*cos(q1+q2);
      0;
     -(l*sin(q1)+l*sin(q1+q2))];
P1 = [l*cos(q1);0;-l*sin(q1)];
M1 = R*P1;
M2 = R*P2;

S1 =[0 -M1(3) M1(2) ; M1(3) 0 -M1(1) ; -M1(2) M1(1) 0 ];
S2 =[0 -M2(3) M2(2) ; M2(3) 0 -M2(1) ; -M2(2) M2(1) 0 ];

Jv1 = [-l*sin(q1), 0 ;0, 0;-l*cos(q1), 0];
Jv2 = [-l*sin(q1)-l*sin(q1+q2), -l*sin(q1+q2);0, 0;-(l*cos(q1)+l*cos(q1+q2)), -l*cos(q1+q2)];
Jw1 = [0, 0; 1, 0;0, 0];
Jw2 = [0, 0; 1, 1;0, 0];
R1 = [cos(q1), 0, sin(q1);0, 1, 0;-sin(q1), 0, cos(q1)];
R2 = [cos(q1+q2), 0, sin(q1+q2); 0, 1, 0;-sin(q1+q2), 0, cos(q1+q2)];

I3 = [1 0 0;0 1 0;0 0 1];
O33 = [0 0 0;0 0 0;0 0 0];
O32 = [0 0;0 0;0 0];
Mtb = ([I3 O33 O32]);
Mrb = [O33 T O32];
Mt1 = [I3 -(S1)*T R*Jv1];
Mt2 = [I3 -(S2)*T R*Jv2];
Mr1 = [O33 T R*Jw1];
Mr2 = [O33 T R*Jw2];

% Inertia matrix
M = Mtb'*mb*Mtb + Mrb'*R*I*R'*Mrb + Mt1'*m*Mt1 + Mr1'*(R*R1)*H*(R*R1)'*Mr1 ...
    +Mt2'*m*Mt2 + Mr2'*(R*R2)*H*(R*R2)'*Mr2;

% Declare an empty syms matrix    
c = [];
for i=1:8
     a=[];
     for j=1:8
         a=[a,sym(0)];
     end
     c=[c;a];
end

% Coriolis matrix
for i=1:8
     for j=1:8
         for k=1:8
             c(k,j)=c(k,j)+(1/2)*(diff(M(k,j),zeta(i))+diff(M(k,i),zeta(j))-diff(M(i,j),zeta(k)))*zetadot(i);
         end
     end
end 
C = simplify(c);

e3 =[0 0 1]';
p=[x y z]';
% Potential Energy
U = mb*g*e3'*p+ g*(m*(e3')*(p+R*P1))+g*(m*(e3')*(p+R*P2));
% Gravity matrix
for i=1:8
    G(i,1)=diff(U,zeta(i));
end