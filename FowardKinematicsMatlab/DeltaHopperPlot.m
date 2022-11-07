clear;
clf;
% using radians.

% mech configuration
L0 = 0.10294; % length between hip motors
L1 = 0.20825;  % base to knee
L2 = 0.360;  % knee to ankle
a0 = 0.02596; % length between ankle joints
% F  = 0.5;
baseHeight = 0;
footLength = 0.03;

% joint configuration
theta0 = 0; 
theta1 = pi/3;
theta2 = 0;

%% hip motor

%   Base0 (0,L/sqrt(3))
%     /\
%    /  \
%   /----\
% Base 1(-L/2,-L/(2*sqrt3))       
%           Base 2:(L/2,-L/(2*sqrt3))   
%  ↑positive Y     ->  positive X

hip0 = [0,L0/sqrt(3),baseHeight];
hip1 = [-L0/2,-L0/(2*sqrt(3)),baseHeight];
hip2 = [L0/2,-L0/(2*sqrt(3)),baseHeight];



%% knee motor 

% Origin------------- Hip0 -----
%                   /  )theta
%                  /

knee0 = hip0 + [0,L1*cos(theta0),-L1*sin(theta0)];
knee1 = hip1 + [-L1*cos(theta1)*sqrt(3)/2,-L1*cos(theta1)/2,-L1*sin(theta1)];
knee2 = hip2 + [L1*cos(theta2)*sqrt(3)/2,-L1*cos(theta2)/2,-L1*sin(theta2)];

%% ankle
% eqns

F = @(x)[
    (x(1)-knee0(1))^2 + (x(2)-knee0(2))^2 + (x(3)-knee0(3))^2 - L2^2,...
    (x(4)-knee1(1))^2 + (x(5)-knee1(2))^2 + (x(6)-knee1(3))^2 - L2^2,...
    (x(7)-knee2(1))^2 + (x(8)-knee2(2))^2 + (x(9)-knee2(3))^2 - L2^2,...
    (x(1) - x(4))^2 + (x(2) - x(5))^2 + (x(3) - x(6))^2 - a0^2,...
    (x(1) - x(7))^2 + (x(2) - x(8))^2 + (x(3) - x(9))^2 - a0^2,...
    (x(7) - x(4))^2 + (x(8) - x(5))^2 + (x(9) - x(6))^2 - a0^2,...
    dot(knee0-[x(1) x(2) x(3)],[x(4)-x(7) x(5)-x(8) x(6)-x(9)]),...
    dot(knee1-[x(4) x(5) x(6)],[x(1)-x(7) x(2)-x(8) x(3)-x(9)]),...
    dot(knee2-[x(7) x(8) x(9)],[x(4)-x(1) x(5)-x(2) x(6)-x(3)])
    
    
    % dot(knee0-[x(1) x(2) x(3)],[x(4)-x(7) x(5)-x(8) x(6)-x(9)]),...
    % dot(knee1-[x(4) x(5) x(6)],[x(1)-x(7) x(2)-x(8) x(3)-x(9)]),...
    % dot(knee2-[x(7) x(8) x(9)],[x(4)-x(1) x(5)-x(2) x(6)-x(3)])
    ];

x = fsolve(F,[0,0,-2,0,0,-2,0,0,-2]); % using this initial condition because the ankles should always be below the hips.



ankle0 = x(:,1:3);
ankle1 = x(:,4:6);
ankle2 = x(:,7:9);

%% foot
footCenter = (ankle0+ankle1+ankle2)/3;
footdir = cross((ankle0-ankle1),(ankle2-ankle1));
footdir = footdir/norm(footdir);
if(norm(footCenter+footdir)<norm(footCenter))
    footdir = -footdir;
end
foot = footCenter + footdir * footLength;

%% plot

% hip motors plate
a = [hip0;hip1;hip2;hip0];
plot3(a(:,1),a(:,2),a(:,3))
hold on

% convert to link
BaseLink0 = [hip0;knee0];
plot3(BaseLink0(:,1),BaseLink0(:,2),BaseLink0(:,3))
hold on;

BaseLink1 = [hip1;knee1];
plot3(BaseLink1(:,1),BaseLink1(:,2),BaseLink1(:,3))
hold on;

BaseLink2 = [hip2;knee2];
plot3(BaseLink2(:,1),BaseLink2(:,2),BaseLink2(:,3))
hold on

LowLink0 = [knee0;ankle0];
plot3(LowLink0(:,1),LowLink0(:,2),LowLink0(:,3))
hold on;

LowLink1 = [knee1;ankle1];
plot3(LowLink1(:,1),LowLink1(:,2),LowLink1(:,3))
hold on;

LowLink2 = [knee2;ankle2];
plot3(LowLink2(:,1),LowLink2(:,2),LowLink2(:,3))
hold on;

a = [ankle0;ankle1;ankle2;ankle0];
plot3(a(:,1),a(:,2),a(:,3));
hold on

footLink = [footCenter;foot];
plot3(footLink(:,1),footLink(:,2),footLink(:,3),'-o');

axis equal
