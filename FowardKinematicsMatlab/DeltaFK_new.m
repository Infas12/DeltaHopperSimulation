function footPos = DeltaFK_new(L0,L1,L2,a0,theta0,theta1,theta2)

% mech configuration
% F  = 0.5;
baseHeight = 0;
footLength = 0.03;


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

% ------------- Hip0 -----
%              /  )theta
%            /

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
opt=optimset('Display','off');
x = fsolve(F,[0,0,-2,0,0,-2,0,0,-2],opt); % using this initial condition because the ankles should always be below the hips.



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
footPos = footCenter + footdir * footLength;
footPos = footPos';
end