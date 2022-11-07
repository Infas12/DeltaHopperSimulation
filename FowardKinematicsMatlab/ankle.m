function foot = ankle(knee_in,L2,F)
footLength = 0.05;

knee0 = knee_in(:,1:3);
knee1 = knee_in(:,4:6);
knee2 = knee_in(:,7:9);

%% ankle

F = @(x)[
    (x(1)-knee0(1))^2 + (x(2)-knee0(2))^2 + (x(3)-knee0(3))^2 - L2^2,...
    (x(4)-knee1(1))^2 + (x(5)-knee1(2))^2 + (x(6)-knee1(3))^2 - L2^2,...
    (x(7)-knee2(1))^2 + (x(8)-knee1(2))^2 + (x(9)-knee1(3))^2 - L2^2,...
    (x(1) - x(4))^2 + (x(2) - x(5))^2 + (x(3) - x(6))^2 - F^2,...
    (x(1) - x(7))^2 + (x(2) - x(8))^2 + (x(3) - x(9))^2 - F^2,...
    (x(7) - x(4))^2 + (x(8) - x(5))^2 + (x(9) - x(6))^2 - F^2,...
    dot(knee0-[x(1) x(2) x(3)],[x(4)-x(7) x(5)-x(8) x(6)-x(9)]),...
    dot(knee1-[x(4) x(5) x(6)],[x(1)-x(7) x(2)-x(8) x(3)-x(9)]),...
    dot(knee2-[x(7) x(8) x(9)],[x(4)-x(1) x(5)-x(2) x(6)-x(3)])
    ];

x = fsolve(F,zeros(1,9)-2); % using this initial condition because the ankles should always be below the hips.

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

foot = foot';

end


