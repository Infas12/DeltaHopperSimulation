function J = HalfJacobian(theta0,theta1,theta2)
    
    % Only contains the position 
    e = 10^-8;
    A = DeltaFK([theta0,theta1,theta2]);
    delta0 = DeltaFK([theta0+e,theta1,theta2]);
    delta1 = DeltaFK([theta0,theta1+e,theta2]);
    delta2 = DeltaFK([theta0,theta1,theta2+e]);
    [delta0,delta1,delta2];
    J = ([delta0,delta1,delta2]-[A,A,A])/e;
    
end