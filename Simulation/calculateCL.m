function CL = calculateCL(AOA)
% can depend on your airfoil
CL_alpha = 0.1; % Joukousky profile

CL = CL_alpha * AOA;

end