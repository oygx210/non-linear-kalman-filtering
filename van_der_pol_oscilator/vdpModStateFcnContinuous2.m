function dxdt = vdpModStateFcnContinuous2(t, x)
%vdpStateFcnContinuous Evaluate the van der Pol ODEs for mu = 1
dxdt = [x(2); (1-x(1)^2)*x(2)-sin(x(1))];
end
