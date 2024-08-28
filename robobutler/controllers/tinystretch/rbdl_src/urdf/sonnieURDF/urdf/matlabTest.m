clear variables;
close all;
robot=importrobot('SonnyV4_wholebody.urdf');
config=homeConfiguration(robot);
J=geometricJacobian(robot,config,'toe_right');
disp(J(4:6,:))
