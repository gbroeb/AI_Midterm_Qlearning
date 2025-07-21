function [robot_t]= motion_model(robot_t_1,a,dt)

VW.v=3*5;
degrees = [-30, -15, 0, 15, 30];
radians = degrees * (pi / 180);
%if a==3
%    VW.w=0;
%end
VW.w = radians(a);                                      


%%motion model
robot_t.x=robot_t_1.x+VW.v*cos(robot_t_1.t)*dt;
robot_t.y=robot_t_1.y+VW.v*sin(robot_t_1.t)*dt;
robot_t.t=robot_t_1.t+VW.w*dt;
t = 1 ;
end