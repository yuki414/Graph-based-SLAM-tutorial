function [new_angle] = pi2pi(angle)
new_angle = rem(angle+pi,2*pi)-pi;