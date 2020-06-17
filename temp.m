clc;
close all;
clear;

s = tf('s');


a = 10;
a2 = 12;

Ls = (s+a)*(s+a2)/(s*(s+2)*(s+4))

H = 1/(s*(s+2)*(s+4));


figure;
rlocus(Ls);

