%Attitude Exclusion Algorithm Runcode

clear; close all; clc;

%%

%Defining function inputs:
boresightVec=[.01,.0,.80];
sunVec
omega=[0.2,0,0];
Ym_prev
sunBoresightAnglePrev=0.50;
YdirectionPrev=[0,1,0];

[q_modified,omega_modified,omega_dot_modified]=attitudeExclusionR2(q,omega,omega_dot,R,V,r,v,sunVec,YdirectionPrev,Ym_prev)
