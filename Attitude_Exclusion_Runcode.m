%Attitude Exclusion Algorithm Runcode

clear; close all; clc;

%%

%Defining function inputs:
boresightVec=[.01,.0,.80];
sunVec_gps=[.557,.743,.371];
sunVec_sunSensor=[.553,.753,.361];
omegaBody=[0.2,0,0];
Yprev1=.0;
Yprev2=.0;
YratePrev=.1;
sunBoresightAnglePrev=0.50;
YdirectionPrev=[0,1,0];

[Ymodified,Yrate]=attitudeExclusion(boresightVec,omegaBody,sunVec_sunSensor,YdirectionPrev, Yprev1,Yprev2,YratePrev,sunBoresightAnglePrev)
