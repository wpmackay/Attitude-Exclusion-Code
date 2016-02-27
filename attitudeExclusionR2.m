function [q_modified,omega_modified,omega_dot_modified]=attitudeExclusionR2(q,omega,omega_dot,R,V,r,v)
%Attitude Exclusion Algorithm

rho = r - R;
rho_dot = v - V; % In inertial frame

%Defining the body frame coordinate system for the satellite
i_z = rho/norm(rho); % Line of Sight Direction (cameras)
i_w = omega/norm(omega); % Direction of LOS Angular rate
i_y = cross(i_z,i_w); % Third Triad Direction (solar panels)
A_targeter=q2a(q);


%Step 415-Determine axis of rotation between commanded boresight vector
%and sun vector


%Step 420- Calculate commanded cross-boresight angluar rate


%Step 425- Calculate y direction
%Step 430- Prevent direction from reversing
if 
end

%Step 435- Define maximum delta y for time step and prevent rotation by 
%more than allowed amount

    
%Step 440- Calculate current target tracking pointing y coordinate Y0

% %Step 445- If Y0 is greater than exclusion zone radius, no modification is
% %necessary.
% if Y0<r
%     %Step 452- if the target-tracking pointing is outside of the exclusion 
%     %zone, and is moving away from the exclusion zone, and an avoidance 
%     %maneuver is not currently underway , no modification. If not, go to
%     %step 454.
%     sunBoresightAngle=acos(dot(boresightVec,sunVec_sunSensor)/(norm(sunVec_sunSensor)*norm(boresightVec)));
%     if  sunBoresightAngle>r && sunBoresightAngle>sunBoresightAnglePrev && dot(Ydirection,YdirectionPrev)<0
%         %Step 450- No Modification Necessary
%         disp('No modification necessary')
%         q_modified=0;
%         Yrate=0;
%     else
%         %Step 454- If the target-tracking pointing is outside of the
%         %exclusion zone and is moving away from the exclusion zone, then
%         %got to step 458. If not,go to 462.
%         if sunBoresightAngle>r && sunBoresightAngle>sunBoresightAnglePrev
%             %Step 458/460/466- Apply first step of 1-D slew profiler to return 
%             %boresight to target tracking as quickly as possible. Apply 3rd
%             %order low-pass filer to modified Y command, use Y rate from 
%             %slew profiler.
%             [Yrate,Ycalc]=slewProfiler(tStep,omegaMax,maxAcceleration,Yprev1,YratePrev,0,0);
%             Yfiltered=filter(b,a,[Yprev2,Yprev1,Ycalc]);
%             q_modified=Yfiltered(3);
%         else
%             %Step 462/460/466-the target-tracking pointing is currently either in 
%             %or headed toward the exclusion zone, and the algorithm 
%             %calculates the desired Y offset, Ycalc. Ycalc is filtered
%             %using a 3rd order low-pass filter.
%             
%             %Constants that define shape of exclusion zone, will need to be
%             %tweaked.
%             k2=1; k3=1; k4=1;
%             
%             X0=arccos(dot(iHat, boresightVec));
%             A=r-Y0;
%             n_omega=(omegaMax-omegaCross)/omegaMax;
%             if n_omega<0
%                 n_omega=0;
%             end
%             n_d=k2*(1+(n_omega*((exp(-X0/(k3*r))^k4)-1)));
%             n_Y0=Y0/r;
%             d=n_d*(A/((1-n_Y0)^0.5));
%             Ycalc=A*exp((-X0/d)^2);
%             Yfiltered=filter(b,a,[Yprev2,Yprev1,Ycalc]);
%             q_modified=Yfiltered(3);
%             
%             %Step 472- Using X0 to get new Y rate command.
%             if dot(Ydirection,Yprev1)<0
%                 X0_next=X0+omegaCross*tStep;
%             else
%                 X0_next=X0-omegaCross*tStep;
%             end
%             %Estimating Ymodified for next step to get Y rate
%             n_d_next=k2*(1+(n_omega*((exp(-X0_next/(k3*r))^k4)-1)));
%             d_next=n_d_next*(A/((1-n_Y0)^0.5));
%             Ycalc_next=A*exp((-X0_next/d_next)^2);
%             Yrate=(Ycalc_next-Ycalc)/tStep;
%         end
%     end
%     
% else
%     %Step 450- No Modification Necessary
%     disp('No modification necessary');
%     q_modified=0;
%     Yrate=0;
% end

end
