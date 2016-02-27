function [omega,position]=slewProfiler(slewDuration,maxOmega,maxAcceleration,initialPosition,initialOmega,finalOmega,finalPosition)
%This function generates a 1-D slew based on desired duration, maximum
%rate and acceleration, and initial and final rate.

decelerationTime=abs((finalOmega-initialOmega)/maxAcceleration);
remainingTime=abs((finalOmega+initialOmega)/(2*(finalPosition-initialPosition)));

%First period of slew: acclerate while angular rate is under maximum
if initialOmega<maxOmega && decelerationTime<remainingTime
    omega=initialOmega+maxAcceleration*slewDuration;
    position=initialPosition+slewDuration*initialOmega;
    
%Third period of slew: If time to end of 
%maneuver is just enough to slow to final angular rate, begin
%decelerating.
elseif decelerationTime>=remainingTime
    omega=initialOmega-maxAcceleration*slewDuration;
    position=initialPosition+slewDuration*initialOmega;
    
%Second period of slew: coast at maximum angular rate.
else
    omega=maxOmega;
    position=initialPosition+slewDuration*initialOmega;
end


end

