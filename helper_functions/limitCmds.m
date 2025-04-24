function[cmdV,cmdW] = limitCmds(fwdVel,angVel,maxV,wheel2Center)
% LIMITCMDS: scale forward and angular velocity commands to avoid
% saturating motors.
% 
%   [CMDV,CMDW] = LIMITCMDS(FWDVEL,ANGVEL,MAXV,WHEEL2CENTER) returns the 
%   forward and angular velocity commands to drive a differential-drive 
%   robot whose wheel motors saturate at +/- maxV.
% 
%   INPUTS
%       fwdVel      desired forward velocity (m/s)
%       angVel      desired angular velocity (rad/s)
%       maxV        maximum motor velocity (assumes symmetric saturation)
%       wheel2Center distance from the wheels to the center of the robot(in meters)
% 
%   OUTPUTS
%       cmdV        scaled forward velocity command (m/s)
%       cmdW        scaled angular velocity command (rad/s)
% 
% 
%   Cornell University
%   Autonomous Mobile Robots
%   Homework #1
%   Huang, Ivan

cmdV = fwdVel; cmdW = angVel;
vr = fwdVel + angVel * wheel2Center;
vl = fwdVel - angVel * wheel2Center;
if vr > maxV || vr < -maxV || vl > maxV || vl < -maxV
    ratio = min(abs(maxV / vr), abs(maxV / vl));
    vr = vr * ratio;
    vl = vl * ratio;
    cmdW = angVel * ratio;
    cmdV = 0.5 * (vr + vl);
end
