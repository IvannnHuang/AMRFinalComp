function enc = readEncoderData(Robot)
% readEncoderData reads left and right wheel encoder ticks from the robot.
% OUTPUT: enc = [leftTicks, rightTicks]

    try
        enc = EncoderSensorRoomba(Robot); % returns [leftTicks, rightTicks]
    catch
        warning('Failed to read encoder data. Returning zeros.');
        enc = [0, 0];
    end
end