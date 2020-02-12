function [sensPosWRTlink] = printSensorPosition(suit)
%PRINTSENSORPOSITION prints a structure with the estimated position of the
% sensors w.r.t. the attached link frame.
%
% Author(s): Claudia Latella, 2017
% Dynamic Interaction Control, Istituto Italiano di Tecnologia

sensPosWRTlink = struct;
    for i = 1 : size(suit.sensors,1)
        sensPosWRTlink(i).attachedLink = suit.sensors{i, 1}.attachedLink;
        sensPosWRTlink(i).pos_x = suit.sensors{i, 1}.position(1);
        sensPosWRTlink(i).pos_y = suit.sensors{i, 1}.position(2);
        sensPosWRTlink(i).pos_z = suit.sensors{i, 1}.position(3);
    end

end
