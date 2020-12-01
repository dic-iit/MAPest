
% Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT)
% All rights reserved.
%
% This software may be modified and distributed under the terms of the
% GNU Lesser General Public License v2.1 or any later version.

function [shoes] = transformShoesWrenches_iWear(orientationInQuat, wearData, height_heelFoot, mass)
% TRANSFORMSHOESWRENCHES transforms external wrenches coming
% from the ftShoes into human frames.
% Shoes wrenches are estimated in their frames (origin and
% orientation) that are located at a known position (in [ftShoe] frame)
% For the human estimation we need to get from this wrenches but:
% - multiplied by -1 (as the wrench applied on the human is exactly the
%   opposite of the one excerted on each shoe)
% - expressed in the frame of the human link in contact.
%
% This function computes the wrenches that the each shoe exerts on the link
% in contact.  See the sketch footInShoe.pdf for the reference frames
% locations.

samples = size(orientationInQuat.leftFootOrientation,2);
ftShoeSeenFromHeel = [0.037; 0 ; -0.050]; %FtShoe ref frame
heelSeenFromFoot   = [-0.0383; 0 ; -height_heelFoot];

gravityZero = iDynTree.Vector3();
gravityZero.zero();
% LEFT---------------------------------------------------------------------
% position
leftHeel_T_leftFtShoePos = iDynTree.Position();
leftHeel_T_leftFtShoePos.fromMatlab(ftShoeSeenFromHeel); % in m
leftFoot_T_leftHeelPos = iDynTree.Position();
leftHeelSeenFromLeftFoot = heelSeenFromFoot;
leftFoot_T_leftHeelPos.fromMatlab(leftHeelSeenFromLeftFoot); % in m
for i = 1 : samples
    % rotation => hp: pure rotation around y axis
    leftFoot_T_leftFtShoeRot = iDynTree.Rotation();
    [~, shoes.betaInDeg_LF(i), leftFoot_R_leftFtShoe_minus]  = computePureRotationAroundY(orientationInQuat.leftFootOrientation(:,i));
    shoes.leftFoot_R_leftFtShoe{i,1} = leftFoot_R_leftFtShoe_minus;
    leftFoot_T_leftFtShoeRot.fromMatlab(leftFoot_R_leftFtShoe_minus);
    % transform
    leftFoot_T_leftFtShoe = iDynTree.Transform(leftFoot_T_leftFtShoeRot,...
        leftFoot_T_leftHeelPos + leftHeel_T_leftFtShoePos);
    leftShoeWrench = wearData.ftShoes.Left(:,i);
    shoes.Left_HF(:,i) = -1 * leftFoot_T_leftFtShoe.asAdjointTransformWrench().toMatlab() * leftShoeWrench;
    % compute G_R_S(left) = G_R_F * F_R_S(left)
    G_R_leftFoot = quat2Mat(orientationInQuat.leftFootOrientation(:,i));
    shoes.G_R_leftFtShoe{i,1} = G_R_leftFoot * leftFoot_R_leftFtShoe_minus;
    
    shoes.G_f_leftFtShoe(:,i) = -1 * shoes.G_R_leftFtShoe{i,1} * leftShoeWrench(1:3,:);
end
% RIGHT--------------------------------------------------------------------
% position
rightHeel_T_rightFtShoePos = iDynTree.Position();
rightHeel_T_rightFtShoePos.fromMatlab(ftShoeSeenFromHeel); % in m
rightFoot_T_rightHeelPos = iDynTree.Position();
rightHeelSeenFromRightFoot = heelSeenFromFoot;
rightFoot_T_rightHeelPos.fromMatlab(rightHeelSeenFromRightFoot); % in m
for i = 1 : samples
    % rotation => hp: pure rotation around y axis
    rightFoot_T_rightFtShoeRot = iDynTree.Rotation();
    [~, shoes.betaInDeg_RF(1,i), rightFoot_R_rightFtShoe_minus] = computePureRotationAroundY(orientationInQuat.rightFootOrientation(:,i));
    shoes.rightFoot_R_rightFtShoe{i,1} = rightFoot_R_rightFtShoe_minus;
    rightFoot_T_rightFtShoeRot.fromMatlab(rightFoot_R_rightFtShoe_minus);
    % transform
    rightFoot_T_rightFtShoe = iDynTree.Transform(rightFoot_T_rightFtShoeRot,...
        rightFoot_T_rightHeelPos + rightHeel_T_rightFtShoePos);
    rightShoeWrench = wearData.ftShoes.Right(:,i);
    shoes.Right_HF(:,i) = -1 * rightFoot_T_rightFtShoe.asAdjointTransformWrench().toMatlab() * rightShoeWrench;
    % compute G_R_S(right) = G_R_F * F_R_S(right)
    G_R_rightFoot = quat2Mat(orientationInQuat.rightFootOrientation(:,i));
    shoes.G_R_rightFtShoe{i,1} = G_R_rightFoot * rightFoot_R_rightFtShoe_minus;
    
    shoes.G_f_rightFtShoe(:,i) = -1 * shoes.G_R_rightFtShoe{i,1} * rightShoeWrench(1:3,:);
end

% Computation of the equation terms
for i = 1 : samples
    % term 1: b_R_G * G_R_Sleft * f_Sleft + b_R_G * G_R_Sright * f_Sright
    G_R_b = quat2Mat(orientationInQuat.baseOrientation(:,i));
    shoes.term1(:,i) = G_R_b' * shoes.G_R_leftFtShoe{i,1} * wearData.ftShoes.Left(1:3,i) + ...
        G_R_b' * shoes.G_R_rightFtShoe{i,1} * wearData.ftShoes.Right(1:3,i); 
    % term 2: b_R_G * m * g,  g = [0;0;9.81]
    shoes.term2(:,i) =G_R_b' * mass * [0.0; 0.0; 9.81];
end
end

%% Inline function
function [R_y_inRad, betaInDeg, R_y_inRad_minus] = computePureRotationAroundY(orientationInQuaternion)
e3 = [0;0;1];
G_R_foot = quat2Mat(orientationInQuaternion);
zComponentOf_G_R_foot = G_R_foot * e3;
cosBeta = e3' * zComponentOf_G_R_foot;
sinBeta = sqrt(1-cosBeta^2);
R_y_inRad = [cosBeta, 0.0, sinBeta;
                 0.0, 1.0, 0.0;
            -sinBeta, 0.0, cosBeta];
tan_y = sqrt(sinBeta^2/cosBeta^2);
betaInDeg = atan(tan_y) * 180/pi;
R_y_inRad_minus = [cos(-atan(tan_y)), 0.0, sin(-atan(tan_y));
                 0.0, 1.0, 0.0;
            -sin(-atan(tan_y)), 0.0, cos(-atan(tan_y))];
end
