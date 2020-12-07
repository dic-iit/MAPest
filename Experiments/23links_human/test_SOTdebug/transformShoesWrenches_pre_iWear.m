%% PRE

function [shoes] = transformShoesWrenches_pre_iWear(orientationInQuat, wearData, height_heelFoot, mass)
% TRANSFORMSHOESWRENCHES transforms external wrenches coming
% from the ftShoes into human frames.
% Shoes wrenches are estimated in their frames (origin and
% orientation) that are located at a known position.
% For the human estimation we need to get from this wrenches but:
% - multiplied by -1 (as the wrench applied on the human is exactly the
%   opposite of the one excerted on each shoe)
% - expressed in the frame of the human link in contact.
%
% This function computes the wrenches that the each shoe exerts on the link
% in contact.  See the sketch footInShoe.pdf for the reference frames
% locations.

heelSeenFromFoot = [-0.0383; 0 ; -height_heelFoot];

%% Build the transformations
gravityZero = iDynTree.Vector3();
gravityZero.zero();

% LEFT---------------------------------------------------------------------
leftHeel_T_leftFtShoeRot = iDynTree.Rotation();
leftHeel_T_leftFtShoeRot.fromMatlab([ 1.0,  0.0,  0.0; ...
    0.0,  1.0,  0.0; ...
    0.0,  0.0,  1.0]);
leftHeel_T_leftFtShoePos = iDynTree.Position();
leftFtShoeSeenFromLeftHeel = [0.037; 0 ; -0.050]; %FtShoe (totalForce ref)
leftHeel_T_leftFtShoePos.fromMatlab(leftFtShoeSeenFromLeftHeel); % in m
leftFoot_T_leftHeelPos = iDynTree.Position();
leftHeelSeenFromLeftFoot = heelSeenFromFoot;
leftFoot_T_leftHeelPos.fromMatlab(leftHeelSeenFromLeftFoot); % in m
leftFoot_T_leftFtShoe = iDynTree.Transform(leftHeel_T_leftFtShoeRot,...
    leftFoot_T_leftHeelPos + leftHeel_T_leftFtShoePos);

% RIGHT--------------------------------------------------------------------
rightHeel_T_rightFtShoeRot = iDynTree.Rotation();
rightHeel_T_rightFtShoeRot.fromMatlab([ 1.0,  0.0,  0.0; ...
    0.0,  1.0,  0.0; ...
    0.0,  0.0,  1.0]);
rightHeel_T_rightFtShoePos = iDynTree.Position();
rightFtShoeSeenFromRightHeel = [0.037; 0 ; -0.050];
rightHeel_T_rightFtShoePos.fromMatlab(rightFtShoeSeenFromRightHeel); % in m
rightFoot_T_rightHeelPos = iDynTree.Position();
rightHeelSeenFromRightFoot = heelSeenFromFoot;
rightFoot_T_rightHeelPos.fromMatlab(rightHeelSeenFromRightFoot); % in m
rightFoot_T_rightFtShoe = iDynTree.Transform(rightHeel_T_rightFtShoeRot,...
    rightFoot_T_rightHeelPos + rightHeel_T_rightFtShoePos);

%% Transform wrenches from shoes frames into human frames

leftShoeWrench(1:3,:) = wearData.ftShoes.Left(1:3,:);
leftShoeWrench(4:6,:) = wearData.ftShoes.Left(4:6,:);

rightShoeWrench(1:3,:) = wearData.ftShoes.Right(1:3,:);
rightShoeWrench(4:6,:) = wearData.ftShoes.Right(1:3,:);

samples = size(orientationInQuat.leftFootOrientation,2); % ---- test
% HF = human frame
shoes.Left_HF = ...
    -1*(leftFoot_T_leftFtShoe.asAdjointTransformWrench().toMatlab()*leftShoeWrench);
shoes.Right_HF = ...
    -1*(rightFoot_T_rightFtShoe.asAdjointTransformWrench().toMatlab()*rightShoeWrench);

% ---- test
identityMatrix = [ 1.0,  0.0,  0.0; ...
    0.0,  1.0,  0.0; ...
    0.0,  0.0,  1.0];
for i = 1 : samples
    % right
    G_R_rightFoot = quat2Mat(orientationInQuat.rightFootOrientation(:,i));
    shoes.G_R_rightFtShoe{i,1} = G_R_rightFoot * identityMatrix;
    shoes.G_f_rightFtShoe(:,i) = -1 * shoes.G_R_rightFtShoe{i,1} * rightShoeWrench(1:3,i);
    % left
    G_R_leftFoot = quat2Mat(orientationInQuat.leftFootOrientation(:,i));
    shoes.G_R_leftFtShoe{i,1} = G_R_leftFoot * identityMatrix;
    shoes.G_f_leftFtShoe(:,i) = -1 * shoes.G_R_leftFtShoe{i,1} * leftShoeWrench(1:3,i);
    
    % test balance
    % term 1: b_R_G * G_R_Sleft * f_Sleft + b_R_G * G_R_Sright * f_Sright
    G_R_b = quat2Mat(orientationInQuat.baseOrientation(:,i));
    shoes.term1(:,i) = G_R_b' * shoes.G_R_leftFtShoe{i,1} * wearData.ftShoes.Left(1:3,i) + ...
        G_R_b' * shoes.G_R_rightFtShoe{i,1} * wearData.ftShoes.Right(1:3,i); 
    % term 2: b_R_G * m * g,  g = [0;0;9.81]
    shoes.term2(:,i) = G_R_b' * mass * [0.0; 0.0; 9.81];
end
% ----
end
