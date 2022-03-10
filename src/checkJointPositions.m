function jointPosErr = checkJointPositions(joints)
% Checks if joint positions are out of bounds.
% Args:
% joints : vector of xyz positions of the joints (derived from the
% series of transformation matrices)
% Returns:
% one-hot vector of joint positions with errors
    
    % safety factor to keep each joint above the ground
    JOINT_Z_OFFSET = 15;
    
    jointPosErr = zeros( length(joints), 'uint8' );
    
    % joint 1 is the origin, so it is fixed, doesn't need to be checked.
    for idx=2:length(joints)
        joint_z = joints(3,idx);
        
        % TODO make other checks if necessary
        if (joint_z < JOINT_Z_OFFSET)
            jointPosErr(idx) = 1;
        end
    end
    
end