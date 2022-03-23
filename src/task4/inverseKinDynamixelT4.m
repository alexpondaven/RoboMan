function raw_angles = inverseKinDynamixelT4(x,y,z,theta_G,theta5)
    % Function to convert raw angles in radians to Dynamixel servo ticks.
    
        [raw_angles, ec] = inverseKin2(x,y,z,theta_G,true);
    
        % Need to add on our own theta5 for Task 4 (specify rotation of
        % gripper)
        raw_angles = [raw_angles(1:4) theta5];
        if ec~=0
            IK_ErrorCodes(ec);
        end
        PARAMS = getJointParams();
        
        % modify raw_angles
        for i=1:5
            % Modify raw_angles
            raw_angles(i) = raw_angles(i) * PARAMS(i).invert;     % invert if necessary (for shoulder, wrist, elbow)
            raw_angles(i) = raw_angles(i) + PARAMS(i).offset;
    
            % Convert to ticks
            raw_angles(i) = raw_angles(i) * 2048/pi;
            raw_angles(i) = cast(raw_angles(i), 'uint32');
    
            % Make sure raw_angles is in range described by PARAMS
            if raw_angles(i) > PARAMS(i).UB
                fprintf("Angle %d of servo %d over upper bound %d\n", raw_angles(i), i, PARAMS(i).UB)
                
                raw_angles(i) = PARAMS(i).UB;
            end
            if raw_angles(i) < PARAMS(i).LB
                fprintf("Angle %d of servo %d under lower bound %d\n", raw_angles(i), i, PARAMS(i).LB)
                
                raw_angles(i) = PARAMS(i).LB;
            end
            
        end
    
    end