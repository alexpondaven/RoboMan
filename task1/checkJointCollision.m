function ret = checkJointCollision(jointPos)
    JOINT_OFFSET = 20;

    joint_x = jointPos(1);
    joint_y = jointPos(2);
    joint_z = jointPos(3);
%     if (joint_x>250 || joint_x < -100)
%         "Joint X out of bounds"
%     end
%    
%     if (joint_y>250 || joint_y < -250)
%         "Joint Y out of bounds"
%     end
    if (joint_z > 250 || joint_z < 0 + JOINT_OFFSET)
        disp("Joint Z out of bounds")
        ret = -1;
        return
    end
    
    ret = 0;
end