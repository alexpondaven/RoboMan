function joints = getJointPositions(theta)
% Returns joint positions given theta values calculated from an Inverse
% Kinematics function (eg. `inverseKin2`)
    T = getTransformMatrices(theta);

    % Base Link
    jointTransform(:,:,1) = T(:,:,1);
    joints(:,1) = jointTransform(1:3,4,1);
    % Apply transforms
    for i=2:size(T,3)
        jointTransform(:,:,i) = jointTransform(:,:,i-1) * T(:,:,i);
        joints(:,i) = jointTransform(1:3,4,i);
    end

end