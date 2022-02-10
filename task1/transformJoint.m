function tJoint = transformJoint(joint, T)
    % Transform joint by transformation T
    % Joint made up of position P and coordinate frame CoordX,Y,Z
    tJoint.Pos = T * joint.Pos;
    tJoint.CoordX = T * joint.CoordX;
    tJoint.CoordY = T * joint.CoordY;
    tJoint.CoordZ = T * joint.CoordZ;
end