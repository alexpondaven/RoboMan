cubeMoves = [[2,2,1];
            [2,2,1]];
cubeStacks = [0,1,0,0,0,0];
startPos = [225,0,100,0];

[via_paths, isHoldingCube, waypoints] = planCubesPath(cubeMoves, cubeStacks, startPos)