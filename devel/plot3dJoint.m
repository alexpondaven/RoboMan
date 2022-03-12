%% Code snippet to test plotting two spaces in 3D, testing for the AStarDriver code block.

trf0 = getDHTransform(0,0,pi/2,0);
trf1 = getDHTransform(pi-pi/2,0,0,0);
trf2 = getDHTransform(0,10,0,0);

base = eye(4);

loc = ((base*trf0)*trf1)*trf2;
plot3(1,1,1)
hold on
plot3([base(1,4), loc(1,4)], [base(2,4), loc(2,4)], [base(3,4), loc(3,4)], 'k.-')
hold off
grid on