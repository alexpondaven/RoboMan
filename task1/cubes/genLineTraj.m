function posPath = genLineTraj(pos1,pos2)
% GENTRAJ   generate trajectory between given points
%
% Take into account velocity?
%
% Sample points according to sine wave as gaps
% Smaller gaps at ends
gaps = sin(0:0.1:pi);
normGaps = cumsum(gaps / sum(gaps));

posPath = zeros(size(normGaps,2),3);
for i=1:size(normGaps,2)
    posPath(i,:) = pos1 + normGaps(i) * (pos2-pos1);
end

end