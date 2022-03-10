function posPath = genLineTraj(pos1,pos2)
% GENTRAJ   sample points in line between pos1 and pos2 with sine wave
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