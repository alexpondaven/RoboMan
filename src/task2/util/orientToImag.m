function imag = orientToImag(orient)
% Convert orientation to imaginary number for easy calculations of rotation
%
% Args
% orient  : Orientation consisting of
%           - "up": upwards
%           - "back": towards robot
%           - "front": away from robot
%           - "down": downwards
%
% Return
% imag    : Unit imaginary number i, 1, -1 or -i

switch orient
    case "up"
        imag = 1i;
    case "back"
        imag = 1;
    case "front"
        imag = -1;
    case "down"
        imag = -1i;
end