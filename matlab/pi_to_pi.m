% PI_TO_PI Normalize the angles to the range (-pi, pi]
%
% OUTPUT
%   - angle: Normalized angles.
%
% INPUT: 
%   - angle: Array of angles.

function angle = pi_to_pi(angle)
%----------------------------------------------------------------------%
% 1. MAKE SURE THAT THE ANGLE IS IN THE RANGE: [-2*pi, 2*pi].
%----------------------------------------------------------------------%
twopi = 2*pi;
angle = angle - twopi*fix(angle/twopi);

%----------------------------------------------------------------------%
% 2. MAKE SURE THAT THE ANGLE IS IN THE RANGE: (-pi, pi]
%----------------------------------------------------------------------%
%  2.1. For all the angles "angle > pi".
i = find(angle > pi);
angle(i) = angle(i) - twopi;

%  2.2. For all the angles "angle <= -pi"
i = find(angle <= -pi);
angle(i) = angle(i) + twopi;

end % END