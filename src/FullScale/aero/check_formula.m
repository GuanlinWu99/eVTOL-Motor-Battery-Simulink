alpha = 10/180*pi;
beta = -10/180*pi;

%.. force conversion
%.. CFx (back), CFy (right), CFz (up) -> CD (back), CS (right), CL (up) => negative of wind
([cos(beta) sin(beta) 0; -sin(beta) cos(beta) 0; 0 0 1]*...
[cos(alpha) 0 sin(alpha); 0 1 0; -sin(alpha) 0 cos(alpha)]*...
[-(-0.176597455602999); (0.0268344802269999); -(1.56222604452)]).*[-1; 1; -1]

%.. moment conversion (question, why are the moments computed in wind axes, not body axes?)
%.. CFx (back), CFy (right), CFz (up) -> CMl (front), CMm (right), CMn (down) => wind
[-(-0.00745548778199999); (-0.491519290817999); -(-0.000432990528)]
