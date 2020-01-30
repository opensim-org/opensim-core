function sandboxDGFPassive

global xmin;
global FmaxStrain;

xmin = 0.2;
FmaxStrain = 0.6;

x = linspace(0, 1.8, 1000);

y = passive(x);
plot(x, y);
hold on;
y_new = passive_new(x);
plot(x, y_new);
fprintf('max error: %f\n', max(abs(y_new - y)));

legend('old', 'new');

fprintf('f(xmin): %f\n', passive(xmin));
fprintf('f(1): %f\n', passive(1.0));
fprintf('f(1 + FmaxStrain): %f\n', passive(1 + FmaxStrain));
fprintf('\n');
fprintf('fn(xmin): %f\n', passive_new(xmin));
fprintf('fn(1): %f\n', passive_new(1.0));
fprintf('fn(1 + FmaxStrain): %f\n', passive_new(1 + FmaxStrain));

syms k c x xm;
y = (exp(k*(x - 1)/c) - exp(k*(xm - 1)/c)) / (exp(k) - exp(k*(xm - 1)/c))
yint = simplify(int(y));
pretty(yint)
pretty(simplify(diff(yint, x)))

end

function y = passive(x)

global xmin;
global FmaxStrain;

kPE = 4;
e0 = FmaxStrain;

numer_offset = exp(kPE * (xmin - 1.0) / e0);
% numer_offset = 1;

denom = exp(kPE) - 1.0;
y = (exp(kPE * (x - 1.0) / e0) - numer_offset) / denom;


end

function y = passive_new(x)

global xmin;
global FmaxStrain;

kPE = 4;
e0 = FmaxStrain;

offset = exp(kPE * (xmin - 1.0) / e0);

denom = exp(kPE) - offset;
y = (exp(kPE * (x - 1.0) / e0) - offset) / denom;


end