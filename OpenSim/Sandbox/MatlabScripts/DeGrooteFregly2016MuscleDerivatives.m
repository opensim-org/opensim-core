syms b1 b2 b3 b4 lMtilde min_lMtilde c1 c2 c3 lTtilde min_lTtilde kT kPE e0 

fprintf('Derivative of tendon force length curve \n')
fprintf('======================================= \n')

f_t = c1*exp(kT*(lTtilde - c2)) - c3;
simplify(diff(f_t, lTtilde))

fprintf('Integral of tendon force length curve \n')
fprintf('===================================== \n')

f_ten = c1*exp(kT*(lTtilde - c2)) - c3;
f_ten_int = simplify(int(f_t, lTtilde, [min_lTtilde, lTtilde]));
pretty(f_ten_int)

% The same tendon force-length curve integral, now written as in the
% DeGrooteFregly2016Muscle class where the full expression is decomposed into
% multiple parts.
temp1 = exp(kT * lTtilde) - exp(kT * min_lTtilde);
temp2 = c1 * exp(-c2 * kT) / kT;
temp3 = c3 * (lTtilde - min_lTtilde);
f_ten_int_decomposed = temp1 * temp2 - temp3;
assert(isequal(f_ten_int, f_ten_int_decomposed), 'Expressions not equal!');

fprintf('Derivative of active fiber force length curve \n')
fprintf('============================================= \n')

f_act = b1 * exp((-0.5*(lMtilde-b2)^2) / ((b3 + b4*lMtilde)^2));
simplify(diff(f_act, lMtilde))

fprintf('Derivative of passive fiber force length curve \n')
fprintf('============================================== \n')

offset = exp(kPE * (min_lMtilde - 1.0) / e0);
denom = exp(kPE) - offset;
f_pas = (exp(kPE * (lMtilde - 1.0) / e0) - offset) / denom;

f_pas_deriv = diff(f_pas, lMtilde);
simplify(f_pas_deriv)

% The same passive fiber force-length curve derivative, now written as in the
% DeGrooteFregly2016Muscle class where the full expression is decomposed into
% multiple parts.
offset = exp(kPE * (min_lMtilde - 1.0) / e0);
f_pas_deriv_decomposed = ...
    (kPE * exp((kPE * (lMtilde - 1)) / e0)) / (e0 * (exp(kPE) - offset));

assert(isequal(f_pas_deriv, f_pas_deriv_decomposed), 'Expressions not equal!');

fprintf('Integral of passive fiber force length curve \n')
fprintf('============================================ \n')

f_pas_int = simplify(int(f_pas, lMtilde, [min_lMtilde, lMtilde]));
pretty(f_pas_int)

% The same passive fiber force-length curve integral, now written as in the
% DeGrooteFregly2016Muscle class where the full expression is decomposed into
% multiple parts.
temp1 = e0 + kPE * lMtilde - kPE * min_lMtilde;
temp2 = exp(kPE * (lMtilde - 1.0) / e0);
temp3 = exp(kPE * (min_lMtilde - 1.0) / e0);
numer = exp(kPE) * temp1 - e0 * temp2;
denom = kPE * (temp3 - exp(kPE));
f_pas_int_decomposed = (temp1 / kPE) + (numer / denom);

assert(isequal(f_pas_int, f_pas_int_decomposed), 'Expressions not equal!');
