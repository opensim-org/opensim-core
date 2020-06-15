syms b1 b2 b3 b4 lMtilde min_lMtilde c1 c2 c3 lTtilde kT kPE e0

fprintf('Derivative of tendon force length curve \n')
fprintf('======================================= \n')

f_t = c1*exp(kT*(lTtilde - c2)) - c3;
simplify(diff(f_t, lTtilde))

fprintf('Derivative of active fiber force length curve \n')
fprintf('============================================= \n')

f_act = b1 * exp((-0.5*(lMtilde-b2)^2) / ((b3 + b4*lMtilde)^2));
simplify(diff(f_act, lMtilde))

fprintf('Derivative of passive fiber force length curve \n')
fprintf('============================================== \n')

f_pas = (exp(kPE * (lMtilde - 1.0) / e0) - exp(kPE * (min_lMtilde - 1) / e0)) / ...
        (exp(kPE) - exp(kPE * (min_lMtilde - 1) / e0));
simplify(diff(f_pas, lMtilde))

fprintf('Integral of tendon force length curve \n')
fprintf('===================================== \n')

f_t = c1*exp(kT*(lTtilde - c2)) - c3;
simplify(int(f_t, lTtilde))

fprintf('Integral of passive fiber force length curve \n')
fprintf('============================================ \n')

f_pas_int = simplify(int(f_pas, lMtilde))
pretty(f_pas_int)
