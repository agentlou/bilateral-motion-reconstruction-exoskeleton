function [alpha, beta, delta, err_min] = optimize_symmetry_params(theta_left, theta_right, time)
    interp_func = @(x, t) interp1(time, x, t, 'linear', 0);  % Safe interpolation

    cost_func = @(params) compute_symmetry_cost(params, theta_left, theta_right, time, interp_func);

    init_guess = [1, 1, 0];  % [alpha, beta, delta]
    lb = [-2, 0.5, -1];
    ub = [2, 2, 1];

    opts = optimoptions('fmincon', 'Display', 'off', 'Algorithm', 'sqp');
    [opt_params, err_min] = fmincon(cost_func, init_guess, [], [], [], [], lb, ub, [], opts);

    alpha = opt_params(1);
    beta  = opt_params(2);
    delta = opt_params(3);
end
