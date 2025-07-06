function cost = compute_symmetry_cost(params, theta_left, theta_right, time, interp_func)
    alpha = params(1);
    beta  = params(2);
    delta = params(3);

    warped_time = beta * time + delta;
    warped_left = interp_func(theta_left, warped_time);

    error = theta_right + alpha * warped_left;
    cost = sum(error.^2);
end
