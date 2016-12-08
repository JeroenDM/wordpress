function [f, grad] = nls_casadi(x, cost, grad_cost)
    f = cost(x);
    grad = grad_cost(x);
end
