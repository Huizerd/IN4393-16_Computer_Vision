function [M_adjusted,S_adjusted] = bundle_adjust_j(M,S,D)

    htw_s = size(S, 1) * size(S, 2);

    returnOnlyNonNaN = @(X) (X(~isnan(X)));

    minFun = @(S, M) returnOnlyNonNaN(D-M*S);
    minFunVectorized = @(vars) minFun( reshape(vars(1:htw_s), size(S, 1), size(S, 2)), ...
                                       reshape(vars(htw_s+1:end), size(M, 1), size(M, 2)));

    initialVector = [S(:) ; M(:)];

    var_vect = lsqnonlin(minFunVectorized, initialVector, [], [], ...
        optimoptions(@lsqnonlin, 'Display', 'iter', 'PlotFcn', ...
        {@optimplotx,@optimplotfval,@optimplotresnorm,@optimplotstepsize}));

    S_adjusted = reshape(var_vect(1:htw_s), size(S, 1), size(S, 2));
    M_adjusted = reshape(var_vect(htw_s+1:end), size(M, 1), size(M, 2));
end