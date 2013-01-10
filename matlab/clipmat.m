% clipmat(M, lo, hi)
% Clip each of the entries in the given matrix to the range (lo,hi)
%
function M2 = clipmat(M, lo, hi)
    M2 = M.*(M<=hi) + hi.*(M>hi);
    M2 = M2.*(M2>=lo) + lo.*(M2<lo);
