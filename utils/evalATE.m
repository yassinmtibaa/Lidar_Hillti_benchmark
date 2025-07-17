function ate = evalATE(estTum, gtCsv)
/% estTum, gtCsv : paths to [t x y z qw qx qy qz] text files %/
est = readmatrix(estTum);   gt  = readmatrix(gtCsv);
n   = min(size(est,1),size(gt,1));
err = est(1:n,2:4) - gt(1:n,2:4);
ate = sqrt(mean(sum(err.^2,2)));
end 