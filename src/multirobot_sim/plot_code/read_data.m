% location error
ls_cen_error = readmatrix('../ls_cen/error.txt');
ls_ci_error = readmatrix('../ls_ci/error.txt');
ls_bda_error = readmatrix('../ls_bda/error.txt');
gs_sci_error = readmatrix('../gs_sci/error.txt');
gs_ci_error = readmatrix('../gs_ci/error.txt');

% trace error
ls_cen_tr = readmatrix('../ls_cen/tr.txt'); ls_cen_th_tr = readmatrix('../ls_cen/th_tr.txt');
ls_ci_tr = readmatrix('../ls_ci/tr.txt');
ls_bda_tr = readmatrix('../ls_bda/tr.txt');
gs_sci_tr = readmatrix('../gs_sci/tr.txt');
gs_ci_tr = readmatrix('../gs_ci/tr.txt'); gs_ci_th_tr = readmatrix('../gs_ci/th_tr.txt');

% fix last column
% ls_cen_error = ls_cen_error(:,1:end-1);
% ls_ci_error = ls_ci_error(:,1:end-1);
% ls_bda_error = ls_bda_error(:,1:end-1);
% gs_sci_error = gs_sci_error(:,1:end-1);
% gs_ci_error = gs_ci_error(:,1:end-1);
% 
% ls_cen_tr = ls_cen_tr(:,1:end-1); ls_cen_th_tr = ls_cen_th_tr(:,1:end-1);
% ls_ci_tr = ls_ci_tr(:,1:end-1);
% ls_bda_tr = ls_bda_tr(:,1:end-1);
% gs_sci_tr = gs_sci_tr(:,1:end-1);
% gs_ci_tr = gs_ci_tr(:,1:end-1); gs_ci_th_tr = gs_ci_th_tr(:,1:end-1);

t = linspace(0,10,501);
run color