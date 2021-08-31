
ls_ci_mean = mean(ls_ci_error);
ls_ci_std = std(ls_ci_error);

ls_bda_mean = mean(ls_bda_error);
ls_bda_std = std(ls_bda_error);

ls_cen_mean = mean(ls_cen_error);
ls_cen_std = std(ls_cen_error);

gs_sci_mean = mean(gs_sci_error);
gs_sci_std = std(gs_sci_error);

gs_ci_mean = mean(gs_ci_error);
gs_ci_std = std(gs_ci_error);


width = 1.4;

figure;
hold on;

H_ls_ci_mean = plot(t, ls_ci_mean, 'Color', Tangerine, 'LineWidth', width);
%H_ls_ci_std = plot(t, [ls_ci_mean - 2*ls_ci_std; ls_ci_mean + 2*ls_ci_std], '--', 'Color', colorGen(Tangerine));

H_ls_bda_mean = plot(t, ls_bda_mean, 'Color', TreeTop, 'LineWidth', width);
%H_ls_bda_std = plot(t, [ls_bda_mean - 2*ls_bda_std; ls_bda_mean + 2*ls_bda_std], '--', 'Color', colorGen(TreeTop));

H_ls_cen_mean = plot(t, ls_cen_mean, 'Color', Cayenne, 'LineWidth', width);
%H_ls_cen_std = plot(t, [ls_cen_mean - 2*ls_cen_std; ls_cen_mean + 2*ls_cen_std], '--', 'Color', colorGen(Cayenne));

H_gs_sci_mean = plot(t, gs_sci_mean, 'Color', Freesia, 'LineWidth', width);
%H_gs_sci_std = plot(t, [gs_sci_mean - 2*gs_sci_std; gs_ci_mean + 2*gs_sci_std], '--', 'Color', colorGen(Freesia));

H_gs_ci_mean = plot(t, gs_ci_mean, 'Color', ClassicBlue, 'LineWidth', width);
%H_gs_ci_std = plot(t, [gs_ci_mean - 2*gs_ci_std; gs_ci_mean + 2*gs_ci_std], '--', 'Color', colorGen(ClassicBlue));

axis([ 0 10 0 0.3])


legend([H_ls_ci_mean, H_ls_bda_mean, H_ls_cen_mean, H_gs_sci_mean, H_gs_ci_mean], ...
    'LS-CI', 'LS-BDA', 'LS-Cen', 'GS-SCI', 'GS-CI', ...
    'Location', 'Northeast');

xlabel('time (s)');
ylabel('RMS error (m)')