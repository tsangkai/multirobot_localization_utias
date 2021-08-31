

ls_ci_tr_x = mean(ls_ci_tr);
ls_ci_tr_y = std(ls_ci_tr);

ls_bda_tr_x = mean(ls_bda_tr);
ls_bda_tr_y = std(ls_bda_tr);

ls_cen_tr_x = mean(ls_cen_tr);
ls_cen_tr_y = std(ls_cen_tr);
ls_cen_th_tr_x = mean(ls_cen_th_tr);

gs_sci_tr_x = mean(gs_sci_tr);
gs_sci_tr_y = std(gs_sci_tr);

gs_ci_tr_x = mean(gs_ci_tr);
gs_ci_tr_y = std(gs_ci_tr);
gs_ci_th_tr_x = mean(gs_ci_th_tr);


width = 1.4;

%%%
figure;
hold on;


H_ls_ci_tr_x = plot(t, ls_ci_tr_x, 'Color', Tangerine, 'LineWidth', width);


H_ls_bda_tr_x = plot(t, ls_bda_tr_x, 'Color', TreeTop, 'LineWidth', width);

H_ls_cen_tr_x = plot(t, ls_cen_tr_x, 'Color', Cayenne, 'LineWidth', width);
H_ls_cen_th_tr_x = plot(t, ls_cen_th_tr_x, '-.', 'Color', colorGen(Cayenne), 'LineWidth', width);

H_gs_sci_tr_x = plot(t, gs_sci_tr_x, 'Color', Freesia, 'LineWidth', width);


H_gs_ci_tr_x = plot(t, gs_ci_tr_x, 'Color', ClassicBlue, 'LineWidth', width);
H_gs_ci_th_tr_x = plot(t, gs_ci_th_tr_x, '-.', 'Color', colorGen(ClassicBlue), 'LineWidth', width);

axis([ 0 10 0 0.3])


legend([H_ls_ci_tr_x, H_ls_bda_tr_x, H_ls_cen_tr_x,H_ls_cen_th_tr_x, H_gs_sci_tr_x, H_gs_ci_tr_x, H_gs_ci_th_tr_x], ...
    'LS-CI', 'LS-BDA', 'LS-Cen', 'LS-Cen (anly.)', 'GS-SCI', 'GS-CI', 'GS-CI (anly.)', ...
    'Location', 'Northeast');

xlabel('time (s)');
ylabel('RMT error (m)')

