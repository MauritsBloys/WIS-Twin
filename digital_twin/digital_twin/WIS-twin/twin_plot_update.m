function twin_plot_update(handles, t_vec, y_hist, y_pred_hist, innov_hist, u_hist, K_diag_hist, mpc_traj, ~, y_nompc_hist, alpha_hat_hist, beta_hat_hist, q_leak_est_hist, q_leak_nom_hist)
%TWIN_PLOT_UPDATE  Refresh all live plot windows with current history.
%   alpha_hat_hist, beta_hat_hist optioneel (AEMF params, fig 8).
%   q_leak_est_hist, q_leak_nom_hist optioneel (lekkageflow [cm³/s], fig 9).

for i = 1:3
    set(handles.h_meas(i),  'XData', t_vec, 'YData', y_hist(i,:));
    set(handles.h_pred(i),  'XData', t_vec, 'YData', y_pred_hist(i,:));
    set(handles.h_u(i),     'XData', t_vec, 'YData', u_hist(i,:));
    set(handles.h_innov(i), 'XData', t_vec, 'YData', innov_hist(i,:));
    set(handles.h_kg(i),    'XData', t_vec, 'YData', K_diag_hist(i,:));
end

t_hor = t_vec(end) + (0:size(mpc_traj,2)-1);
for i = 1:3
    set(handles.h_hor(i), 'XData', t_hor, 'YData', mpc_traj(i,:));
end

if nargin >= 10 && ~isempty(y_nompc_hist)
    cantoni_to_servo = 255 / 0.5;   % u_max = 0.5 Cantoni → servo 255
    for i = 1:3
        set(handles.h_cmp_mpc(i), 'XData', t_vec, 'YData', y_hist(i,:));
        set(handles.h_cmp_nom(i), 'XData', t_vec, 'YData', y_nompc_hist(i,:));
        % Sluisbewegingen als trapvorm, geschaald naar servo-eenheden
        [t_s, u_s] = make_stairs(t_vec, u_hist(i,:) * cantoni_to_servo);
        set(handles.h_gates(i), 'XData', t_s, 'YData', u_s);
    end
end

% Figuur 8: geschatte alpha en beta per kanaal (AEMF H(q)x + (L(q)+aL1+bL2)z)
if nargin >= 12 && isfield(handles, 'h_alpha')
    for i = 1:3
        valid = ~isnan(alpha_hat_hist(i,:));
        if any(valid)
            set(handles.h_alpha(i), 'XData', t_vec(valid), 'YData', alpha_hat_hist(i, valid));
        end
        valid = ~isnan(beta_hat_hist(i,:));
        if any(valid)
            set(handles.h_beta(i), 'XData', t_vec(valid), 'YData', beta_hat_hist(i, valid));
        end
    end
end

% Figuur 9: lekkageflow per kanaal [cm³/s]
if nargin >= 14 && isfield(handles, 'h_qest')
    for i = 1:3
        valid_e = ~isnan(q_leak_est_hist(i,:));
        valid_n = ~isnan(q_leak_nom_hist(i,:));
        if any(valid_e)
            q_over = q_leak_est_hist(i,:) - q_leak_nom_hist(i,:);
            set(handles.h_qest(i),  'XData', t_vec(valid_e), 'YData', q_leak_est_hist(i, valid_e));
            set(handles.h_qover(i), 'XData', t_vec(valid_e), 'YData', q_over(valid_e));
        end
        if any(valid_n)
            set(handles.h_qnom(i), 'XData', t_vec(valid_n), 'YData', q_leak_nom_hist(i, valid_n));
        end
    end
end

drawnow;
end

function [t_out, u_out] = make_stairs(t, u)
%MAKE_STAIRS  Zet tijdreeks om naar trapvorm voor sluisvisualisatie.
n     = numel(t);
t_out = reshape([t; [t(2:end), t(end)]], 1, 2*n);
u_out = reshape([u; u], 1, 2*n);
end
