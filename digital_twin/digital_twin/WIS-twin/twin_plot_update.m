function twin_plot_update(handles, t_vec, y_hist, y_pred_hist, innov_hist, u_hist, K_diag_hist, mpc_traj, ~, y_nompc_hist, q_nom_hist, q_hat_hist, c_hat_hist)
%TWIN_PLOT_UPDATE  Refresh all live plot windows with current history.
%   Call every time step inside the main loop.
%   q_nom_hist, q_hat_hist, c_hat_hist zijn optioneel (AEMF lekkageplots).

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

% Figuur 8: lekkageflow en effectieve alpha (AEMF)
if nargin >= 13 && isfield(handles, 'h_qnom')
    for i = 1:3
        set(handles.h_qnom(i), 'XData', t_vec, 'YData', q_nom_hist(i,:));
        valid_q = ~isnan(q_hat_hist(i,:));
        if any(valid_q)
            set(handles.h_qhat(i), 'XData', t_vec(valid_q), 'YData', q_hat_hist(i, valid_q));
        end
        valid_c = ~isnan(c_hat_hist(i,:));
        if any(valid_c) && ~isempty(handles.alpha_nom)
            set(handles.h_alpha(i), 'XData', t_vec(valid_c), ...
                'YData', handles.alpha_nom(i) * (1 + c_hat_hist(i, valid_c)));
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
