function twin_plot_leakage_curves(alpha_hat_hist, beta_hat_hist, Wis)
%TWIN_PLOT_LEAKAGE_CURVES  Post-run plot: geschatte q(Dh)-curven vs. nominaal.
%
%   Toont voor elk lekkagekanaal de volledige q(Dh)-curve voor de
%   gemiddelde geschatte alpha_eff en beta_eff over de run.
%
%   Inputs:
%     alpha_hat_hist — [3xT] geschatte alpha per tijdstap (NaN = niet beschikbaar)
%     beta_hat_hist  — [3xT] geschatte beta  per tijdstap
%     Wis            — struct: leak_alpha, leak_beta

valid = ~any(isnan(alpha_hat_hist), 1) & ~any(isnan(beta_hat_hist), 1);
if sum(valid) < 1
    fprintf('twin_plot_leakage_curves: geen geldige schattingen beschikbaar.\n');
    return;
end

alpha_eff = mean(alpha_hat_hist(:, valid), 2);
beta_eff  = mean(beta_hat_hist(:, valid),  2);

dh_cm  = linspace(0, 18, 200);
colors = {'#1f77b4', '#ff7f0e', '#2ca02c'};
labels = {'Bak0\rightarrowBassin 1', 'Bassin 1\rightarrow2', 'Bassin 2\rightarrow3'};

figure('Name', 'WIS — Lekkagecurven (AEMF post-run)', 'NumberTitle', 'off', ...
       'Color', 'white', 'Position', [150 150 860 620]);

for j = 1:3
    subplot(3, 1, j); hold on; box on; grid on;

    q_nom = Wis.leak_alpha(j) * sqrt(dh_cm) + Wis.leak_beta(j) * dh_cm.^(3/2);
    q_eff = max(0, alpha_eff(j) * sqrt(dh_cm) + beta_eff(j) * dh_cm.^(3/2));

    plot(dh_cm, q_nom, '--', 'Color', colors{j}, 'LineWidth', 1.5, 'DisplayName', 'Nominaal');
    plot(dh_cm, q_eff, '-',  'Color', colors{j}, 'LineWidth', 2.5, 'DisplayName', 'Geschat (AEMF)');

    xlabel('\Deltah [cm]'); ylabel('q [cm^3/s]');
    title(sprintf('Kanaal %d: %s  |  \\alpha_{eff}=%.2f (nom %.2f),  \\beta_{eff}=%.3f (nom %.3f)', ...
          j, labels{j}, alpha_eff(j), Wis.leak_alpha(j), beta_eff(j), Wis.leak_beta(j)));
    legend('Location', 'northwest');
end

sgtitle(sprintf('Lekkagecurven: nominaal vs. AEMF-schatting  (%d tijdstappen)', sum(valid)), ...
        'FontSize', 12, 'FontWeight', 'bold');
end
