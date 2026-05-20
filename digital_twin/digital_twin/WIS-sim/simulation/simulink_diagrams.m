%% Save Simulink Diagrams as PDF for use in Latex report

simulate_3pools

diagrams = ["simulate_3pools", "Global controller hatK_i", "Local controller W_i", "W1 implementation", "Undershot gate simulation", "Pool simulation P_i - digital delay"];


for diagram = diagrams
    print(sprintf('-s%s',diagram{1}), '-dpdf', sprintf('/Users/bas/Dropbox/Apps/Overleaf/Master Thesis/images/simulink/sim_%s.pdf',diagram{1}))
end

close_system

simulate_3pools_hil

diagrams = ["simulate_3pools_hil", "Firefly1", "Pressure Sensors"];


for diagram = diagrams
    print(sprintf('-s%s',diagram{1}), '-dpdf', sprintf('/Users/bas/Dropbox/Apps/Overleaf/Master Thesis/images/simulink/hil_%s.pdf',diagram{1}))
end

close_system