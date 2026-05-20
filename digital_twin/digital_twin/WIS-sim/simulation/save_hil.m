%experimentName = "continuous_disturbance_etc_0-01-1";
% experimentName = "continuous_disturbance_etc_0-005-1";
% experimentName = "continuous_disturbance_etc_0-0025-1";
% experimentName = "continuous_disturbance_etc_0-00125-1";

%experimentName = "continuous_disturbance_etc_0-01-0-1";
%experimentName = "continuous_disturbance_etc_0-005-0-1";
%experimentName = "continuous_disturbance_etc_0-0025-0-1"; 
%experimentName = "continuous_disturbance_etc_0-00125-0-1"; 

%experimentName = "continuous_disturbance_etc_0-0025-0-01";
%experimentName = "continuous_disturbance_etc_0-005-0-01";
%experimentName = "continuous_disturbance_etc_0-01-0-01";
%experimentName = "continuous_disturbance_etc_0-00125-0-01"; 

%experimentName = "continuous_disturbance_etc_0-0-0-0";

%experimentName = "continuous_disturbance_etc_force_trigger";

%experimentName = "continuous_disturbance_etc_no_trigger";

%experimentName = "continuous_disturbance_periodic";

%experimentName = "continuous_disturbance_etc_0-05-1";

%experimentName = "continuous_disturbance_etc_0-05-1-flow";

%experimentName = "continuous_disturbance_etc_0-1-1-flow";

%experimentName = "continuous_disturbance_etc_0-1-1";

% NEW: Pade for PSTC
%experimentName = "continuous_disturbance_etc_0-0025-0-01-pade";

%experimentName = "20210805test_pstc_force_trigger";

% NEW FIXED TRIGGERING

%experimentName = "new_triggering_etc_0-1_1";

%experimentName = "new_triggering_etc_0-05_1";

%experimentName = "new_triggering_etc_0-025_1";

%experimentName = "new_triggering_etc_0-05_2";

%experimentName = "new_triggering_etc_0-025_2";

%experimentName = "new_triggering_etc_0-1_2";

%experimentName = "new_triggering_etc_0-2_2";

%experimentName = "new_triggering_etc_0-4_2";

%experimentName = "new_triggering_etc_0-4_4-bad";

%experimentName = "new_triggering_etc_0-4_4";

%experimentName = "new_triggering_etc_0-4_8";

%experimentName = "new_triggering_etc_0-4_16";

%experimentName = "new_triggering_etc_0-2_16";

%--

fileName = sprintf('mat/hil_%s.mat', experimentName);

save(fileName,'flow','level','servo','epoch',...
        'radio_on', 'SPS');    
