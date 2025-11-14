%% ========= SOH & SOP parameters ========= %%
C_nom_Ah   = 3*200;                                                         % 【Ref】https://www.batemo.com/products/batemo-cell-explorer/samsung-inr18650-30q/
Vmin       = 2.5*200;     
Vmax       = 3.6*200;     
Imax_dis   = 22.6*200;                                                      
R0_ref     = [];      

%% ========= Pull data from outTuned ========= %%
t = outTuned.Battery_Data.Batt.Voltage__V_.Time(:);
V = outTuned.Battery_Data.Batt.Voltage__V_.Data(:);
I = outTuned.Battery_Data.Batt.Current__A_.Data(:);
SOC = outTuned.Battery_Data.Batt.SOC____.Data(:); 
Voc = outTuned.Battery_Data.Batt.OCV.Data(:);
Temp = outTuned.Battery_Data.Batt.signal7.Data(:);
SOC = SOC/100;

dt = [0; diff(t)];                                                          % [s]
Ts_med = median(diff(t));


%% ========= SOP (discharge), kW ========= %%
Ns                                        = 200;                            % number of series
Np                                        = 200;                            % number of parallel

% Samsung INR18650-300 cell 
% Scaling law from Prof. Jung's group at 50% soc
R0                                        = 0.012;
R1                                        = 0.004;
R2                                        = 0.0015;
C1                                        = 136.29;
C2                                        = 872.87;
Tau1                                      = R1*C1;
R                                      = ones(size(I)) * ((Ns/Np)*R0   +   (Ns/Np)*R1  + (Ns/Np)*R2);             % Battery pack resistance[Ohm]
% Voc                                       = 800;%V + I .* R;                     % approximate ocv = terminal voltage + I*Ruse

% Not violate minimum voltage
Iv_dis = max((Voc - Vmin) ./ R, 0);                                         % 【Ref】Review of State of Power Estimation for Li-Ion Batteries: 
Pv_dis = Vmin .* Iv_dis;                                                    % Methods, Issues, and Prospects


% Not violate maximum current
Pi_dis = Voc .* Imax_dis - R .* (Imax_dis.^2);
Pi_dis = max(Pi_dis, 0);
SOP_dis_kW = min(Pv_dis, Pi_dis) / 1000;



%% ========= Plot SOP ========= %%
figure('Units','normalized','OuterPosition',[0.52 0.1 0.42 0.7],'Color','w');
tiledlayout(3,2,'Padding','compact','TileSpacing','compact');

nexttile; plot(t,SOC,'LineWidth',1.8); grid on; ylabel('SOC (-)'); xlabel('t (s)'); title('SOC');
nexttile; plot(t,V,'LineWidth',1.8); grid on; ylabel('V (V)'); xlabel('t (s)'); title('Terminal Voltage');
nexttile; plot(t,Temp,'LineWidth',1.8); grid on; ylabel('Temp (K)'); xlabel('t (s)'); title('Temperature');

nexttile; plot(t,Voc, 'LineWidth', 1.8);grid on; ylabel('OCV(V)');xlabel('t(s)');title('Open Circuit Voltage');
nexttile; plot(t,SOP_dis_kW,'LineWidth',1.8); grid on; ylabel('kW'); xlabel('t (s)'); title('SOP_{dis} (max deliverable)');
