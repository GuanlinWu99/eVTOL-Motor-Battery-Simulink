%% reconstruct_currents.m
% Reconstruct 3-phase inverter currents from a system-level run (torque/speed),
% purely algebraically:  iq=Te/Kt (id=0),  theta=integral(p*omega),
% ia=-iq*sin(theta), ib/ic shifted +-120deg.
% Full-mission RMS/peak envelope at the log rate; fine waveform only inside WIN.
% Needs in the workspace after a run: outTuned, p, psim.

%% ---- settings ----
if ~exist('dt_fine','var'), dt_fine = 5e-5;  end   % waveform step [s]
if ~exist('WIN','var'),     WIN     = [0 5];  end  % waveform window [s]
motorToPlot = 1;

%% ---- motor params ----
if ~exist('p','var') || ~exist('psim','var')
    warning('p/psim missing; using Evolito defaults (p=10, psim=0.045).');
    p = 10; psim = 0.045;
end
Kt = 1.5*p*psim;

%% ---- extract speed [rad/s] and torque [N*m] per motor ----
if isa(outTuned,'Simulink.SimulationOutput'), fn = outTuned.who; else, fn = fieldnames(outTuned); end
has = @(n) ismember(n, fn);

if has('Rotor1_RPM')
    t   = outTuned.Rotor1_RPM.Time(:);
    rpm = [squeeze(outTuned.Rotor1_RPM.Data(:)) squeeze(outTuned.Rotor2_RPM.Data(:)) ...
           squeeze(outTuned.Rotor3_RPM.Data(:)) squeeze(outTuned.Rotor4_RPM.Data(:))];
    omega = rpm*2*pi/60;
elseif has('UAV_State')
    rp = outTuned.UAV_State.RotorParameters;  t = rp.w1.Time(:);
    omega = [squeeze(rp.w1.Data(:)) squeeze(rp.w2.Data(:)) squeeze(rp.w3.Data(:)) squeeze(rp.w4.Data(:))];
else
    error('No rotor speed in outTuned (Rotor#_RPM or UAV_State.RotorParameters.w#).');
end

% torque: logged EM torque > logged drag torque > aero-drag proxy
if has('Motor1_Torque')
    Te = [squeeze(outTuned.Motor1_Torque.Data(:)) squeeze(outTuned.Motor2_Torque.Data(:)) ...
          squeeze(outTuned.Motor3_Torque.Data(:)) squeeze(outTuned.Motor4_Torque.Data(:))];
elseif has('Rotor1_Drag_Tq')
    Te = zeros(numel(t),4);
    for m = 1:4
        dts = outTuned.(sprintf('Rotor%d_Drag_Tq',m));
        Te(:,m) = interp1(dts.Time(:), abs(squeeze(dts.Data(:))), t, 'linear', 'extrap');
    end
elseif exist('uavParams','var')
    Cq=uavParams.rotor.Cq; Aar=uavParams.geom.RotorArea; Rpr=uavParams.geom.PropDiameter/2;
    Te = Cq*1.225*Aar*(omega*Rpr).^2*Rpr;
    warning('No logged torque; using aero-drag proxy (underestimates during acceleration).');
else
    error('No motor torque logged and no uavParams for the drag fallback.');
end

nM = size(omega,2);

%% ---- full-mission envelope ----
Iph_peak = abs(Te / Kt);                 % phase peak (= iq, id=0)
Iph_rms  = Iph_peak / sqrt(2);

figure('Color','w','Name','Phase-current envelope');
plot(t, Iph_rms, 'LineWidth',1.6); grid on; hold on;
plot(t, Iph_peak, '--', 'LineWidth',1.0);
xlabel('Time (s)'); ylabel('Phase current (A)'); legend('RMS','Peak');
title('Reconstructed phase-current envelope');

fprintf('\n=== Reconstructed currents (Kt = %.4g Nm/A) ===\n', Kt);
for m = 1:nM
    fprintf('  Motor %d:  peak %.0f A,  RMS %.0f A\n', m, max(Iph_peak(:,m)), max(Iph_rms(:,m)));
end

%% ---- fine waveform inside WIN ----
t1 = max(WIN(1), t(1));  t2 = min(WIN(2), t(end));
if t2 <= t1, error('WIN=[%g %g] outside sim time [%g %g].', WIN(1),WIN(2),t(1),t(end)); end
tf   = (t1:dt_fine:t2).';
om_f = interp1(t, omega(:,motorToPlot), tf, 'linear', 'extrap');
Te_f = interp1(t, Te(:,motorToPlot),    tf, 'linear', 'extrap');
theta_e = cumtrapz(tf, p*om_f);
iq_f    = Te_f / Kt;
ia = -iq_f.*sin(theta_e);
ib = -iq_f.*sin(theta_e - 2*pi/3);
ic = -iq_f.*sin(theta_e + 2*pi/3);

figure('Color','w','Name','3-phase waveform');
plot(tf, ia, tf, ib, tf, ic, 'LineWidth',1.2); grid on;
xlabel('Time (s)'); ylabel('Phase current (A)'); legend('i_a','i_b','i_c');
title(sprintf('Motor %d 3-phase currents  [%g, %g] s @ %g \\mus', motorToPlot, t1, t2, dt_fine*1e6));

recon.t=tf; recon.ia=ia; recon.ib=ib; recon.ic=ic; recon.iq=iq_f; recon.theta_e=theta_e; recon.motor=motorToPlot;
fprintf('  Fine waveform on [%g, %g] s @ %g us (%d pts). See ''recon''.\n', t1, t2, dt_fine*1e6, numel(tf));
