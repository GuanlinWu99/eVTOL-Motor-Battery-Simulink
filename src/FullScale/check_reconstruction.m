W='/Volumes/PortableSSD/files/maltab/kiat_pmsm'; cd(W);
L=load([W '/recon_validation.mat']); V=L.V;
Kt=V.Kt; Jm=V.Jm; Bm=V.Bm;
fprintf('Kt=%.3f Nm/A, Jm=%.4g kg*m^2, Bm=%.4g\n\n', Kt, Jm, Bm);

t = V.t_det(:);
idq = squeeze(V.idq); if size(idq,1)~=numel(t), idq=idq.'; end
iq_det = idq(:,2:2:end);                              % iq of the 4 motors

drag = interp1(V.t_drg(:), V.drag, t, 'linear','extrap');
rpm  = interp1(V.t_rpm(:), V.rpm,  t, 'linear','extrap');
w    = rpm*2*pi/60;
iq_rec = abs(drag)/Kt;                                % section 9 reconstruction

% settled window: rotor speed within 0.5 % of its final value
m=1; ref=rpm(end,m); settled = t > t(find(abs(rpm(:,m)-ref)>0.005*ref,1,'last'));
fprintf('settled from t=%.3f s (%.0f %% of the window)\n', t(find(settled,1)), 100*mean(settled));

lab={'whole window','settled only'};
for c=1:2
    sel = (c==1) | settled;  if c==2, sel=settled; end
    e = iq_rec(sel,:)-iq_det(sel,:);
    rel = 100*abs(e)./max(abs(iq_det(sel,:)),1);
    fprintf('\n--- %s ---\n', lab{c});
    fprintf('  detailed iq   mean %.1f A, max %.1f A\n', mean(iq_det(sel,1)), max(iq_det(sel,1)));
    fprintf('  reconstructed mean %.1f A, max %.1f A\n', mean(iq_rec(sel,1)), max(iq_rec(sel,1)));
    fprintf('  error  mean %+.2f A (%.3f %%), rms %.2f A, max %.1f A (%.2f %%)\n', ...
        mean(e(:,1)), mean(rel(:,1)), rms(e(:,1)), max(abs(e(:,1))), max(rel(:,1)));
end

% where the error is worst, and the torque balance that explains it
[~,iw]=max(abs(iq_rec(:,1)-iq_det(:,1)));
dw = gradient(w(:,1), t);
fprintf('\n--- worst point, motor 1 ---\n');
fprintf('  t=%.3f s, rotor %.0f rpm, dw/dt=%.0f rad/s^2\n', t(iw), rpm(iw,1), dw(iw));
fprintf('  detailed iq=%.1f A -> Te=%.0f Nm\n', iq_det(iw,1), Kt*iq_det(iw,1));
fprintf('  drag=%.0f Nm, Jm*dw/dt=%.0f Nm, Bm*w=%.1f Nm, sum=%.0f Nm\n', ...
    abs(drag(iw,1)), Jm*dw(iw), Bm*w(iw,1), abs(drag(iw,1))+Jm*dw(iw)+Bm*w(iw,1));
fprintf('  reconstruction misses the inertia term, so it reads %.1f A low\n', iq_det(iw,1)-iq_rec(iw,1));

% steady-state endpoint, the figure section 9 should quote
fprintf('\n--- endpoint (t=%.2f s) ---\n', t(end));
fprintf('  detailed iq=%.2f A, reconstructed=%.2f A, error %+.2f A (%.3f %%)\n', ...
    iq_det(end,1), iq_rec(end,1), iq_rec(end,1)-iq_det(end,1), ...
    100*(iq_rec(end,1)-iq_det(end,1))/iq_det(end,1));
