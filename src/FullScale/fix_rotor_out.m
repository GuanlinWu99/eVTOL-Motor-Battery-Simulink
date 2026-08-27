%% fix_rotor_out.m - drive Rotor Assembly speed (Mot_RPM_OUT_1..4 had no writer)
% Connect Gain4-7 (motor rpm / RPMMAX, normalized) to new Goto Mot_RPM_OUT_x.
m='VTOLDynamics'; load_system(m);
P=[m '/Force and Moments/Propulsion'];
map={'Gain4','Mot_RPM_OUT_1'; 'Gain5','Mot_RPM_OUT_2'; 'Gain6','Mot_RPM_OUT_3'; 'Gain7','Mot_RPM_OUT_4'};
for i=1:4
  gn=map{i,1}; tag=map{i,2};
  ex=find_system(P,'SearchDepth',1,'BlockType','Goto','GotoTag',tag);   % drop dup if any
  for k=1:numel(ex), delete_block(ex{k}); end
  gp=get_param([P '/' gn],'Position');
  gb=[P '/OUT_' num2str(i)];
  add_block('simulink/Signal Routing/Goto',gb,'GotoTag',tag,'TagVisibility','local', ...
      'Position',[gp(3)+40 gp(2) gp(3)+110 gp(4)]);
  ph=get_param([P '/' gn],'PortHandles'); lh=get_param(ph.Outport(1),'Line');   % clear stale line
  if lh~=-1 && ishandle(lh), delete_line(lh); end
  add_line(P,[gn '/1'],['OUT_' num2str(i) '/1'],'autorouting','on');
  fprintf('  %s -> Goto %s\n', gn, tag);
end
save_system(m);
disp('DONE: Rotor Assembly now gets rotor speed. Re-run startup.');
