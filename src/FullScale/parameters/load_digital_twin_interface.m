%=========================================================================%
% Define buses for digital twin module                                    %
% author(s): minhyun                                                      %
% description:                                                            %
%=========================================================================%

function load_digital_twin_interface()
%exampleHelperDefineDigitalTwinInterface define digital twin interface for
%VTOL.

% Copyright 2023 The MathWorks, Inc.
%% Design bus - Fixed Wing Control
clear elems
elems(1) = Simulink.BusElement;
elems(1).Name = 'dA';
elems(2) = Simulink.BusElement;
elems(2).Name = 'dE';
elems(3) = Simulink.BusElement;
elems(3).Name = 'dR';
elems(4) = Simulink.BusElement;
elems(4).Name = 'dT';
FixedWingCtrlBus = Simulink.Bus;
FixedWingCtrlBus.Elements = elems;
%% Design bus - Rotor Control
clear elems
elems(1) = Simulink.BusElement;
elems(1).Name = 'w1';
elems(2) = Simulink.BusElement;
elems(2).Name = 'w2';
elems(3) = Simulink.BusElement;
elems(3).Name = 'w3';
elems(4) = Simulink.BusElement;
elems(4).Name = 'w4';
elems(5) = Simulink.BusElement;
elems(5).Name = 'Tilt1';
elems(6) = Simulink.BusElement;
elems(6).Name = 'Tilt2';
RotorCntrlBus = Simulink.Bus;
RotorCntrlBus.Elements = elems;                 %... 오타 수정
%% Assign into base workspace
assignin('base',"FixedWingCtrlBus",FixedWingCtrlBus);
assignin('base',"RotorCtrlBus",RotorCntrlBus);
end