classdef exampleHelperTiltRotorAnimator < matlab.System
% EXAMPLEHELPERTILTROTORANIMATOR class for animating tiltorotor.

% Copyright 2023 The MathWorks, Inc.

    properties (Access = private)
        HFigure
        HAx
        Body
        RotorA
        RotorB
        RotorC
        RotorD
        MeshBody
    end

    properties (Nontunable)
        SampleTime = 0.05
    end

    methods (Access = protected)
        function stepImpl(obj, position, orientation, rotorAngles)
            %

            obj.Body.Matrix = trvec2tform(position)*quat2tform(orientation);
            obj.RotorA.Matrix = trvec2tform([-40,50 0])*eul2tform([0 rotorAngles(1) 0]);
            obj.RotorB.Matrix = trvec2tform([-40,-50 0])*eul2tform([0 rotorAngles(2) 0]);
            obj.RotorC.Matrix = trvec2tform([22,-50 0])*eul2tform([0 rotorAngles(3) 0]);
            obj.RotorD.Matrix = trvec2tform([22,50 0])*eul2tform([0 rotorAngles(4) 0]);
        end

        function setupImpl(obj)
            % Perform one-time calculations, such as computing constants

            obj.HFigure = figure();

            %% Set rotor angles here
            rotorA=0;
            rotorB=0;
            rotorC=0;
            rotorD=0;
            %% Set aircraft orientation
            roll=0;
            pitch=0;
            yaw=0;

            %%
            mesh=uav.internal.util.readMeshData('TiltRotorFrame.stl');
            disc=uav.internal.util.readMeshData('disc.stl');
            disc.Vertices(:,1)=disc.Vertices(:,1)-mean(disc.Vertices(:,1));
            disc.Vertices(:,2)=disc.Vertices(:,2)-mean(disc.Vertices(:,2));
            disc.Vertices(:,3)=disc.Vertices(:,3)-mean(disc.Vertices(:,3));

            obj.HAx=newplot(obj.HFigure);
            %% Plot aircraft
            hold(obj.HAx, 'on');
            axis(obj.HAx, 'equal');
            view(obj.HAx,3)
            %% Plot motors.
            MotorA=makehgtform('translate',[-40,50 0],'yrotate',rotorA);
            MotorB=makehgtform('translate',[-40,-50 0],'yrotate',rotorB);
            MotorC=makehgtform('translate',[22,-50 0],'yrotate',rotorC);
            MotorD=makehgtform('translate',[22,50 0],'yrotate',rotorD);
            AircraftRot=makehgtform('xrotate',roll,'yrotate',pitch,'zrotate',yaw);
            MeshRot=makehgtform('xrotate',pi);
            inertia = hgtransform('Matrix', eul2tform([0 0 pi]), 'Parent', obj.HAx);
            obj.Body=hgtransform('Matrix',AircraftRot,'Parent',inertia);
            plot3([0 100], [0 0], [0 0], 'r', 'Parent', obj.Body,'LineWidth',2);
            plot3([0 0], [0 100], [0 0], 'g', 'Parent', obj.Body,'LineWidth',2);
            plot3([0 0], [0 0], [0 100], 'b', 'Parent', obj.Body,'LineWidth',2);
            obj.MeshBody=hgtransform('Matrix',eul2tform([pi 0 pi]),'Parent',obj.Body);
            obj.RotorA = hgtransform('Parent',obj.MeshBody);
            obj.RotorB = hgtransform('Parent',obj.MeshBody);
            obj.RotorC = hgtransform('Parent',obj.MeshBody);
            obj.RotorD = hgtransform('Parent',obj.MeshBody);
            set(obj.RotorA,'Matrix',MotorA);
            set(obj.RotorB,'Matrix',MotorB);
            set(obj.RotorC,'Matrix',MotorC);
            set(obj.RotorD,'Matrix',MotorD);
            %Aircraft
            patch('Faces',mesh.Faces,'Vertices',mesh.Vertices,'FaceColor',[0.5 0.5 0.5],'EdgeColor','none','FaceAlpha',0.5,'Parent',obj.MeshBody)
            %Props
            patch('Faces',disc.Faces,'Vertices',disc.Vertices,'FaceColor','[0 0 1]','EdgeColor','none','Parent',obj.RotorA,'FaceAlpha',0.2)
            patch('Faces',disc.Faces,'Vertices',disc.Vertices,'FaceColor','[0 0 1]','EdgeColor','none','Parent',obj.RotorB,'FaceAlpha',0.2)
            patch('Faces',disc.Faces,'Vertices',disc.Vertices,'FaceColor','[0 1 0]','EdgeColor','none','Parent',obj.RotorC,'FaceAlpha',0.2)
            patch('Faces',disc.Faces,'Vertices',disc.Vertices,'FaceColor','[0 1 0]','EdgeColor','none','Parent',obj.RotorD,'FaceAlpha',0.2)
            hold(obj.HAx, 'off');

        end

        function releaseImpl(obj)
            % Release resources, such as file handles

            delete(obj.HFigure);
        end

        function sts = getSampleTimeImpl(obj)
            % Define sample time type and parameters

            % Example: specify discrete sample time
            sts = obj.createSampleTime("Type", "Discrete", ...
                "SampleTime", obj.SampleTime);
        end
    end

    methods (Access = protected, Static)
        function simMode = getSimulateUsingImpl
            % Return only allowed simulation mode in System block dialog
            simMode = "Interpreted execution";
        end

        function flag = showSimulateUsingImpl
            % Return false if simulation mode hidden in System block dialog
            flag = false;
        end
    end
end

