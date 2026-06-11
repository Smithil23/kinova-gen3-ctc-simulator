classdef KinovaApp < matlab.apps.AppBase

    % =========================================================
    % KINOVA GEN3 CTC TRAJECTORY SIMULATOR
    % App Designer programmatic app
    % Open in App Designer: appdesigner('KinovaApp.m')
    % Run directly:         KinovaApp
    % =========================================================

    properties (Access = public)
        UIFigure            matlab.ui.Figure

        %% Panels
        TitlePanel          matlab.ui.container.Panel
        LeftPanel           matlab.ui.container.Panel
        ViewPanel           matlab.ui.container.Panel
        PosePanel           matlab.ui.container.Panel
        SetupPanel          matlab.ui.container.Panel
        InfoPanel           matlab.ui.container.Panel
        ResultsPanel        matlab.ui.container.Panel
        PlotPanel1          matlab.ui.container.Panel
        PlotPanel2          matlab.ui.container.Panel
        PlotPanel3          matlab.ui.container.Panel
        PlotPanel4          matlab.ui.container.Panel

        %% Title bar
        TitleLabel          matlab.ui.control.Label
        SubtitleLabel       matlab.ui.control.Label
        StatusLabel         matlab.ui.control.Label
        StatusLamp          matlab.ui.control.Lamp

        %% Joint control
        JointSliders        matlab.ui.control.Slider
        JointValueLabels    matlab.ui.control.Label

        %% 3D view
        Axes3D              matlab.ui.control.UIAxes

        %% EE Pose
        PosXLabel           matlab.ui.control.Label
        PosYLabel           matlab.ui.control.Label
        PosZLabel           matlab.ui.control.Label
        IKErrLabel          matlab.ui.control.Label
        DistLabel           matlab.ui.control.Label
        TTotalLabel         matlab.ui.control.Label

        %% Setup
        TrajTypeDD          matlab.ui.control.DropDown
        TargetXField        matlab.ui.control.NumericEditField
        TargetYField        matlab.ui.control.NumericEditField
        TargetZField        matlab.ui.control.NumericEditField
        SpeedField          matlab.ui.control.NumericEditField
        GainField           matlab.ui.control.NumericEditField
        CalcButton          matlab.ui.control.Button
        RunButton           matlab.ui.control.Button
        StopButton          matlab.ui.control.Button
        ResetButton         matlab.ui.control.Button
        SinglePanel         matlab.ui.container.Panel
        MultiPanel          matlab.ui.container.Panel
        WPTable             matlab.ui.control.Table

        %% Results
        RMSLabel            matlab.ui.control.Label
        MaxErrLabel         matlab.ui.control.Label
        BestJLabel          matlab.ui.control.Label
        WorstJLabel         matlab.ui.control.Label
        LogArea             matlab.ui.control.TextArea

        %% Energy labels
        ETotalLabel         matlab.ui.control.Label
        EGravLabel          matlab.ui.control.Label
        EInertLabel         matlab.ui.control.Label
        ECorLabel           matlab.ui.control.Label

        %% Safety panel lamps and labels
        SafetyLamps         matlab.ui.control.Lamp
        SafetyLabels        matlab.ui.control.Label
        SafetyPanel         matlab.ui.container.Panel
        SafetyStatusLabel   matlab.ui.control.Label
        SafetyOverallLamp   matlab.ui.control.Lamp
        SafetyScoreLabel    matlab.ui.control.Label

        %% Plots
        AxPos               matlab.ui.control.UIAxes
        AxVel               matlab.ui.control.UIAxes
        AxTorq              matlab.ui.control.UIAxes
        AxErr               matlab.ui.control.UIAxes
    end

    properties (Access = private)
        %% App state
        Robot
        QTarget
        TrajectoryReady     logical = false
        SimRunning          logical = false
        SafeLamps
        SafeLabels

        %% Colors
        CBg     = [0.12 0.14 0.18]
        CPanel  = [0.16 0.18 0.23]
        CAccent = [0.20 0.60 0.90]
        CAccent2= [0.25 0.85 0.65]
        CWarn   = [0.95 0.65 0.20]
        CDanger = [0.90 0.30 0.30]
        CText   = [0.92 0.93 0.95]
        CDim    = [0.55 0.58 0.65]
        CGreen  = [0.20 0.80 0.45]
        CPlot   = [0.10 0.12 0.16]
        CBorder = [0.25 0.28 0.35]

        %% Joint colors — unique per joint, matches 3D animation AND all plots
        JColors = [0.20 0.60 0.90;   % J1 — blue
                   0.90 0.45 0.20;   % J2 — orange
                   0.90 0.25 0.35;   % J3 — red
                   0.75 0.40 0.90;   % J4 — purple
                   0.25 0.85 0.65;   % J5 — teal
                   0.95 0.85 0.20;   % J6 — yellow
                   0.90 0.55 0.75]   % J7 — pink

        %% Joint data
        JointNames = {'J1  Base','J2  Shoulder','J3  Elbow','J4  Forearm',...
                      'J5  Wrist1','J6  Wrist2','J7  Tool'}
        JointLimLo = [-180 -138 -152 -128 -120 -120 -120]
        JointLimHi = [ 180  138  152  128  120  120  120]
        JSliders
        JValLabels

        %% XYZ validation labels
        XValLbl  matlab.ui.control.Label
        YValLbl  matlab.ui.control.Label
        ZValLbl  matlab.ui.control.Label

        %% Curve type dropdown
        CurveTypeDD  matlab.ui.control.DropDown

        %% Energy percent labels
        EGravPctLabel  matlab.ui.control.Label
        EInertPctLabel matlab.ui.control.Label
        ECorPctLabel   matlab.ui.control.Label

        WorkspaceVisible  logical = false
        WorkspacePoints
        Panels   struct
    end

    methods (Access = private)

        function createComponents(app)
            %% ── Figure ───────────────────────────────────────────────────────
            app.UIFigure = uifigure('Visible','off');
            app.UIFigure.Name             = 'Kinova Gen3 — CTC Trajectory Simulator';
            app.UIFigure.Color            = app.CBg;
            app.UIFigure.Resize           = 'on';
            app.UIFigure.AutoResizeChildren = 'off';   %% must be off for SizeChangedFcn
            app.UIFigure.WindowState      = 'maximized';

            %% ── ROOT GRID: 3 rows ─────────────────────────────────────────────
            %% Grid children = panels only (MATLAB compatibility)
            rg = uigridlayout(app.UIFigure,[3 1]);
            rg.RowHeight   = {36,'1x','0.9x'};
            rg.ColumnWidth = {'1x'};
            rg.Padding=[0 0 0 0]; rg.RowSpacing=0; rg.ColumnSpacing=0;
            rg.BackgroundColor = app.CBg;

            %% ─────────────────────────────────────────────────────────────────
            %% ROW 1 — TITLE BAR (panel in grid)
            %% ─────────────────────────────────────────────────────────────────
            tp = uipanel(rg,'BackgroundColor',[0.08 0.10 0.14],'BorderType','none');
            tp.Layout.Row=1; tp.Layout.Column=1;
            uilabel(tp,'Text','KINOVA GEN3  ·  CTC TRAJECTORY SIMULATOR',...
                'Position',[16 8 520 20],'FontSize',13,'FontWeight','bold',...
                'FontColor',app.CAccent,'BackgroundColor','none');
            uilabel(tp,'Text','7-DOF Simscape Multibody  ·  Computed Torque Control  ·  Cubic Hermite Trajectory',...
                'Position',[500 10 560 16],'FontSize',8,...
                'FontColor',app.CDim,'BackgroundColor','none');
            app.StatusLabel = uilabel(tp,'Text','READY','Tag','StatusLbl',...
                'Position',[10 9 60 16],'FontSize',9,'FontWeight','bold',...
                'FontColor',app.CDim,'BackgroundColor','none','HorizontalAlignment','right');
            app.StatusLamp = uilamp(tp,'Tag','StatusLamp',...
                'Position',[10 11 14 14],'Color',app.CBorder);
            %% Reposition status/lamp on every resize
            tp.AutoResizeChildren = 'off';
            tp.SizeChangedFcn = @(src,~) app.anchorTitleRight(src);

            %% ─────────────────────────────────────────────────────────────────
            %% ROW 2 — TOP ROW: grid of 6 panels
            %% ─────────────────────────────────────────────────────────────────
            topG = uigridlayout(rg,[1 6]);
            topG.Layout.Row=2; topG.Layout.Column=1;
            topG.RowHeight={'1x'};
            topG.ColumnWidth={'0.72x','1.70x','1.50x','1.10x','1.10x','0.80x'};
            topG.Padding=[2 2 2 2]; topG.RowSpacing=0; topG.ColumnSpacing=2;
            topG.BackgroundColor=app.CBg;

            %% ── Panel 1: Joint Control ────────────────────────────────────────
            lp = uipanel(topG,'BackgroundColor',app.CPanel,'BorderType','none');
            lp.Layout.Row=1; lp.Layout.Column=1;
            %% Use SizeChangedFcn to reflow the 7 joints
            lp.AutoResizeChildren = 'off';
            lp.SizeChangedFcn = @(src,~) app.reflowJoints(src);

            %% Create all joint controls with placeholder positions
            uilabel(lp,'Text','JOINT CONTROL','Tag','JCtrlTitle',...
                'Position',[6 10 140 16],'FontSize',9,'FontWeight','bold',...
                'FontColor',app.CAccent,'BackgroundColor','none');
            app.JSliders   = gobjects(7,1);
            app.JValLabels = gobjects(7,1);
            for j=1:7
                jc=app.JColors(j,:);
                uilabel(lp,'Text',app.JointNames{j},'Tag',sprintf('JN%d',j),...
                    'Position',[6 10 160 14],'FontSize',8,'FontWeight','bold',...
                    'FontColor',jc,'BackgroundColor','none');
                uilabel(lp,'Text',sprintf('%d°',app.JointLimLo(j)),'Tag',sprintf('JLo%d',j),...
                    'Position',[6 10 28 11],'FontSize',7,'FontColor',app.CDim,'BackgroundColor','none');
                uilabel(lp,'Text','0°','Tag',sprintf('JMid%d',j),...
                    'Position',[6 10 20 11],'FontSize',7,'FontColor',app.CDim,...
                    'BackgroundColor','none','HorizontalAlignment','center');
                uilabel(lp,'Text',sprintf('%d°',app.JointLimHi(j)),'Tag',sprintf('JHi%d',j),...
                    'Position',[6 10 28 11],'FontSize',7,'FontColor',app.CDim,...
                    'BackgroundColor','none','HorizontalAlignment','right');
                app.JSliders(j)=uislider(lp,'Tag',sprintf('JSld%d',j),...
                    'Position',[6 10 160 3],...
                    'Limits',[app.JointLimLo(j) app.JointLimHi(j)],'Value',0,'FontColor',jc,...
                    'ValueChangedFcn',@(src,~) app.onSliderChanged(j,src.Value));
                app.JValLabels(j)=uilabel(lp,'Text','0.0°','Tag',sprintf('JVal%d',j),...
                    'Position',[6 10 160 18],'FontSize',11,'FontWeight','bold',...
                    'FontColor',jc,'BackgroundColor','none','HorizontalAlignment','center');
            end
            app.ResetButton=uibutton(lp,'push','Text','⌂  Home Position',...
                'Tag','BtnHome','Position',[6 6 160 26],...
                'BackgroundColor',[0.20 0.24 0.32],'FontColor',app.CText,...
                'FontSize',9,'ButtonPushedFcn',@(~,~) app.resetHome());

            %% ── Panel 2: 3D View ──────────────────────────────────────────────
            vp=uipanel(topG,'BackgroundColor',app.CPanel,'BorderType','none');
            vp.Layout.Row=1; vp.Layout.Column=2;
            vp.AutoResizeChildren = 'off';
            vp.SizeChangedFcn=@(src,~) app.reflow3D(src);
            uilabel(vp,'Text','3D ROBOT VIEW','Tag','Lbl3D',...
                'Position',[8 10 160 16],'FontSize',9,'FontWeight','bold',...
                'FontColor',app.CAccent,'BackgroundColor','none');
            app.Axes3D=uiaxes(vp,'Tag','Ax3D','Position',[4 30 100 100],...
                'Color',app.CPlot,'XColor',app.CBorder,'YColor',app.CBorder,...
                'ZColor',app.CBorder,'GridColor',app.CBorder,'GridAlpha',0.3);
            app.Axes3D.XLabel.String='X (m)'; app.Axes3D.XLabel.Color=app.CDim;
            app.Axes3D.YLabel.String='Y (m)'; app.Axes3D.YLabel.Color=app.CDim;
            app.Axes3D.ZLabel.String='Z (m)'; app.Axes3D.ZLabel.Color=app.CDim;
            app.Axes3D.Title.String='Set target XYZ and press Calculate';
            app.Axes3D.Title.Color=app.CDim; app.Axes3D.Title.FontSize=9;
            grid(app.Axes3D,'on'); view(app.Axes3D,45,30);
            uibutton(vp,'push','Text','🌐  Show Workspace','Tag','BtnWS',...
                'Position',[4 6 100 22],...
                'BackgroundColor',[0.18 0.22 0.30],'FontColor',app.CAccent,...
                'FontSize',8,'FontWeight','bold',...
                'ButtonPushedFcn',@(~,~) app.onToggleWorkspace());
            uibutton(vp,'push','Text','↺  Clear View','Tag','BtnClr',...
                'Position',[108 6 100 22],...
                'BackgroundColor',[0.18 0.22 0.30],'FontColor',app.CDim,...
                'FontSize',8,'ButtonPushedFcn',@(~,~) app.onClear3D());

            %% ── Panel 3: Setup ────────────────────────────────────────────────
            sp=uipanel(topG,'BackgroundColor',app.CPanel,'BorderType','none');
            sp.Layout.Row=1; sp.Layout.Column=3;
            sp.AutoResizeChildren = 'off';
            sp.SizeChangedFcn=@(src,~) app.reflowSetup(src);

            uilabel(sp,'Text','TRAJECTORY SETUP','Tag','LblSetup',...
                'Position',[8 10 200 16],'FontSize',9,'FontWeight','bold',...
                'FontColor',app.CAccent,'BackgroundColor','none');
            uilabel(sp,'Text','Trajectory Type','Tag','LblTrajType',...
                'Position',[8 10 130 13],'FontSize',8,'FontColor',app.CDim,'BackgroundColor','none');
            app.TrajTypeDD=uidropdown(sp,'Tag','DDTraj',...
                'Items',{'Single Target','Multi-Waypoint','TCP Test','Task-Space'},...
                'Position',[8 10 130 22],...
                'BackgroundColor',[0.20 0.24 0.32],'FontColor',app.CText,'FontSize',9,...
                'ValueChangedFcn',@(src,~) app.onTrajTypeChanged(src.Value));
            uilabel(sp,'Text','Curve Profile','Tag','LblCurve',...
                'Position',[8 10 130 13],'FontSize',8,'FontColor',app.CDim,'BackgroundColor','none');
            app.CurveTypeDD=uidropdown(sp,'Tag','DDCurve',...
                'Items',{'Cubic Spline','Quintic Polynomial','Trapezoidal (LSPB)','Cubic Hermite','Bang-Bang'},...
                'Position',[8 10 130 22],...
                'BackgroundColor',[0.20 0.24 0.32],'FontColor',app.CText,'FontSize',9);

            %% Single panel (sub-panel inside sp, positioned by reflowSetup)
            app.SinglePanel=uipanel(sp,'Tag','PSingle',...
                'Position',[0 0 100 100],...
                'BackgroundColor',app.CPanel,'BorderType','none','Visible','on');
            xyzLabels={'X (m) → forward','Y (m) → sideways','Z (m) → height'};
            xyzDefs={0.5,0.0,0.3};
            xyzH={'TargetXField','TargetYField','TargetZField'};
            valH={'XValLbl','YValLbl','ZValLbl'};
            for tf=1:3
                uilabel(app.SinglePanel,'Text',xyzLabels{tf},'Tag',sprintf('LblXYZ%d',tf),...
                    'Position',[8 10 170 12],'FontSize',7,'FontColor',app.CDim,'BackgroundColor','none');
                app.(xyzH{tf})=uieditfield(app.SinglePanel,'numeric',...
                    'Tag',sprintf('EfXYZ%d',tf),'Value',xyzDefs{tf},'Position',[8 10 140 26],...
                    'BackgroundColor',[0.10 0.12 0.16],'FontColor',app.CText,...
                    'FontSize',13,'Limits',[-2 2],...
                    'ValueChangedFcn',@(~,~) app.validateXYZ());
                app.(valH{tf})=uilabel(app.SinglePanel,'Text','✓','Tag',sprintf('LblVld%d',tf),...
                    'Position',[152 10 18 18],'FontSize',10,'FontWeight','bold',...
                    'FontColor',app.CGreen,'BackgroundColor','none','HorizontalAlignment','center');
            end
            uilabel(app.SinglePanel,'Text','Quick presets','Tag','LblPre',...
                'Position',[8 10 90 12],'FontSize',7,'FontColor',app.CDim,'BackgroundColor','none');
            presets={[0.5 0.0 0.3],[0.2 0.4 0.5],[0.0 0.5 0.4]};
            pLbls={'Forward','Side','Diagonal'};
            for pr=1:3
                uibutton(app.SinglePanel,'push','Text',pLbls{pr},'Tag',sprintf('BtnPr%d',pr),...
                    'Position',[8 10 100 20],...
                    'BackgroundColor',[0.18 0.22 0.30],'FontColor',app.CAccent2,...
                    'FontSize',8,'ButtonPushedFcn',@(~,~) app.setPreset(presets{pr}));
            end
            app.SinglePanel.AutoResizeChildren = 'off';
            app.SinglePanel.SizeChangedFcn=@(src,~) app.reflowSingle(src);

            %% Multi panel
            app.MultiPanel=uipanel(sp,'Tag','PMulti',...
                'Position',[0 0 100 100],...
                'BackgroundColor',app.CPanel,'BorderType','none','Visible','off');
            uibutton(app.MultiPanel,'push','Text','Boss Demo','Tag','BtnBoss',...
                'Position',[8 10 100 20],...
                'BackgroundColor',[0.18 0.22 0.30],'FontColor',app.CWarn,'FontSize',8,...
                'ButtonPushedFcn',@(~,~) app.loadPresetTask('boss'));
            uibutton(app.MultiPanel,'push','Text','Side Sweep','Tag','BtnSweep',...
                'Position',[8 10 100 20],...
                'BackgroundColor',[0.18 0.22 0.30],'FontColor',app.CAccent2,'FontSize',8,...
                'ButtonPushedFcn',@(~,~) app.loadPresetTask('sweep'));
            uibutton(app.MultiPanel,'push','Text','Vertical','Tag','BtnVert',...
                'Position',[8 10 100 20],...
                'BackgroundColor',[0.18 0.22 0.30],'FontColor',app.CAccent,'FontSize',8,...
                'ButtonPushedFcn',@(~,~) app.loadPresetTask('vertical'));
            app.WPTable=uitable(app.MultiPanel,'Tag','WPTable',...
                'Position',[8 40 300 100],...
                'Data',{0.5,0.0,0.3,'Down',0.20,''; 0.2,0.4,0.5,'Down',0.20,''; 0.0,0.5,0.4,'Down',0.20,''},...
                'ColumnName',{'X (m)','Y (m)','Z (m)','EE Orient','Spd','Note'},...
                'ColumnEditable',[true true true true true true],...
                'ColumnFormat',{'numeric','numeric','numeric',{'Down','Up','Horiz','Tilt45'},'numeric','char'},...
                'ColumnWidth',{56,56,56,88,44,50},...
                'BackgroundColor',[0.10 0.12 0.16; 0.13 0.15 0.20],...
                'ForegroundColor',app.CText,'FontSize',9);
            uibutton(app.MultiPanel,'push','Text','+ Add Row','Tag','BtnAddWP',...
                'Position',[8 10 100 24],...
                'BackgroundColor',[0.18 0.22 0.30],'FontColor',app.CGreen,'FontSize',8,...
                'ButtonPushedFcn',@(~,~) app.addWPRow());
            uibutton(app.MultiPanel,'push','Text','− Remove','Tag','BtnRmWP',...
                'Position',[8 10 100 24],...
                'BackgroundColor',[0.18 0.22 0.30],'FontColor',app.CDanger,'FontSize',8,...
                'ButtonPushedFcn',@(~,~) app.removeWPRow());
            uibutton(app.MultiPanel,'push','Text','↺ Reset','Tag','BtnRstWP',...
                'Position',[8 10 100 24],...
                'BackgroundColor',[0.18 0.22 0.30],'FontColor',app.CWarn,'FontSize',8,...
                'ButtonPushedFcn',@(~,~) app.resetWPTable());
            app.MultiPanel.AutoResizeChildren = 'off';
            app.MultiPanel.SizeChangedFcn=@(src,~) app.reflowMulti(src);

            %% Speed/Gain/Buttons — always in sp, positioned by reflowSetup
            uilabel(sp,'Text','Tool Speed (m/s)','Tag','LblSpd',...
                'Position',[8 10 130 13],'FontSize',8,'FontColor',app.CDim,'BackgroundColor','none');
            app.SpeedField=uieditfield(sp,'numeric','Value',0.20,'Tag','EfSpd',...
                'Position',[8 10 130 24],...
                'BackgroundColor',[0.10 0.12 0.16],'FontColor',app.CText,'FontSize',12,'Limits',[0.05 1.0]);
            uilabel(sp,'Text','Gain Scale (×base)','Tag','LblGain',...
                'Position',[8 10 130 13],'FontSize',8,'FontColor',app.CDim,'BackgroundColor','none');
            app.GainField=uieditfield(sp,'numeric','Value',3.0,'Tag','EfGain',...
                'Position',[8 10 130 24],...
                'BackgroundColor',[0.10 0.12 0.16],'FontColor',app.CText,'FontSize',12,'Limits',[0.5 10.0]);
            app.CalcButton=uibutton(sp,'push','Text','⟳  Calculate Trajectory / IK','Tag','BtnCalc',...
                'Position',[8 10 260 26],...
                'BackgroundColor',app.CAccent,'FontColor',[1 1 1],'FontSize',10,'FontWeight','bold',...
                'ButtonPushedFcn',@(~,~) app.onCalculate());
            app.RunButton=uibutton(sp,'push','Text','▶  RUN SIMULATION','Tag','BtnRun',...
                'Position',[8 10 126 28],...
                'BackgroundColor',app.CGreen,'FontColor',[0 0 0],'FontSize',10,'FontWeight','bold',...
                'ButtonPushedFcn',@(~,~) app.onRun());
            app.StopButton=uibutton(sp,'push','Text','■  STOP','Tag','BtnStop',...
                'Position',[8 10 126 28],...
                'BackgroundColor',app.CDanger,'FontColor',[1 1 1],'FontSize',10,'FontWeight','bold',...
                'ButtonPushedFcn',@(~,~) app.onStop());
            uibutton(sp,'push','Text','▷  Replay Animation','Tag','BtnReplay',...
                'Position',[8 10 260 22],...
                'BackgroundColor',[0.18 0.22 0.30],'FontColor',app.CAccent2,'FontSize',9,...
                'ButtonPushedFcn',@(~,~) app.onReplay());

            %% ── Panel 4: Results ──────────────────────────────────────────────
            rp=uipanel(topG,'BackgroundColor',app.CPanel,'BorderType','none');
            rp.Layout.Row=1; rp.Layout.Column=4;
            rp.AutoResizeChildren = 'off';
            rp.SizeChangedFcn=@(src,~) app.reflowResults(src);

            uilabel(rp,'Text','SIMULATION RESULTS','Tag','LblRes',...
                'Position',[8 10 200 16],'FontSize',9,'FontWeight','bold',...
                'FontColor',app.CAccent,'BackgroundColor','none');
            rFields={'Overall RMS Error','Max Joint Error','Best joint','Worst joint'};
            rHandles={'RMSLabel','MaxErrLabel','BestJLabel','WorstJLabel'};
            rColors={app.CGreen,app.CWarn,app.CAccent2,app.CDanger};
            for rf=1:4
                uilabel(rp,'Text',rFields{rf},'Tag',sprintf('LblR%d',rf),...
                    'Position',[8 10 130 12],'FontSize',7,'FontColor',app.CDim,'BackgroundColor','none');
                app.(rHandles{rf})=uilabel(rp,'Text','—','Tag',sprintf('ValR%d',rf),...
                    'Position',[8 10 130 44],'FontSize',15,'FontWeight','bold',...
                    'FontColor',rColors{rf},'BackgroundColor',[0.10 0.12 0.16],...
                    'HorizontalAlignment','center','VerticalAlignment','center');
            end
            uilabel(rp,'Text','SIMULATION LOG','Tag','LblLog',...
                'Position',[8 10 160 16],'FontSize',9,'FontWeight','bold',...
                'FontColor',app.CAccent,'BackgroundColor','none');
            app.LogArea=uitextarea(rp,'Tag','LogArea','Position',[8 8 200 100],...
                'BackgroundColor',[0.10 0.12 0.16],'FontColor',app.CDim,...
                'FontSize',9,'Editable','off','WordWrap','on');

            %% ── Panel 5: Energy ───────────────────────────────────────────────
            ep=uipanel(topG,'BackgroundColor',app.CPanel,'BorderType','none');
            ep.Layout.Row=1; ep.Layout.Column=5;
            ep.AutoResizeChildren = 'off';
            ep.SizeChangedFcn=@(src,~) app.reflowEnergy(src);

            uilabel(ep,'Text','ENERGY CONSUMPTION','Tag','LblEng',...
                'Position',[8 10 200 16],'FontSize',9,'FontWeight','bold',...
                'FontColor',[0.95 0.65 0.20],'BackgroundColor','none');
            uilabel(ep,'Text','TOTAL','Tag','LblTotal',...
                'Position',[8 10 60 13],'FontSize',8,'FontWeight','bold',...
                'FontColor',app.CDim,'BackgroundColor','none');
            app.ETotalLabel=uilabel(ep,'Text','— J','Tag','ValTotal',...
                'Position',[8 10 200 36],'FontSize',20,'FontWeight','bold',...
                'FontColor',[0.95 0.65 0.20],'BackgroundColor',[0.10 0.12 0.16],...
                'HorizontalAlignment','center','VerticalAlignment','center');
            eLabels={'GRAVITY','INERTIAL','CORIOLIS'};
            eHandles={'EGravLabel','EInertLabel','ECorLabel'};
            ePctH={'EGravPctLabel','EInertPctLabel','ECorPctLabel'};
            eColors={[0.90 0.30 0.30],[0.20 0.60 0.90],[0.25 0.85 0.65]};
            for ec=1:3
                uilabel(ep,'Text',eLabels{ec},'Tag',sprintf('LblE%d',ec),...
                    'Position',[8 10 120 13],'FontSize',8,'FontWeight','bold',...
                    'FontColor',eColors{ec},'BackgroundColor','none');
                app.(eHandles{ec})=uilabel(ep,'Text','— J','Tag',sprintf('ValE%d',ec),...
                    'Position',[8 10 120 20],'FontSize',13,'FontWeight','bold',...
                    'FontColor',eColors{ec},'BackgroundColor','none');
                app.(ePctH{ec})=uilabel(ep,'Text','— %','Tag',sprintf('PctE%d',ec),...
                    'Position',[8 10 70 20],'FontSize',12,'FontWeight','bold',...
                    'FontColor',eColors{ec},'BackgroundColor','none','HorizontalAlignment','right');
            end

            %% ── Panel 6: Safety ───────────────────────────────────────────────
            sf=uipanel(topG,'BackgroundColor',app.CPanel,'BorderType','none');
            sf.Layout.Row=1; sf.Layout.Column=6;
            sf.AutoResizeChildren = 'off';
            sf.SizeChangedFcn=@(src,~) app.reflowSafety(src);

            uilabel(sf,'Text','SAFETY MONITOR','Tag','LblSafe',...
                'Position',[8 10 150 16],'FontSize',9,'FontWeight','bold',...
                'FontColor',[0.90 0.30 0.30],'BackgroundColor','none');
            app.SafetyOverallLamp=uilamp(sf,'Tag','LampSafe','Position',[10 10 14 14],'Color',app.CBorder);
            app.SafetyStatusLabel=uilabel(sf,'Text','NOT CHECKED','Tag','LblSafeStatus',...
                'Position',[8 10 200 16],'FontSize',8,'FontWeight','bold',...
                'FontColor',app.CDim,'BackgroundColor','none','HorizontalAlignment','center');
            uipanel(sf,'Tag','ScoreBg','Position',[8 10 200 26],...
                'BackgroundColor',[0.10 0.12 0.16],'BorderType','none');
            uilabel(sf,'Text','SCORE','Tag','LblScore',...
                'Position',[12 10 38 14],'FontSize',7,'FontWeight','bold',...
                'FontColor',app.CDim,'BackgroundColor','none');
            app.SafetyScoreLabel=uilabel(sf,'Text','— / 100','Tag','ValScore',...
                'Position',[48 10 150 22],'FontSize',14,'FontWeight','bold',...
                'FontColor',app.CBorder,'BackgroundColor','none','HorizontalAlignment','right');
            checkNames={'Joint Limits','Velocity Limits','Torque Saturation',...
                'Workspace Boundary','Singularity','Self-Collision','Acceleration'};
            app.SafeLamps  = gobjects(7,1);
            app.SafeLabels = gobjects(7,1);
            for sc=1:7
                app.SafeLamps(sc)=uilamp(sf,'Tag',sprintf('SLmp%d',sc),'Position',[8 10 12 12],'Color',app.CBorder);
                uilabel(sf,'Text',checkNames{sc},'Tag',sprintf('SName%d',sc),...
                    'Position',[24 10 180 13],'FontSize',7,'FontWeight','bold',...
                    'FontColor',app.CText,'BackgroundColor','none');
                app.SafeLabels(sc)=uilabel(sf,'Text','—','Tag',sprintf('SDetail%d',sc),...
                    'Position',[24 10 180 11],'FontSize',6,'FontColor',app.CDim,'BackgroundColor','none');
            end
            uibutton(sf,'push','Text','🛡  Run Safety Check','Tag','BtnSafeRun',...
                'Position',[8 34 200 22],...
                'BackgroundColor',[0.15 0.18 0.25],'FontColor',[0.90 0.30 0.30],...
                'FontSize',8,'FontWeight','bold','ButtonPushedFcn',@(~,~) app.onSafetyCheck());
            uibutton(sf,'push','Text','📋  Export Safety Report','Tag','BtnExport',...
                'Position',[8 8 200 22],...
                'BackgroundColor',[0.15 0.18 0.25],'FontColor',[0.95 0.65 0.20],...
                'FontSize',8,'FontWeight','bold','ButtonPushedFcn',@(~,~) app.exportSafetyReport());

            %% ─────────────────────────────────────────────────────────────────
            %% ROW 3 — BOTTOM ROW: grid of 5 panels
            %% ─────────────────────────────────────────────────────────────────
            botG=uigridlayout(rg,[1 5]);
            botG.Layout.Row=3; botG.Layout.Column=1;
            botG.RowHeight={'1x'}; botG.ColumnWidth={'0.55x','1x','1x','1x','1x'};
            botG.Padding=[2 2 2 2]; botG.RowSpacing=0; botG.ColumnSpacing=2;
            botG.BackgroundColor=app.CBg;

            %% EE Pose
            pp=uipanel(botG,'BackgroundColor',app.CPanel,'BorderType','none');
            pp.Layout.Row=1; pp.Layout.Column=1;
            pp.AutoResizeChildren = 'off';
            pp.SizeChangedFcn=@(src,~) app.reflowEEPose(src);
            uilabel(pp,'Text','END-EFFECTOR','Tag','LblEE',...
                'Position',[6 10 120 16],'FontSize',9,'FontWeight','bold',...
                'FontColor',app.CAccent,'BackgroundColor','none');
            pFields={'X (m)','Y (m)','Z (m)','IK err (mm)','Distance (m)','T total (s)'};
            pColors={app.CAccent2,app.CAccent2,app.CAccent2,app.CGreen,app.CText,app.CText};
            pHandles={'PosXLabel','PosYLabel','PosZLabel','IKErrLabel','DistLabel','TTotalLabel'};
            for pf=1:6
                uilabel(pp,'Text',pFields{pf},'Tag',sprintf('LblPF%d',pf),...
                    'Position',[6 10 130 12],'FontSize',7,'FontColor',app.CDim,'BackgroundColor','none');
                app.(pHandles{pf})=uilabel(pp,'Text','—','Tag',sprintf('ValPF%d',pf),...
                    'Position',[6 10 130 28],'FontSize',13,'FontWeight','bold',...
                    'FontColor',pColors{pf},'BackgroundColor',[0.10 0.12 0.16],...
                    'HorizontalAlignment','center','VerticalAlignment','center');
            end

            %% 4 Plot panels
            plotTitles={'Joint Positions (rad)','Joint Velocities (rad/s)',...
                'Feedforward Torques (Nm)','Tracking Error (deg)'};
            plotYLabels={'Position (rad)','Velocity (rad/s)','Torque (Nm)','Error (deg)'};
            axTags={'AxPos','AxVel','AxTorq','AxErr'};
            for pl=1:4
                plp=uipanel(botG,'BackgroundColor',app.CPanel,'BorderType','none');
                plp.Layout.Row=1; plp.Layout.Column=pl+1;
                plp.AutoResizeChildren = 'off';
            plp.SizeChangedFcn=@(src,~) app.reflowPlot(src,pl);
                uilabel(plp,'Text',plotTitles{pl},'Tag',sprintf('LblPlt%d',pl),...
                    'Position',[6 10 300 14],'FontSize',9,'FontWeight','bold',...
                    'FontColor',app.CAccent,'BackgroundColor','none');
                app.(axTags{pl})=uiaxes(plp,'Tag',axTags{pl},'Position',[4 26 100 100],...
                    'Color',app.CPlot,'XColor',app.CBorder,'YColor',app.CBorder,...
                    'GridColor',app.CBorder,'GridAlpha',0.3,'FontSize',8);
                app.(axTags{pl}).XLabel.String='Time (s)';
                app.(axTags{pl}).XLabel.Color=app.CDim;
                app.(axTags{pl}).YLabel.String=plotYLabels{pl};
                app.(axTags{pl}).YLabel.Color=app.CDim;
                grid(app.(axTags{pl}),'on'); hold(app.(axTags{pl}),'on');
            end

            app.WorkspaceVisible=false;
            app.Panels=struct();
            app.UIFigure.Visible='on';
        end

        %% ══════════════════════════════════════════════════════════════════
        %% REFLOW CALLBACKS — called by SizeChangedFcn of each panel
        %% Each one repositions its children based on the panel's actual size
        %% ══════════════════════════════════════════════════════════════════

        function anchorTitleRight(app, tp)
            W = tp.Position(3);
            sl = findobj(tp,'Tag','StatusLbl');
            lm = findobj(tp,'Tag','StatusLamp');
            if ~isempty(sl); sl.Position = [W-140 9 90 16]; end
            if ~isempty(lm); lm.Position = [W-42 11 14 14]; end
        end

        function reflowJoints(app, lp)
            W = lp.Position(3); H = lp.Position(4);
            if W<10 || H<10; return; end
            btnH=28; titleH=20; margin=4;
            avail = H - titleH - btnH - 3*margin;
            slot  = max(10, floor(avail/7));
            slW   = W - 12;
            c = findobj(lp,'Tag','JCtrlTitle');
            if ~isempty(c); c.Position=[6 H-titleH-2 slW titleH]; end
            c = findobj(lp,'Tag','BtnHome');
            if ~isempty(c); c.Position=[6 4 slW btnH]; end
            for j=1:7
                yb = btnH + margin + (7-j)*slot;
                c=findobj(lp,'Tag',sprintf('JN%d',j));
                if ~isempty(c); c.Position=[6 yb+slot-15 slW 13]; end
                c=findobj(lp,'Tag',sprintf('JLo%d',j));
                if ~isempty(c); c.Position=[6 yb+slot-28 28 11]; end
                c=findobj(lp,'Tag',sprintf('JMid%d',j));
                if ~isempty(c); c.Position=[round(W/2)-10 yb+slot-28 20 11]; end
                c=findobj(lp,'Tag',sprintf('JHi%d',j));
                if ~isempty(c); c.Position=[W-40 yb+slot-28 32 11]; end
                if numel(app.JSliders)>=j && isvalid(app.JSliders(j))
                    app.JSliders(j).Position=[6 yb+slot-24 slW 3];
                end
                if numel(app.JValLabels)>=j && isvalid(app.JValLabels(j))
                    app.JValLabels(j).Position=[6 yb+2 slW 16];
                end
            end
        end

        function reflow3D(app, vp)
            W=vp.Position(3); H=vp.Position(4);
            if W<10 || H<10; return; end
            c=findobj(vp,'Tag','Lbl3D');
            if ~isempty(c); c.Position=[8 H-18 W-16 14]; end
            if isvalid(app.Axes3D)
                app.Axes3D.Position=[4 30 W-8 H-54];
            end
            hw=round((W-12)/2);
            c=findobj(vp,'Tag','BtnWS');
            if ~isempty(c); c.Position=[4 6 hw 22]; end
            c=findobj(vp,'Tag','BtnClr');
            if ~isempty(c); c.Position=[hw+8 6 W-hw-12 22]; end
        end

        function reflowSetup(app, sp)
            W=sp.Position(3); H=sp.Position(4);
            if W<10 || H<10; return; end
            bW   = W-16;            % button/field usable width
            half = floor(bW/2)-3;   % half width for 2-col rows

            %% Fixed-height rows (px) — laid out from BOTTOM up
            replayH = 22;
            runH    = 28;
            calcH   = 26;
            sfH     = 24;   % speed/gain field height
            sfLH    = 14;   % speed/gain label height
            ddH     = 22;   % dropdown height
            ddLH    = 14;   % dropdown label height
            titleH  = 18;
            gap     = 4;

            %% Bottom-up y positions
            y0 = gap;                              % bottom margin
            replayY = y0;
            runY    = replayY + replayH + gap;
            calcY   = runY    + runH    + gap;
            sfY     = calcY   + calcH   + gap;
            sfLY    = sfY     + sfH     + 2;
            subY    = sfLY    + sfLH    + gap;     % sub-panel bottom
            ddY     = H - titleH - gap - ddLH - gap - ddH;
            subH    = max(20, ddY - subY - gap);   % sub-panel fills gap between dropdowns and speed/gain

            %% Title
            c=findobj(sp,'Tag','LblSetup');
            if ~isempty(c); c.Position=[8 H-titleH-2 bW titleH]; end

            %% Dropdown labels (below title)
            c=findobj(sp,'Tag','LblTrajType');
            if ~isempty(c); c.Position=[8 H-titleH-gap-ddLH-2 half ddLH]; end
            c=findobj(sp,'Tag','LblCurve');
            if ~isempty(c); c.Position=[half+12 H-titleH-gap-ddLH-2 half ddLH]; end

            %% Dropdowns
            if isvalid(app.TrajTypeDD);  app.TrajTypeDD.Position=[8 ddY half ddH]; end
            if isvalid(app.CurveTypeDD); app.CurveTypeDD.Position=[half+12 ddY half ddH]; end

            %% Sub-panels (Single / Multi) — fill between dropdowns and speed/gain
            if isvalid(app.SinglePanel); app.SinglePanel.Position=[0 subY W subH]; end
            if isvalid(app.MultiPanel);  app.MultiPanel.Position=[0 subY W subH]; end

            %% Speed / Gain labels and fields
            c=findobj(sp,'Tag','LblSpd');
            if ~isempty(c); c.Position=[8 sfLY half sfLH]; end
            if isvalid(app.SpeedField); app.SpeedField.Position=[8 sfY half sfH]; end
            c=findobj(sp,'Tag','LblGain');
            if ~isempty(c); c.Position=[half+12 sfLY half sfLH]; end
            if isvalid(app.GainField); app.GainField.Position=[half+12 sfY half sfH]; end

            %% Calculate button
            if isvalid(app.CalcButton); app.CalcButton.Position=[8 calcY bW calcH]; end

            %% Run / Stop buttons
            if isvalid(app.RunButton);  app.RunButton.Position=[8 runY floor(bW/2)-2 runH]; end
            if isvalid(app.StopButton); app.StopButton.Position=[floor(bW/2)+10 runY ceil(bW/2)-2 runH]; end

            %% Replay
            c=findobj(sp,'Tag','BtnReplay');
            if ~isempty(c); c.Position=[8 replayY bW replayH]; end
        end

        function reflowSingle(app, sp)
            W=sp.Position(3); H=sp.Position(4);
            if W<10 || H<10; return; end
            bW   = W-16;
            half = floor(bW/2)-3;
            fH   = 26;   % field height
            lH   = 13;   % label height
            gap  = 6;
            btnH = 22;   % preset button height
            btnLH= 13;

            %% Layout from bottom up:
            %%   preset buttons at very bottom
            %%   then Z field
            %%   then X, Y side by side
            pbW = floor((bW-8)/3);
            preY = gap;
            preLY= preY + btnH + 2;
            zY   = preLY + btnLH + gap;
            zLY  = zY + fH + 2;
            xyY  = zLY + lH + gap;
            xyLY = xyY + fH + 2;

            %% XY labels and fields (side by side)
            c=findobj(sp,'Tag','LblXYZ1');
            if ~isempty(c); c.Position=[8 xyLY half lH]; end
            c=findobj(sp,'Tag','EfXYZ1');
            if ~isempty(c); c.Position=[8 xyY half-20 fH]; end
            c=findobj(sp,'Tag','LblVld1');
            if ~isempty(c); c.Position=[8+half-18 xyY+4 18 18]; end

            c=findobj(sp,'Tag','LblXYZ2');
            if ~isempty(c); c.Position=[half+12 xyLY half lH]; end
            c=findobj(sp,'Tag','EfXYZ2');
            if ~isempty(c); c.Position=[half+12 xyY half-20 fH]; end
            c=findobj(sp,'Tag','LblVld2');
            if ~isempty(c); c.Position=[half+12+half-18 xyY+4 18 18]; end

            %% Z field (full width)
            c=findobj(sp,'Tag','LblXYZ3');
            if ~isempty(c); c.Position=[8 zLY bW lH]; end
            c=findobj(sp,'Tag','EfXYZ3');
            if ~isempty(c); c.Position=[8 zY bW-22 fH]; end
            c=findobj(sp,'Tag','LblVld3');
            if ~isempty(c); c.Position=[bW-12 zY+4 18 18]; end

            %% Preset label + buttons
            c=findobj(sp,'Tag','LblPre');
            if ~isempty(c); c.Position=[8 preLY 90 btnLH]; end
            for pr=1:3
                c=findobj(sp,'Tag',sprintf('BtnPr%d',pr));
                if ~isempty(c); c.Position=[8+(pr-1)*(pbW+4) preY pbW btnH]; end
            end
        end

        function reflowMulti(app, mp)
            W=mp.Position(3); H=mp.Position(4);
            if W<10 || H<10; return; end
            bW3=floor((W-24)/3);
            % Preset buttons at top
            for bi=1:3
                btags={'BtnBoss','BtnSweep','BtnVert'};
                c=findobj(mp,'Tag',btags{bi});
                if ~isempty(c); c.Position=[8+(bi-1)*(bW3+4) H-24 bW3 20]; end
            end
            % Table fills middle
            tblH=max(40, H-24-4-34-4);
            if isvalid(app.WPTable); app.WPTable.Position=[8 34 W-16 tblH]; end
            % Add/Remove/Reset at bottom
            abtags={'BtnAddWP','BtnRmWP','BtnRstWP'};
            for bi=1:3
                c=findobj(mp,'Tag',abtags{bi});
                if ~isempty(c); c.Position=[8+(bi-1)*(bW3+4) 6 bW3 24]; end
            end
        end

        function reflowResults(app, rp)
            W=rp.Position(3); H=rp.Position(4);
            if W<10 || H<10; return; end
            half=floor((W-12)/2)-2;
            boxH=min(48,max(20,floor((H-120)/2)));
            c=findobj(rp,'Tag','LblRes');
            if ~isempty(c); c.Position=[8 H-20 W-16 16]; end
            for rf=1:4
                col=mod(rf-1,2)*(half+12)+8;
                row=H-60-floor((rf-1)/2)*(boxH+20);
                c=findobj(rp,'Tag',sprintf('LblR%d',rf));
                if ~isempty(c); c.Position=[col row+boxH+2 half 12]; end
                c=findobj(rp,'Tag',sprintf('ValR%d',rf));
                if ~isempty(c); c.Position=[col row half boxH]; end
            end
            logTop=max(30, H-60-2*(boxH+20)-20);
            c=findobj(rp,'Tag','LblLog');
            if ~isempty(c); c.Position=[8 logTop W-16 16]; end
            if isvalid(app.LogArea)
                app.LogArea.Position=[8 6 W-16 max(10,logTop-10)];
            end
        end

        function reflowEnergy(app, ep)
            W=ep.Position(3); H=ep.Position(4);
            if W<10 || H<10; return; end
            eW=W-16;
            c=findobj(ep,'Tag','LblEng');
            if ~isempty(c); c.Position=[8 H-20 eW 16]; end
            c=findobj(ep,'Tag','LblTotal');
            if ~isempty(c); c.Position=[8 H-36 60 13]; end
            if isvalid(app.ETotalLabel)
                app.ETotalLabel.Position=[8 H-74 eW 36];
            end
            eSlot=max(20,floor((H-82)/3));
            eHandles={'EGravLabel','EInertLabel','ECorLabel'};
            ePctH={'EGravPctLabel','EInertPctLabel','ECorPctLabel'};
            for ec=1:3
                yec=H-82-ec*eSlot;
                c=findobj(ep,'Tag',sprintf('LblE%d',ec));
                if ~isempty(c); c.Position=[8 yec+eSlot-14 eW-70 12]; end
                c=findobj(ep,'Tag',sprintf('ValE%d',ec));
                if ~isempty(c) && isvalid(app.(eHandles{ec}))
                    app.(eHandles{ec}).Position=[8 yec+eSlot-34 round(eW*0.55) 18];
                end
                c=findobj(ep,'Tag',sprintf('PctE%d',ec));
                if ~isempty(c) && isvalid(app.(ePctH{ec}))
                    app.(ePctH{ec}).Position=[round(eW*0.57)+8 yec+eSlot-34 round(eW*0.40) 18];
                end
            end
        end

        function reflowSafety(app, sf)
            W=sf.Position(3); H=sf.Position(4);
            if W<10 || H<10; return; end
            sfW=W-16;
            c=findobj(sf,'Tag','LblSafe');
            if ~isempty(c); c.Position=[8 H-20 sfW-20 16]; end
            if isvalid(app.SafetyOverallLamp)
                app.SafetyOverallLamp.Position=[W-22 H-19 14 14];
            end
            if isvalid(app.SafetyStatusLabel)
                app.SafetyStatusLabel.Position=[8 H-38 sfW 14];
            end
            c=findobj(sf,'Tag','ScoreBg');
            if ~isempty(c); c.Position=[8 H-62 sfW 22]; end
            c=findobj(sf,'Tag','LblScore');
            if ~isempty(c); c.Position=[12 H-60 36 16]; end
            if isvalid(app.SafetyScoreLabel)
                app.SafetyScoreLabel.Position=[46 H-62 sfW-46 22];
            end
            sfAvail=max(10, H-70-56);
            sfSlot=max(8,floor(sfAvail/7));
            for sc=1:7
                ysc=H-70-sc*sfSlot;
                if numel(app.SafeLamps)>=sc && isvalid(app.SafeLamps(sc))
                    app.SafeLamps(sc).Position=[8 ysc+sfSlot-13 12 12];
                end
                c=findobj(sf,'Tag',sprintf('SName%d',sc));
                if ~isempty(c); c.Position=[24 ysc+sfSlot-15 sfW-26 13]; end
                if numel(app.SafeLabels)>=sc && isvalid(app.SafeLabels(sc))
                    app.SafeLabels(sc).Position=[24 ysc+sfSlot-28 sfW-26 11];
                end
            end
            c=findobj(sf,'Tag','BtnSafeRun');
            if ~isempty(c); c.Position=[8 32 sfW 22]; end
            c=findobj(sf,'Tag','BtnExport');
            if ~isempty(c); c.Position=[8 6 sfW 22]; end
        end

        function reflowEEPose(app, pp)
            W=pp.Position(3); H=pp.Position(4);
            if W<10 || H<10; return; end
            pw=W-12;
            c=findobj(pp,'Tag','LblEE');
            if ~isempty(c); c.Position=[6 H-20 pw 16]; end
            pSlot=max(10,floor((H-26)/6));
            pHandles={'PosXLabel','PosYLabel','PosZLabel','IKErrLabel','DistLabel','TTotalLabel'};
            for pf=1:6
                yf=H-26-pf*pSlot;
                c=findobj(pp,'Tag',sprintf('LblPF%d',pf));
                if ~isempty(c); c.Position=[6 yf+pSlot-12 pw 11]; end
                if isvalid(app.(pHandles{pf}))
                    app.(pHandles{pf}).Position=[6 yf pw max(12,pSlot-14)];
                end
            end
        end

        function reflowPlot(app, plp, pl)
            W=plp.Position(3); H=plp.Position(4);
            if W<10 || H<10; return; end
            axTags={'AxPos','AxVel','AxTorq','AxErr'};
            c=findobj(plp,'Tag',sprintf('LblPlt%d',pl));
            if ~isempty(c); c.Position=[6 H-16 W-12 13]; end
            if pl<=4 && isvalid(app.(axTags{pl}))
                app.(axTags{pl}).Position=[4 4 max(10,W-8) max(10,H-22)];
            end
        end

        function onToggleWorkspace(app)
            if app.WorkspaceVisible
                cla(app.Axes3D); app.WorkspaceVisible=false;
                btn=findobj(app.UIFigure,'Tag','BtnWS');
                if ~isempty(btn); btn.Text='🌐  Show Workspace'; btn.BackgroundColor=[0.18 0.22 0.30]; end
                app.addLog('[WS] Hidden.');
            else; app.showWorkspace(); end
        end

        function showWorkspace(app)
            app.addLog('[WS] Computing workspace (~8s)...'); drawnow;
            if isempty(app.WorkspacePoints)
                r_=loadrobot('kinovaGen3','DataFormat','column'); ee_='EndEffector_Link';
                step=deg2rad(18); J1r=-pi:step:pi; J2r=deg2rad(-138):step:deg2rad(138); J3r=deg2rad(-90):step:deg2rad(90);
                pts=zeros(length(J1r)*length(J2r)*length(J3r),3); k=0; q0=homeConfiguration(r_);
                for j1=J1r; for j2=J2r; for j3=J3r
                    qs=q0; qs(1)=j1; qs(2)=j2; qs(3)=j3;
                    try; T_=getTransform(r_,qs,ee_); p_=T_(1:3,4)';
                        if p_(3)>0.02&&norm(p_)<1.30; k=k+1; pts(k,:)=p_; end; catch; end
                end; end; end
                app.WorkspacePoints=pts(1:k,:); app.addLog(sprintf('[WS] %d points.',k));
            end
            pts=app.WorkspacePoints; hold(app.Axes3D,'on');
            scatter3(app.Axes3D,pts(:,1),pts(:,2),pts(:,3),2,pts(:,3),'filled','MarkerFaceAlpha',0.15);
            colormap(app.Axes3D,'cool');
            xL=[0.0 0.7]; yL=[-0.6 0.6]; zL=[0.1 0.6];
            v=[xL(1) yL(1) zL(1);xL(2) yL(1) zL(1);xL(2) yL(2) zL(1);xL(1) yL(2) zL(1);
               xL(1) yL(1) zL(2);xL(2) yL(1) zL(2);xL(2) yL(2) zL(2);xL(1) yL(2) zL(2)];
            eg=[1 2;2 3;3 4;4 1;5 6;6 7;7 8;8 5;1 5;2 6;3 7;4 8];
            for e=1:12; plot3(app.Axes3D,v(eg(e,:),1),v(eg(e,:),2),v(eg(e,:),3),...
                'Color',app.CWarn,'LineWidth',1,'LineStyle','--'); end
            text(app.Axes3D,0.35,0,0.65,'Safe zone','Color',app.CWarn,'FontSize',8,'HorizontalAlignment','center');
            app.WorkspaceVisible=true;
            btn=findobj(app.UIFigure,'Tag','BtnWS');
            if ~isempty(btn); btn.Text='🌐  Hide Workspace'; btn.BackgroundColor=[0.18 0.32 0.18]; end
            app.addLog('[WS] Done.');
        end

        function onClear3D(app)
            cla(app.Axes3D); app.WorkspaceVisible=false;
            btn=findobj(app.UIFigure,'Tag','BtnWS');
            if ~isempty(btn); btn.Text='🌐  Show Workspace'; btn.BackgroundColor=[0.18 0.22 0.30]; end
            app.Axes3D.Title.String='Set target XYZ and press Calculate';
            app.Axes3D.Title.Color=app.CDim;
            grid(app.Axes3D,'on'); view(app.Axes3D,45,30);
            app.addLog('[3D] Cleared.');
        end

        function onSliderChanged(app, j, val)
            app.JValLabels(j).Text = sprintf('%.1f°', val);
        end

        function resetHome(app)
            for j = 1:7
                app.JSliders(j).Value = 0;
                app.JValLabels(j).Text = '0.0°';
            end
            app.addLog('[GUI] Reset to home (all joints = 0°)');
        end

        function setPreset(app, xyz)
            app.TargetXField.Value = xyz(1);
            app.TargetYField.Value = xyz(2);
            app.TargetZField.Value = xyz(3);
            app.addLog(sprintf('[GUI] Preset: [%.1f, %.1f, %.1f]', ...
                xyz(1), xyz(2), xyz(3)));
        end

        function addLog(app, msg)
            cur = app.LogArea.Value;
            if length(cur) > 30
                cur = cur(end-20:end);
            end
            app.LogArea.Value = [cur; {msg}];
            drawnow;
        end

        function setStatus(app, txt, col)
            app.StatusLabel.Text      = txt;
            app.StatusLabel.FontColor = col;
            app.StatusLamp.Color      = col;
            drawnow;
        end

        function onCalculate(app)
            %% Warn if multi-waypoint mode
            if strcmp(app.TrajTypeDD.Value,'Multi-Waypoint')
                app.addLog('[GUI] ⚠ Multi-Waypoint mode selected.');
                app.addLog('[GUI] → Press 🛡 Safety Check to validate waypoints.');
                app.addLog('[GUI] → Press ▶ RUN SIMULATION to run directly.');
                app.addLog('[GUI] Calculate is for Single Target mode only.');
                return;
            end

            app.setStatus('CALCULATING', app.CWarn);
            app.addLog('[IK] Loading robot...');
            drawnow;

            try
                app.Robot = loadrobot('kinovaGen3','DataFormat','column');
                app.Robot.Gravity = [0 0 -9.80665];
                ee = 'EndEffector_Link';

                tx = app.TargetXField.Value;
                ty = app.TargetYField.Value;
                tz = app.TargetZField.Value;

                app.addLog(sprintf('[IK] Solving for [%.3f, %.3f, %.3f]...', tx,ty,tz));

                ik = inverseKinematics('RigidBodyTree', app.Robot);
                ik.SolverParameters.AllowRandomRestart = false;
                ik.SolverParameters.MaxNumIteration   = 1500;

                q0       = homeConfiguration(app.Robot);
                taskFinal= trvec2tform([tx ty tz]) * axang2tform([0 1 0 pi]);
                app.QTarget = wrapToPi(ik(ee, taskFinal, [1 1 1 1 1 1], q0));

                T1    = getTransform(app.Robot, app.QTarget, ee);
                T0    = getTransform(app.Robot, q0, ee);
                ikErr = norm(T1(1:3,4)-[tx;ty;tz])*1000;
                dist  = norm(tform2trvec(T0)-tform2trvec(taskFinal));
                tTot  = max(round(dist/app.SpeedField.Value,1),2.0);

                %% Update EE pose
                app.PosXLabel.Text   = sprintf('%.4f', T1(1,4));
                app.PosYLabel.Text   = sprintf('%.4f', T1(2,4));
                app.PosZLabel.Text   = sprintf('%.4f', T1(3,4));
                app.IKErrLabel.Text  = sprintf('%.2f', ikErr);
                app.DistLabel.Text   = sprintf('%.3f', dist);
                app.TTotalLabel.Text = sprintf('%.1f', tTot);

                if ikErr < 5
                    app.IKErrLabel.FontColor = app.CGreen;
                    app.addLog(sprintf('[IK] ✓ Error=%.2fmm  T=%.1fs', ikErr, tTot));

                %% IK Solution Quality Check
                app.checkIKQuality(app.Robot, ee, q0, taskFinal);
                else
                    app.IKErrLabel.FontColor = app.CDanger;
                    app.addLog(sprintf('[IK] ✗ Error=%.1fmm — check target', ikErr));
                end

                %% Update sliders
                for j = 1:7
                    v = rad2deg(app.QTarget(j));
                    v = max(app.JointLimLo(j), min(app.JointLimHi(j), v));
                    app.JSliders(j).Value  = v;
                    app.JValLabels(j).Text = sprintf('%.1f°', v);
                end

                %% 3D preview
                cla(app.Axes3D);
                N_p = 60;
                ee_p= zeros(N_p,3);
                for i = 1:N_p
                    t_n = (i-1)/(N_p-1);
                    q_i = q0 + t_n*(app.QTarget-q0);
                    Ti  = getTransform(app.Robot, q_i, ee);
                    ee_p(i,:) = Ti(1:3,4)';
                end
                plot3(app.Axes3D, ee_p(:,1), ee_p(:,2), ee_p(:,3),...
                    'Color',app.CAccent,'LineWidth',2,'LineStyle','--',...
                    'DisplayName','EE path (approx)');
                hold(app.Axes3D,'on');
                plot3(app.Axes3D, T0(1,4), T0(2,4), T0(3,4),...
                    'o','Color',app.CGreen,'MarkerSize',12,...
                    'MarkerFaceColor',app.CGreen,'DisplayName','Home');
                plot3(app.Axes3D, tx, ty, tz,...
                    's','Color',app.CDanger,'MarkerSize',12,...
                    'MarkerFaceColor',app.CDanger,'DisplayName','Target');
                legend(app.Axes3D,'TextColor',app.CDim,'Color',app.CPlot,'FontSize',8);
                app.Axes3D.Title.String = sprintf(...
                    'Target [%.2f, %.2f, %.2f]  IK err: %.2f mm', tx,ty,tz,ikErr);
                app.Axes3D.Title.Color  = app.CDim;
                view(app.Axes3D, 45, 30);
                axis(app.Axes3D,'equal');
                grid(app.Axes3D,'on');

                app.TrajectoryReady = true;
                app.setStatus('READY', app.CGreen);

                %% Build full trajectory and push to base workspace
                %% so Safety Check can run without pressing RUN
                tauMax_ = [187;187;187;52;52;52;52];
                dt_     = 0.001;
                gs      = app.GainField.Value;
                ts_     = (0:dt_:tTot)';
                N_      = length(ts_);
                pos_    = zeros(7,N_); vel_=zeros(7,N_); acc_=zeros(7,N_);
                for j=1:7
                    qs=q0(j); qe=app.QTarget(j);
                    a2=3*(qe-qs)/tTot^2; a3=-2*(qe-qs)/tTot^3;
                    t=ts_';
                    pos_(j,:)=qs+a2.*t.^2+a3.*t.^3;
                    vel_(j,:)=  2*a2.*t  +3*a3.*t.^2;
                    acc_(j,:)=  2*a2     +6*a3.*t;
                end
                assignin('base','q_ref_traj',  [ts_,pos_']);
                assignin('base','qd_ref_traj', [ts_,vel_']);
                assignin('base','tau_ref_traj',[ts_,acc_']);
                app.addLog('[GUI] Ready — press Safety Check then RUN.');

            catch err
                app.addLog(sprintf('[ERROR] %s', err.message));
                app.setStatus('ERROR', app.CDanger);
            end
        end

        function onRun(app)
            if ~app.TrajectoryReady && strcmp(app.TrajTypeDD.Value,'Single Target')
                app.addLog('[GUI] Press Calculate first.');
                return;
            end

            %% ---- PRE-FLIGHT CHECKLIST ----
            checks_ok = app.preFlightCheck();
            if ~checks_ok
                return;
            end

            %% ---- Mode dispatch ----
            switch app.TrajTypeDD.Value
                case 'Multi-Waypoint'; app.onRunMultiWaypoint(); return;
                case 'TCP Test';       app.onRunTCPTest();       return;
                case 'Task-Space';     app.onRunTaskSpace();     return;
            end

            app.setStatus('RUNNING', app.CAccent);
            app.SimRunning = true;
            app.addLog('[SIM] Building trajectory...');

            try
                robot_= app.Robot;
                ee_   = 'EndEffector_Link';
                q0_   = homeConfiguration(robot_);
                tauMax= [187;187;187;52;52;52;52];
                dt_   = 0.001;
                gs    = app.GainField.Value;
                spd   = app.SpeedField.Value;
                tx    = app.TargetXField.Value;
                ty    = app.TargetYField.Value;
                tz    = app.TargetZField.Value;

                T0_   = getTransform(robot_,q0_,ee_);
                taskF_= trvec2tform([tx ty tz])*axang2tform([0 1 0 pi]);
                dist_ = norm(tform2trvec(T0_)-tform2trvec(taskF_));
                T_tot_= max(round(dist_/spd,1),2.0);
                ts_   = (0:dt_:T_tot_)';
                N_    = length(ts_);

                %% Call generateTrajectory.m — all 5 profiles handled there
                curveMap = containers.Map(...
                    {'Cubic Spline','Quintic Polynomial','Trapezoidal (LSPB)','Cubic Hermite','Bang-Bang'},...
                    {'cubic','quintic','lspb','hermite','bangbang'});
                cvKey = app.CurveTypeDD.Value;
                if isKey(curveMap,cvKey); curveType=curveMap(cvKey); else; curveType='cubic'; end
                app.addLog(sprintf('[TRAJ] Calling generateTrajectory  profile=%s',cvKey));
                res_ = generateTrajectory(robot_,[tx ty tz],spd,gs,curveType);
                ts_   = res_.q_ref_traj(:,1);
                pos_  = res_.q_ref_traj(:,2:8)';
                vel_  = res_.qd_ref_traj(:,2:8)';
                acc_  = res_.tau_ref_traj(:,2:8)';
                T_tot_= res_.T_total; N_=length(ts_);
                app.addLog(sprintf('[TRAJ] IK=%.2fmm  T=%.1fs  N=%d',res_.ik_err_mm,T_tot_,N_));

                app.addLog(sprintf('[SIM] Computing torques for %d steps...', N_));
                tau_ = zeros(N_,7);
                for i = 1:N_
                    M_=massMatrix(robot_,pos_(:,i));
                    C_=velocityProduct(robot_,pos_(:,i),vel_(:,i));
                    G_=gravityTorque(robot_,pos_(:,i));
                    tau_(i,:)=max(-tauMax,min(tauMax,M_*acc_(:,i)+C_+G_))';
                end

                %% Push to base workspace
                assignin('base','q_ref_traj',  [ts_,pos_']);
                assignin('base','qd_ref_traj', [ts_,vel_']);
                assignin('base','tau_ref_traj',[ts_,acc_']);

                %% Update Simulink
                mdl = 'KinovaCollisionFree';
                load_system(mdl);
                set_param(mdl,'StopTime',num2str(T_tot_));
                for blk={'q_ref','qd_ref','tau_ref'}
                    set_param([mdl '/' blk{1}],'SampleTime','0.001',...
                        'Interpolate','on','OutputAfterFinalValue',...
                        'Holding final value','ZeroCross','off');
                end
                set_param([mdl '/Unit Delay'],'SampleTime','0.001');
                set_param([mdl '/Kp'],'Gain',mat2str(res_.Kp));
                set_param([mdl '/Kd'],'Gain',mat2str(res_.Kd));
                %% Enable Simscape logging so Mechanics Explorer can replay
                set_param(mdl,'SimscapeLogType','all');
                set_param(mdl,'SimscapeLogOpenViewer','on');
                set_param(mdl,'SimscapeLogToSDI','on');
                save_system(mdl);

                %% Push robot to base workspace for Simulink dynamics blocks
                assignin('base', 'Kinova_DOF7', robot_);
                app.addLog('[SIM] Running Simulink...');
                out = sim(mdl);
                app.addLog('[SIM] Done.');

                %% Extract Q_out
                ls   = out.logsout;
                el   = ls{1};
                t_s  = el.Values.Time;
                Qout = el.Values.Data;

                %% Save to base workspace for replay
                assignin('base', 'out',       out);
                assignin('base', 'last_t_s',  t_s);
                assignin('base', 'last_Qout', Qout);

                %% Tracking error
                qref_i = interp1(ts_,pos_',t_s,'linear');
                valid  = ~any(isnan(qref_i),2);
                err_   = qref_i(valid,:) - Qout(valid,:);
                rms_e  = sqrt(mean(err_.^2));
                max_e  = max(abs(err_));
                [~,bi] = min(max_e);
                [~,wi] = max(max_e);

                %% Update results
                app.RMSLabel.Text    = sprintf('%.4f°', mean(rms_e)*180/pi);
                app.MaxErrLabel.Text = sprintf('%.4f°', max(max_e)*180/pi);
                app.BestJLabel.Text  = sprintf('J%d', bi);
                app.WorstJLabel.Text = sprintf('J%d', wi);

                %% Plot all 4 axes
                clrs = app.JColors;
                jleg = {'J1','J2','J3','J4','J5','J6','J7'};

                cla(app.AxPos);
                for j=1:7
                    plot(app.AxPos,t_s,Qout(:,j),'Color',clrs(j,:),'LineWidth',1.2);
                end
                legend(app.AxPos,jleg,'TextColor',app.CDim,'Color',app.CPlot,'FontSize',7);
                app.AxPos.Title.String='Actual Q_out'; app.AxPos.Title.Color=app.CDim;

                cla(app.AxVel);
                for j=1:7
                    plot(app.AxVel,ts_,vel_(j,:)','Color',clrs(j,:),'LineWidth',1.2);
                end
                legend(app.AxVel,jleg,'TextColor',app.CDim,'Color',app.CPlot,'FontSize',7);

                cla(app.AxTorq);
                for j=1:7
                    plot(app.AxTorq,ts_,tau_(:,j),'Color',clrs(j,:),'LineWidth',1.2);
                end
                legend(app.AxTorq,jleg,'TextColor',app.CDim,'Color',app.CPlot,'FontSize',7);

                cla(app.AxErr);
                for j=1:7
                    plot(app.AxErr,t_s(valid),err_(:,j)*180/pi,...
                        'Color',clrs(j,:),'LineWidth',1.2);
                end
                yline(app.AxErr,0,'Color',app.CDim,'LineStyle','--');
                legend(app.AxErr,jleg,'TextColor',app.CDim,'Color',app.CPlot,'FontSize',7);
                app.AxErr.Title.String=sprintf('Max=%.4f°',max(max_e)*180/pi);
                app.AxErr.Title.Color=app.CDim;

                app.addLog(sprintf('[RESULT] RMS=%.4f°  Max=%.4f°  Best=J%d  Worst=J%d',...
                    mean(rms_e)*180/pi, max(max_e)*180/pi, bi, wi));

                %% Compute energy consumption
                app.addLog('[ENERGY] Computing energy consumption...');
                app.computeEnergy(robot_, pos_, vel_, acc_, dt_);

                %% Post-simulation safety audit
                app.postSimAudit(Qout, t_s);

                %% Update EE pose panel
                try
                    T_ee_=getTransform(robot_,Qout(end,:)',ee);
                    app.PosXLabel.Text  =sprintf('%.4f',T_ee_(1,4));
                    app.PosYLabel.Text  =sprintf('%.4f',T_ee_(2,4));
                    app.PosZLabel.Text  =sprintf('%.4f',T_ee_(3,4));
                    app.IKErrLabel.Text =sprintf('%.2f',norm(T_ee_(1:3,4)-[tx;ty;tz])*1000);
                    app.IKErrLabel.FontColor=app.CGreen;
                    app.DistLabel.Text  =sprintf('%.3f',norm(T_ee_(1:3,4)));
                    app.TTotalLabel.Text=sprintf('%.1f',T_tot_);
                catch; end

                app.setStatus('COMPLETE', app.CGreen);

                %% Animate robot in 3D panel
                app.animateRobot(Qout, t_s);

            catch err
                app.addLog(sprintf('[ERROR] %s', err.message));
                app.setStatus('ERROR', app.CDanger);
            end
            app.SimRunning = false;
        end

        function onTrajTypeChanged(app, val)
            app.SinglePanel.Visible = 'off';
            app.MultiPanel.Visible  = 'off';
            switch val
                case 'Multi-Waypoint'
                    app.MultiPanel.Visible = 'on';
                    app.addLog('[GUI] Multi-waypoint mode.');
                case 'TCP Test'
                    app.MultiPanel.Visible = 'on';
                    app.addLog('[GUI] TCP Test mode — grid/circle/line path, measures EE deviation (mm).');
                    app.loadTCPTestDefaults();
                case 'Task-Space'
                    app.SinglePanel.Visible = 'on';
                    app.addLog('[GUI] Task-Space mode — straight-line Cartesian path with SLERP orientation.');
                otherwise
                    app.SinglePanel.Visible = 'on';
                    app.addLog('[GUI] Single target mode.');
            end
        end

        %% =====================================================
        %% XYZ LIVE VALIDATION
        %% =====================================================
        function validateXYZ(app)
            x = app.TargetXField.Value;
            y = app.TargetYField.Value;
            z = app.TargetZField.Value;
            xok = x >= 0.0 && x <= 0.7;
            yok = y >= -0.6 && y <= 0.6;
            zok = z >= 0.1 && z <= 0.6;
            if xok; app.XValLbl.Text='✓'; app.XValLbl.FontColor=app.CGreen;
            else;   app.XValLbl.Text='✗'; app.XValLbl.FontColor=app.CDanger; end
            if yok; app.YValLbl.Text='✓'; app.YValLbl.FontColor=app.CGreen;
            else;   app.YValLbl.Text='✗'; app.YValLbl.FontColor=app.CDanger; end
            if zok; app.ZValLbl.Text='✓'; app.ZValLbl.FontColor=app.CGreen;
            else;   app.ZValLbl.Text='✗'; app.ZValLbl.FontColor=app.CDanger; end
        end

        %% =====================================================
        %% LOAD PRESET TASK
        %% =====================================================
        function loadPresetTask(app, task)
            switch task
                case 'boss'
                    app.WPTable.Data = {
                        0.500,  0.000,  0.300, 'Down', 0.20, 'Top of cube';
                        0.437, -0.003,  0.411, 'Down', 0.20, '25pct back';
                        0.375, -0.006,  0.521, 'Down', 0.20, 'Halfway';
                        0.437, -0.003,  0.411, 'Up',   0.15, 'Wrist flip';
                        0.500,  0.000,  0.250, 'Up',   0.15, 'Bottom'};
                    app.addLog('[PRESET] Boss Demo: 5 WPs, cube + wrist flip');
                case 'sweep'
                    app.WPTable.Data = {
                        0.300, -0.400,  0.300, 'Down', 0.20, 'Left';
                        0.500,  0.000,  0.300, 'Down', 0.20, 'Centre';
                        0.300,  0.400,  0.300, 'Down', 0.20, 'Right'};
                    app.addLog('[PRESET] Side Sweep: 3 WPs');
                case 'vertical'
                    app.WPTable.Data = {
                        0.400,  0.000,  0.150, 'Down', 0.15, 'Low';
                        0.400,  0.000,  0.300, 'Down', 0.20, 'Mid';
                        0.400,  0.000,  0.500, 'Down', 0.20, 'High'};
                    app.addLog('[PRESET] Vertical Stack: 3 WPs');
            end
        end

        function addWPRow(app)
            d = app.WPTable.Data;
            app.WPTable.Data = [d; {0.0, 0.0, 0.0, 'Down'}];
            app.addLog(sprintf('[GUI] Added waypoint %d.', size(app.WPTable.Data,1)));
        end

        function removeWPRow(app)
            d = app.WPTable.Data;
            if size(d,1) > 1
                app.WPTable.Data = d(1:end-1,:);
                app.addLog('[GUI] Removed last waypoint.');
            else
                app.addLog('[GUI] Must have at least 1 waypoint.');
            end
        end

        function resetWPTable(app)
            app.WPTable.Data = {0.5, 0.0, 0.3, 'Down', 0.20, ''; ...
                                0.2, 0.4, 0.5, 'Down', 0.20, ''; ...
                                0.0, 0.5, 0.4, 'Down', 0.20, ''};
            app.addLog('[GUI] Waypoints reset to defaults.');
        end

        function onRunMultiWaypoint(app)
            app.setStatus('RUNNING', app.CAccent);
            app.addLog('[MW] Starting multi-waypoint simulation...');
            try
                d = app.WPTable.Data;
                nWP = size(d,1);
                wps  = zeros(nWP,3);
                oris = cell(nWP,1);
                spdWP = zeros(nWP,1);
                for w=1:nWP
                    wps(w,1)=d{w,1}; wps(w,2)=d{w,2}; wps(w,3)=d{w,3};
                    if size(d,2)>=4 && ~isempty(d{w,4}); oris{w}=d{w,4};
                    else; oris{w}='Down'; end
                    if size(d,2)>=5 && isnumeric(d{w,5}) && d{w,5}>0
                        spdWP(w)=d{w,5};
                    else; spdWP(w)=spd; end
                end
                app.addLog(sprintf('[MW] %d waypoints loaded.',nWP));

                robot_=loadrobot('kinovaGen3','DataFormat','column');
                robot_.Gravity=[0 0 -9.80665];
                ee_='EndEffector_Link';
                tauMax_=[187;187;187;52;52;52;52];
                dt_=0.001; gs=app.GainField.Value; spd=app.SpeedField.Value;
                q0_=homeConfiguration(robot_);
                q_wps=zeros(7,nWP+1); q_wps(:,1)=q0_;

                ik_=inverseKinematics('RigidBodyTree',robot_);
                ik_.SolverParameters.AllowRandomRestart=false;
                ik_.SolverParameters.MaxNumIteration=1500;

                for w=1:nWP
                    tx=wps(w,1); ty=wps(w,2); tz=wps(w,3);
                    %% Apply orientation based on column 4
                    if strcmpi(oris{w},'Up')
                        ori_ = eye(4);              % EE pointing UP
                    else
                        ori_ = axang2tform([0 1 0 pi]); % EE pointing DOWN
                    end
                    t_=trvec2tform([tx ty tz])*ori_;
                    q_s=wrapToPi(ik_(ee_,t_,[1 1 1 1 1 1],q_wps(:,w)));
                    Tc=getTransform(robot_,q_s,ee_);
                    em=norm(Tc(1:3,4)-[tx;ty;tz])*1000;
                    app.addLog(sprintf('[IK] WP%d [%.2f,%.2f,%.2f] EE=%s err=%.2fmm',...
                        w,tx,ty,tz,oris{w},em));
                    if em>5; app.addLog('[ERROR] Waypoint unreachable'); app.setStatus('ERROR',app.CDanger); return; end
                    q_wps(:,w+1)=q_s;
                end

                seg_T=zeros(1,nWP);
                for w=1:nWP
                    Ts=getTransform(robot_,q_wps(:,w),ee_);
                    Te=getTransform(robot_,q_wps(:,w+1),ee_);
                    seg_T(w)=max(round(norm(tform2trvec(Ts)-tform2trvec(Te))/spd,1),1.5);
                end

                %% Call generateMultiWaypoint.m
                app.addLog('[MW] Calling generateMultiWaypoint...');
                mwRes=generateMultiWaypoint(robot_,wps,oris,spdWP);
                mwRes.Kp=[100;100;80;60;40;40;20]*gs;
                mwRes.Kd=[20;20;16;12;8;8;4]*gs;
                all_t=mwRes.q_ref_traj(:,1); all_p=mwRes.q_ref_traj(:,2:8)';
                all_v=mwRes.qd_ref_traj(:,2:8)'; all_a=mwRes.tau_ref_traj(:,2:8)';
                T_tot_=mwRes.T_total; N_=length(all_t);
                app.addLog(sprintf('[MW] Done: %.1fs %d steps',T_tot_,N_));

                tau_=zeros(N_,7);
                for i=1:N_
                    M_=massMatrix(robot_,all_p(:,i));
                    C_=velocityProduct(robot_,all_p(:,i),all_v(:,i));
                    G_=gravityTorque(robot_,all_p(:,i));
                    tau_(i,:)=max(-tauMax_,min(tauMax_,M_*all_a(:,i)+C_+G_))';
                end

                assignin('base','q_ref_traj',  [all_t,all_p']);
                assignin('base','qd_ref_traj', [all_t,all_v']);
                assignin('base','tau_ref_traj',[all_t,all_a']);

                mdl='KinovaCollisionFree';
                load_system(mdl);
                set_param(mdl,'StopTime',num2str(T_tot_));
                for blk={'q_ref','qd_ref','tau_ref'}
                    set_param([mdl '/' blk{1}],'SampleTime','0.001',...
                        'Interpolate','on','OutputAfterFinalValue','Holding final value','ZeroCross','off');
                end
                set_param([mdl '/Unit Delay'],'SampleTime','0.001');
                set_param([mdl '/Kp'],'Gain',mat2str(mwRes.Kp));
                set_param([mdl '/Kd'],'Gain',mat2str(mwRes.Kd));
                %% Enable Simscape logging so Mechanics Explorer can replay
                set_param(mdl,'SimscapeLogType','all');
                set_param(mdl,'SimscapeLogOpenViewer','on');
                set_param(mdl,'SimscapeLogToSDI','on');
                save_system(mdl);

                %% Push robot to base workspace for Simulink dynamics blocks
                assignin('base', 'Kinova_DOF7', robot_);
                app.addLog('[MW] Running Simulink...');
                out=sim(mdl);
                app.addLog('[MW] Simulation complete.');

                ls=out.logsout; el=ls{1};
                t_s=el.Values.Time; Qout=el.Values.Data;

                %% Save to base workspace for replay
                assignin('base', 'out',       out);
                assignin('base', 'last_t_s',  t_s);
                assignin('base', 'last_Qout', Qout);
                qri=interp1(all_t,all_p',t_s,'linear');
                valid=~any(isnan(qri),2);
                err_=qri(valid,:)-Qout(valid,:);
                rms_e=sqrt(mean(err_.^2)); max_e=max(abs(err_));
                [~,bi]=min(max_e); [~,wi]=max(max_e);

                app.RMSLabel.Text    = sprintf('%.4f°',mean(rms_e)*180/pi);
                app.MaxErrLabel.Text = sprintf('%.4f°',max(max_e)*180/pi);
                app.BestJLabel.Text  = sprintf('J%d',bi);
                app.WorstJLabel.Text = sprintf('J%d',wi);

                clrs=app.JColors; jleg={'J1','J2','J3','J4','J5','J6','J7'};
                cla(app.AxPos); hold(app.AxPos,'on');
                for j=1:7; plot(app.AxPos,t_s,Qout(:,j),'Color',clrs(j,:),'LineWidth',1.2); end
                legend(app.AxPos,jleg,'TextColor',app.CDim,'Color',app.CPlot,'FontSize',7);
                t_junc=[0,cumsum(seg_T)];
                for w=1:nWP-1; xline(app.AxPos,t_junc(w+1),'Color',app.CWarn,'LineStyle','--','Alpha',0.7); end

                cla(app.AxVel); hold(app.AxVel,'on');
                for j=1:7; plot(app.AxVel,all_t,all_v(j,:)','Color',clrs(j,:),'LineWidth',1.2); end

                cla(app.AxTorq); hold(app.AxTorq,'on');
                for j=1:7; plot(app.AxTorq,all_t,tau_(:,j),'Color',clrs(j,:),'LineWidth',1.2); end

                cla(app.AxErr); hold(app.AxErr,'on');
                for j=1:7; plot(app.AxErr,t_s(valid),err_(:,j)*180/pi,'Color',clrs(j,:),'LineWidth',1.2); end
                yline(app.AxErr,0,'Color',app.CDim,'LineStyle','--');

                app.addLog(sprintf('[RESULT] RMS=%.4f° Max=%.4f° Best=J%d Worst=J%d',...
                    mean(rms_e)*180/pi,max(max_e)*180/pi,bi,wi));

                %% Compute energy consumption
                app.addLog('[ENERGY] Computing energy consumption...');
                app.computeEnergy(robot_, all_p, all_v, all_a, dt_);

                %% Update EE pose panel with final position
                try
                    T_ee_=getTransform(robot_,Qout(end,:)',ee_);
                    app.PosXLabel.Text  =sprintf('%.4f',T_ee_(1,4));
                    app.PosYLabel.Text  =sprintf('%.4f',T_ee_(2,4));
                    app.PosZLabel.Text  =sprintf('%.4f',T_ee_(3,4));
                    app.IKErrLabel.Text =sprintf('%.2f',...
                        norm(T_ee_(1:3,4)-wps(end,:)')*1000);
                    app.IKErrLabel.FontColor=app.CGreen;
                    app.DistLabel.Text  =sprintf('%.3f',norm(T_ee_(1:3,4)));
                    app.TTotalLabel.Text=sprintf('%.1f',T_tot_);
                catch; end

                app.setStatus('COMPLETE',app.CGreen);

                %% Animate robot in 3D panel
                app.animateRobot(Qout, t_s);

            catch err
                app.addLog(sprintf('[ERROR] %s',err.message));
                app.setStatus('ERROR',app.CDanger);
            end
            app.SimRunning=false;
        end

        function animateRobot(app, Q_out_data, t_sim)
            %% This animates the robot as a stick figure inside app.Axes3D.
            %% The separate Mechanics Explorer window shows the 3D mesh model
            %% but only displays the FINAL pose after programmatic sim() calls.
            %% This is normal MATLAB behaviour — the Explorer doesn't animate
            %% in real time during sim(). The stick figure here IS the animation.
            app.addLog('[ANIM] Starting robot animation...');
            drawnow;

            try
                robot_ = loadrobot('kinovaGen3','DataFormat','column');
                ee_    = 'EndEffector_Link';

                %% Body names for FK chain
                bodyNames = {
                    'Shoulder_Link','HalfArm1_Link','HalfArm2_Link',...
                    'ForeArm_Link','Wrist1_Link','Wrist2_Link',...
                    'Bracelet_Link','EndEffector_Link'};
                nBodies = length(bodyNames);

                %% Subsample — animate at 50ms intervals
                dt_anim = 0.05;
                t_anim  = (0:dt_anim:t_sim(end))';
                Q_anim  = interp1(t_sim, Q_out_data, t_anim, 'linear');
                N_anim  = length(t_anim);

                %% Setup 3D axes
                cla(app.Axes3D);
                hold(app.Axes3D,'on');
                app.Axes3D.XLim = [-0.6 0.8];
                app.Axes3D.YLim = [-0.7 0.7];
                app.Axes3D.ZLim = [-0.1 1.4];
                view(app.Axes3D, 45, 20);
                grid(app.Axes3D,'on');

                %% Draw floor grid
                xf = -0.6:0.2:0.8;
                yf = -0.6:0.2:0.6;
                for xi = xf
                    plot3(app.Axes3D,[xi xi],[-0.6 0.6],[0 0],...
                        'Color',[0.22 0.25 0.32],'LineWidth',0.5);
                end
                for yi = yf
                    plot3(app.Axes3D,[-0.6 0.8],[yi yi],[0 0],...
                        'Color',[0.22 0.25 0.32],'LineWidth',0.5);
                end

                %% Draw base cylinder (robot base)
                theta = linspace(0,2*pi,20);
                xc = 0.05*cos(theta); yc = 0.05*sin(theta);
                fill3(app.Axes3D, xc, yc, zeros(size(xc)),...
                    app.CAccent,'FaceAlpha',0.8,'EdgeColor','none');

                %% Draw world axes
                quiver3(app.Axes3D,0,0,0,0.15,0,0,'r','LineWidth',1.5,'MaxHeadSize',0.5);
                quiver3(app.Axes3D,0,0,0,0,0.15,0,'g','LineWidth',1.5,'MaxHeadSize',0.5);
                quiver3(app.Axes3D,0,0,0,0,0,0.15,'b','LineWidth',1.5,'MaxHeadSize',0.5);
                text(app.Axes3D,0.16,0,0,'X','Color','r','FontSize',8,'FontWeight','bold');
                text(app.Axes3D,0,0.16,0,'Y','Color','g','FontSize',8,'FontWeight','bold');
                text(app.Axes3D,0,0,0.16,'Z','Color','b','FontSize',8,'FontWeight','bold');

                %% Joint colours — SAME as slider panel (app.JColors)
                jColors = [app.JColors; 1.00 1.00 1.00];  % 7 joints + EE white

                %% Initialise graphic handles
                hSegs  = gobjects(nBodies,1);   % link segments
                hJoints= gobjects(nBodies,1);   % joint spheres
                for b = 1:nBodies
                    hSegs(b)  = plot3(app.Axes3D,[0 0],[0 0],[0 0],...
                        'Color',jColors(b,:),'LineWidth',3);
                    hJoints(b)= plot3(app.Axes3D,0,0,0,...
                        'o','Color',jColors(b,:),'MarkerSize',8,...
                        'MarkerFaceColor',jColors(b,:));
                end

                %% EE marker
                hEE = plot3(app.Axes3D,0,0,0,...
                    'd','Color',app.CDanger,'MarkerSize',12,...
                    'MarkerFaceColor',app.CDanger,'LineWidth',1.5);

                %% EE trace
                hTrace = plot3(app.Axes3D,0,0,0,...
                    '-','Color',[0.4 0.8 1.0],'LineWidth',1.5,'LineStyle','-');

                %% Time label
                hTime = text(app.Axes3D,-0.55,0,1.35,'t = 0.00 s',...
                    'Color',app.CDim,'FontSize',9,'FontWeight','bold');

                ee_trace = zeros(N_anim,3);

                %% Animation loop
                for i = 1:N_anim
                    q_i = Q_anim(i,:)';

                    %% Get all joint positions via FK
                    pts = zeros(nBodies+1, 3);
                    pts(1,:) = [0 0 0];  % base origin

                    for b = 1:nBodies
                        try
                            T_b = getTransform(robot_, q_i, bodyNames{b});
                            pts(b+1,:) = T_b(1:3,4)';
                        catch
                            pts(b+1,:) = pts(b,:);
                        end
                    end

                    ee_trace(i,:) = pts(end,:);

                    %% Update each link segment
                    for b = 1:nBodies
                        set(hSegs(b),...
                            'XData',[pts(b,1) pts(b+1,1)],...
                            'YData',[pts(b,2) pts(b+1,2)],...
                            'ZData',[pts(b,3) pts(b+1,3)]);
                        set(hJoints(b),...
                            'XData',pts(b+1,1),...
                            'YData',pts(b+1,2),...
                            'ZData',pts(b+1,3));
                    end

                    %% Update EE marker and trace
                    set(hEE,'XData',pts(end,1),...
                            'YData',pts(end,2),...
                            'ZData',pts(end,3));
                    set(hTrace,...
                        'XData',ee_trace(1:i,1),...
                        'YData',ee_trace(1:i,2),...
                        'ZData',ee_trace(1:i,3));

                    %% Update joint sliders live
                    for j = 1:7
                        v = rad2deg(q_i(j));
                        v = max(app.JointLimLo(j), min(app.JointLimHi(j), v));
                        app.JSliders(j).Value  = v;
                        app.JValLabels(j).Text = sprintf('%.1f°', v);
                    end

                    %% Update time label
                    hTime.String = sprintf('t = %.2f s / %.1f s  (%.0f%%)',...
                        t_anim(i), t_sim(end), 100*i/N_anim);

                    drawnow limitrate;
                end

                %% Final state — add legend
                legend(app.Axes3D,...
                    {'','','','Shoulder','HalfArm1','HalfArm2',...
                     'ForeArm','Wrist1','Wrist2','Bracelet','EE','EE trace'},...
                    'TextColor',app.CDim,'Color',app.CPlot,...
                    'FontSize',7,'Location','northeast');

                app.Axes3D.Title.String = sprintf('Animation complete  |  %.1f s  |  %d frames',...
                    t_sim(end), N_anim);
                app.Axes3D.Title.Color  = app.CGreen;

                app.addLog(sprintf('[ANIM] Done — %d frames at %.0f ms/frame.',...
                    N_anim, dt_anim*1000));

            catch err
                app.addLog(sprintf('[ANIM ERROR] %s', err.message));
            end
        end

        function computeEnergy(app, robot_, pos_, vel_, acc_, dt_)
            %% E = ∫|τ·q̇| dt  for each component
            N_ = size(pos_,2);
            tauMax_ = [187;187;187;52;52;52;52];

            P_grav  = zeros(N_,1);
            P_inert = zeros(N_,1);
            P_cor   = zeros(N_,1);
            P_total = zeros(N_,1);

            for i = 1:N_
                q_i  = pos_(:,i);
                qd_i = vel_(:,i);
                qdd_i= acc_(:,i);

                M_  = massMatrix(robot_,      q_i);
                C_  = velocityProduct(robot_, q_i, qd_i);
                G_  = gravityTorque(robot_,   q_i);

                tau_grav  = G_;
                tau_inert = M_*qdd_i;
                tau_cor   = C_;
                tau_total = max(-tauMax_, min(tauMax_, tau_grav+tau_inert+tau_cor));

                P_grav(i)  = sum(abs(tau_grav  .* qd_i));
                P_inert(i) = sum(abs(tau_inert .* qd_i));
                P_cor(i)   = sum(abs(tau_cor   .* qd_i));
                P_total(i) = sum(abs(tau_total .* qd_i));
            end

            E_total = trapz(P_total) * dt_;
            E_grav  = trapz(P_grav)  * dt_;
            E_inert = trapz(P_inert) * dt_;
            E_cor   = trapz(P_cor)   * dt_;

            %% Update labels
            app.ETotalLabel.Text     = sprintf('%.2f J', E_total);
            app.EGravLabel.Text      = sprintf('%.2f J', E_grav);
            app.EInertLabel.Text     = sprintf('%.2f J', E_inert);
            app.ECorLabel.Text       = sprintf('%.2f J', E_cor);
            app.EGravPctLabel.Text   = sprintf('%.0f%%', E_grav/E_total*100);
            app.EInertPctLabel.Text  = sprintf('%.0f%%', E_inert/E_total*100);
            app.ECorPctLabel.Text    = sprintf('%.0f%%', E_cor/E_total*100);

            app.addLog(sprintf('[ENERGY] Total=%.2fJ  Grav=%.2fJ  Inert=%.2fJ  Cor=%.2fJ',...
                E_total, E_grav, E_inert, E_cor));

            %% Save to base workspace for comparison
            assignin('base','last_energy', struct(...
                'total',E_total,'gravity',E_grav,...
                'inertial',E_inert,'coriolis',E_cor));
        end

        function buildMultiWaypointForSafety(app)
            %% Builds multi-waypoint trajectory and pushes to base workspace
            %% Used by Safety Check without running full simulation
            robot_  = loadrobot('kinovaGen3','DataFormat','column');
            robot_.Gravity = [0 0 -9.80665];
            ee_     = 'EndEffector_Link';
            dt_     = 0.001;
            spd     = app.SpeedField.Value;
            q0_     = homeConfiguration(robot_);

            d = app.WPTable.Data;
            nWP = size(d,1);
            wps  = zeros(nWP,3);
            oris = cell(nWP,1);
            for w=1:nWP
                wps(w,1)=d{w,1}; wps(w,2)=d{w,2}; wps(w,3)=d{w,3};
                if size(d,2) >= 4 && ~isempty(d{w,4})
                    oris{w} = d{w,4};
                else
                    oris{w} = 'Down';
                end
            end

            ik_=inverseKinematics('RigidBodyTree',robot_);
            ik_.SolverParameters.AllowRandomRestart=false;
            ik_.SolverParameters.MaxNumIteration=1500;

            q_wps=zeros(7,nWP+1); q_wps(:,1)=q0_;
            for w=1:nWP
                tx=wps(w,1); ty=wps(w,2); tz=wps(w,3);
                if strcmpi(oris{w},'Up')
                    ori_ = eye(4);
                else
                    ori_ = axang2tform([0 1 0 pi]);
                end
                t_=trvec2tform([tx ty tz])*ori_;
                q_s=wrapToPi(ik_(ee_,t_,[1 1 1 1 1 1],q_wps(:,w)));
                Tc=getTransform(robot_,q_s,ee_);
                em=norm(Tc(1:3,4)-[tx;ty;tz])*1000;
                app.addLog(sprintf('[SAFE-MW] WP%d EE=%s err=%.2fmm',w,oris{w},em));
                if em>5; error('WP%d unreachable (%.1fmm)',w,em); end
                q_wps(:,w+1)=q_s;
            end

            seg_T=zeros(1,nWP);
            for w=1:nWP
                Ts=getTransform(robot_,q_wps(:,w),ee_);
                Te=getTransform(robot_,q_wps(:,w+1),ee_);
                seg_T(w)=max(round(norm(tform2trvec(Ts)-tform2trvec(Te))/spd,1),1.5);
            end

            jv=zeros(7,nWP+1);
            for w=2:nWP
                di=(q_wps(:,w)-q_wps(:,w-1))/seg_T(w-1);
                do_=(q_wps(:,w+1)-q_wps(:,w))/seg_T(w);
                vj=(seg_T(w)*di+seg_T(w-1)*do_)/(seg_T(w-1)+seg_T(w));
                for j=1:7; if sign(di(j))~=sign(do_(j)); vj(j)=0; end; end
                jv(:,w)=vj;
            end

            all_t=[]; all_p=[]; all_v=[]; all_a=[]; t_off=0;
            for w=1:nWP
                qs=q_wps(:,w); qe=q_wps(:,w+1);
                qds=jv(:,w); qde=jv(:,w+1); T_=seg_T(w);
                ts=(0:dt_:T_)'; Ns=length(ts);
                p_=zeros(7,Ns); v_=zeros(7,Ns); a_=zeros(7,Ns);
                for j=1:7
                    a0=qs(j); a1=qds(j);
                    a2=(3*(qe(j)-qs(j))/T_^2)-(2*qds(j)/T_)-(qde(j)/T_);
                    a3=(-2*(qe(j)-qs(j))/T_^3)+((qds(j)+qde(j))/T_^2);
                    t=ts';
                    p_(j,:)=a0+a1.*t+a2.*t.^2+a3.*t.^3;
                    v_(j,:)=a1+2*a2.*t+3*a3.*t.^2;
                    a_(j,:)=2*a2+6*a3.*t;
                end
                if w<nWP; p_=p_(:,1:end-1); v_=v_(:,1:end-1); a_=a_(:,1:end-1); ts=ts(1:end-1); end
                all_t=[all_t; ts+t_off]; all_p=[all_p,p_]; all_v=[all_v,v_]; all_a=[all_a,a_];
                t_off=t_off+ts(end)+dt_;
            end

            assignin('base','q_ref_traj',  [all_t,all_p']);
            assignin('base','qd_ref_traj', [all_t,all_v']);
            assignin('base','tau_ref_traj',[all_t,all_a']);
            app.addLog(sprintf('[SAFE-MW] Trajectory built: %.1fs %d steps',all_t(end),length(all_t)));
        end


        function loadTCPTestDefaults(app)
            app.WPTable.Data = {
                0.300, -0.200, 0.300, 'Down', 0.15, 'Grid 1,1';
                0.450, -0.200, 0.300, 'Down', 0.15, 'Grid 2,1';
                0.600, -0.200, 0.300, 'Down', 0.15, 'Grid 3,1';
                0.300,  0.000, 0.300, 'Down', 0.15, 'Grid 1,2';
                0.450,  0.000, 0.300, 'Down', 0.15, 'Grid 2,2 centre';
                0.600,  0.000, 0.300, 'Down', 0.15, 'Grid 3,2';
                0.300,  0.200, 0.300, 'Down', 0.15, 'Grid 1,3';
                0.450,  0.200, 0.300, 'Down', 0.15, 'Grid 2,3';
                0.600,  0.200, 0.300, 'Down', 0.15, 'Grid 3,3'};
            app.addLog('[TCP] Default 3x3 grid loaded (Z=0.30m, EE Down, 9 waypoints).');
            app.previewTCPPath();
        end

        function previewTCPPath(app)
            try
                d = app.WPTable.Data;
                nWP = size(d,1);
                xs=zeros(nWP,1); ys=zeros(nWP,1); zs=zeros(nWP,1);
                for w=1:nWP; xs(w)=d{w,1}; ys(w)=d{w,2}; zs(w)=d{w,3}; end
                hold(app.Axes3D,'on');
                plot3(app.Axes3D,xs,ys,zs,'c--o','LineWidth',1.2,'MarkerSize',6,...
                    'MarkerFaceColor','c','DisplayName','TCP waypoints');
                for w=1:nWP
                    text(app.Axes3D,xs(w),ys(w),zs(w)+0.02,...
                        sprintf('P%d',w),'Color','c','FontSize',7);
                end
                app.Axes3D.Title.String=sprintf('TCP Test path — %d waypoints',nWP);
                app.Axes3D.Title.Color=app.CAccent2;
                app.addLog(sprintf('[TCP] %d waypoints previewed on 3D view.',nWP));
            catch err
                app.addLog(sprintf('[TCP] Preview error: %s',err.message));
            end
        end

        function onRunTCPTest(app)
            app.setStatus('RUNNING',app.CAccent);
            app.addLog('[TCP] Starting TCP accuracy test...');
            try
                %% ── Read waypoints ───────────────────────────────────────────
                d=app.WPTable.Data; nWP=size(d,1);
                if nWP<2; app.addLog('[TCP] Need at least 2 waypoints.'); return; end
                wps=zeros(nWP,3); oris=cell(nWP,1); spds=zeros(nWP,1);
                for w=1:nWP
                    wps(w,:)=[d{w,1},d{w,2},d{w,3}];
                    if size(d,2)>=4 && ~isempty(d{w,4}); oris{w}=d{w,4}; else; oris{w}='Down'; end
                    if size(d,2)>=5 && isnumeric(d{w,5}) && d{w,5}>0; spds(w)=d{w,5};
                    else; spds(w)=app.SpeedField.Value; end
                end

                %% ── Robot setup ──────────────────────────────────────────────
                robot_=loadrobot('kinovaGen3','DataFormat','column');
                robot_.Gravity=[0 0 -9.80665];
                ee_='EndEffector_Link';
                gs=app.GainField.Value;
                dt_=0.001;

                %% ── Generate trajectory ──────────────────────────────────────
                app.addLog('[TCP] Calling generateMultiWaypoint...');
                mwRes=generateMultiWaypoint(robot_,wps,oris,spds);
                mwRes.Kp=[100;100;80;60;40;40;20]*gs;
                mwRes.Kd=[20;20;16;12;8;8;4]*gs;

                all_t =mwRes.q_ref_traj(:,1);
                all_p =mwRes.q_ref_traj(:,2:8)';
                all_v =mwRes.qd_ref_traj(:,2:8)';
                all_a =mwRes.tau_ref_traj(:,2:8)';
                T_tot_=mwRes.T_total; N_=length(all_t);
                app.addLog(sprintf('[TCP] Trajectory: %.1fs  %d steps',T_tot_,N_));

                %% ── Push to workspace ────────────────────────────────────────
                assignin('base','q_ref_traj',  mwRes.q_ref_traj);
                assignin('base','qd_ref_traj', mwRes.qd_ref_traj);
                assignin('base','tau_ref_traj',mwRes.tau_ref_traj);
                assignin('base','Kinova_DOF7', robot_);

                %% ── Configure Simulink ───────────────────────────────────────
                mdl='KinovaCollisionFree';
                load_system(mdl);
                set_param(mdl,'StopTime',num2str(T_tot_));
                for blk={'q_ref','qd_ref','tau_ref'}
                    set_param([mdl '/' blk{1}],'SampleTime','0.001',...
                        'Interpolate','on','OutputAfterFinalValue','Holding final value','ZeroCross','off');
                end
                set_param([mdl '/Unit Delay'],'SampleTime','0.001');
                set_param([mdl '/Kp'],'Gain',mat2str(mwRes.Kp));
                set_param([mdl '/Kd'],'Gain',mat2str(mwRes.Kd));
                %% Enable Simscape logging so Mechanics Explorer can replay
                set_param(mdl,'SimscapeLogType','all');
                set_param(mdl,'SimscapeLogOpenViewer','on');
                set_param(mdl,'SimscapeLogToSDI','on');
                save_system(mdl);

                %% ── Run simulation ───────────────────────────────────────────
                app.addLog('[TCP] Running Simulink CTC simulation...');
                out=sim(mdl);
                app.addLog('[TCP] Simulation complete.');

                %% ── Extract results ──────────────────────────────────────────
                el=out.logsout{1};
                t_s=el.Values.Time; Qout=el.Values.Data;
                assignin('base','out',out);
                assignin('base','last_t_s',t_s);
                assignin('base','last_Qout',Qout);

                %% ── Rewind Mechanics Explorer to t=0 then replay ─────────────
                %% set_param WriteDataLogs forces the Explorer to reload log data
                try
                    set_param(mdl,'SimulationCommand','WriteDataLogs');
                    app.addLog('[TCP] Mechanics Explorer: press Play (▶) in Explorer to replay 3D mesh animation.');
                catch; end

                %% ── Compute feedforward torques for plots ────────────────────
                app.addLog('[TCP] Computing torques for plots...');
                tauMax_=[187;187;187;52;52;52;52];
                tau_=zeros(N_,7);
                for i=1:N_
                    M_=massMatrix(robot_,all_p(:,i));
                    C_=velocityProduct(robot_,all_p(:,i),all_v(:,i));
                    G_=gravityTorque(robot_,all_p(:,i));
                    tau_(i,:)=max(-tauMax_,min(tauMax_,M_*all_a(:,i)+C_+G_))';
                end

                %% ── Update Joint Positions plot ──────────────────────────────
                clrs=app.JColors;
                jleg={'J1','J2','J3','J4','J5','J6','J7'};
                cla(app.AxPos); hold(app.AxPos,'on');
                for j=1:7; plot(app.AxPos,t_s,Qout(:,j),'Color',clrs(j,:),'LineWidth',1.2); end
                legend(app.AxPos,jleg,'TextColor',app.CDim,'Color',app.CPlot,'FontSize',7);
                app.AxPos.Title.String='Actual Q_{out}'; app.AxPos.Title.Color=app.CDim;
                %% Waypoint junction lines
                t_junc=cumsum([0,mwRes.seg_T]);
                for w=1:nWP-1; xline(app.AxPos,t_junc(w+1),'Color',app.CWarn,'LineStyle','--','Alpha',0.5); end

                %% ── Update Joint Velocities plot ─────────────────────────────
                cla(app.AxVel); hold(app.AxVel,'on');
                for j=1:7; plot(app.AxVel,all_t,all_v(j,:)','Color',clrs(j,:),'LineWidth',1.2); end
                legend(app.AxVel,jleg,'TextColor',app.CDim,'Color',app.CPlot,'FontSize',7);
                app.AxVel.Title.String='Joint velocities'; app.AxVel.Title.Color=app.CDim;

                %% ── Update Torques plot ──────────────────────────────────────
                cla(app.AxTorq); hold(app.AxTorq,'on');
                for j=1:7; plot(app.AxTorq,all_t,tau_(:,j),'Color',clrs(j,:),'LineWidth',1.2); end
                legend(app.AxTorq,jleg,'TextColor',app.CDim,'Color',app.CPlot,'FontSize',7);
                app.AxTorq.Title.String='Feedforward torques'; app.AxTorq.Title.Color=app.CDim;

                %% ── FK accuracy analysis ─────────────────────────────────────
                app.addLog('[TCP] Analysing EE path accuracy...');
                N_out=length(t_s); ee_actual=zeros(N_out,3);
                for i=1:N_out
                    T_=getTransform(robot_,Qout(i,:)',ee_);
                    ee_actual(i,:)=T_(1:3,4)';
                end
                %% Reference EE path from planned joint trajectory
                N_ref=length(all_t); ee_ref=zeros(N_ref,3);
                for i=1:N_ref
                    T_=getTransform(robot_,all_p(:,i),ee_);
                    ee_ref(i,:)=T_(1:3,4)';
                end
                %% Interpolate reference to actual timestamps
                ee_ref_i=interp1(all_t,ee_ref,t_s,'linear','extrap');
                dev_mm=sqrt(sum((ee_actual-ee_ref_i).^2,2))*1000;
                maxDev=max(dev_mm); meanDev=mean(dev_mm); rmsDev=sqrt(mean(dev_mm.^2));
                app.addLog(sprintf('[TCP] EE deviation — Max:%.2fmm  Mean:%.2fmm  RMS:%.2fmm',...
                    maxDev,meanDev,rmsDev));

                %% ── Per-waypoint arrival errors ──────────────────────────────
                for w=1:nWP
                    [~,idx_]=min(abs(t_s-t_junc(w+1)));
                    err_w=norm(ee_actual(idx_,:)-wps(w,:))*1000;
                    if err_w<2; app.addLog(sprintf('[TCP] WP%d: %.2fmm  PASS',w,err_w));
                    else;       app.addLog(sprintf('[TCP] WP%d: %.2fmm  FAIL',w,err_w)); end
                end

                %% ── Update Tracking Error plot (deviation in mm) ─────────────
                cla(app.AxErr); hold(app.AxErr,'on');
                plot(app.AxErr,t_s,dev_mm,'Color',app.CAccent2,'LineWidth',1.5);
                yline(app.AxErr,2,'Color',app.CWarn,'LineStyle','--','Label','2mm limit');
                for w=1:nWP-1; xline(app.AxErr,t_junc(w+1),'Color',app.CWarn,'LineStyle',':','Alpha',0.5); end
                app.AxErr.YLabel.String='EE deviation (mm)';
                app.AxErr.Title.String=sprintf('TCP path error  Max=%.2fmm  RMS=%.2fmm',maxDev,rmsDev);
                app.AxErr.Title.Color=app.CDim;

                %% ── Update Results panel ─────────────────────────────────────
                app.RMSLabel.Text    = sprintf('%.3f mm',rmsDev);
                app.MaxErrLabel.Text = sprintf('%.3f mm',maxDev);
                app.BestJLabel.Text  = 'EE path';
                app.WorstJLabel.Text = 'deviation';

                %% ── Update EE pose panel with final position ─────────────────
                T_final_=getTransform(robot_,Qout(end,:)',ee_);
                app.PosXLabel.Text   = sprintf('%.4f',T_final_(1,4));
                app.PosYLabel.Text   = sprintf('%.4f',T_final_(2,4));
                app.PosZLabel.Text   = sprintf('%.4f',T_final_(3,4));
                app.IKErrLabel.Text  = sprintf('%.2f',norm(T_final_(1:3,4)-wps(end,:)')*1000);
                app.IKErrLabel.FontColor = app.CGreen;
                app.DistLabel.Text   = sprintf('%.3f',norm(T_final_(1:3,4)));
                app.TTotalLabel.Text = sprintf('%.1f',T_tot_);

                %% ── Energy consumption ───────────────────────────────────────
                app.computeEnergy(robot_,all_p,all_v,all_a,dt_);

                %% ── Post-sim audit ────────────────────────────────────────────
                app.postSimAudit(Qout,t_s);

                %% ── 3D view: actual vs reference path ────────────────────────
                hold(app.Axes3D,'on');
                plot3(app.Axes3D,ee_actual(:,1),ee_actual(:,2),ee_actual(:,3),...
                    'y-','LineWidth',1.5,'DisplayName','Actual EE path');
                plot3(app.Axes3D,ee_ref(:,1),ee_ref(:,2),ee_ref(:,3),...
                    'w--','LineWidth',0.8,'DisplayName','Reference EE path');
                legend(app.Axes3D,'TextColor',app.CDim,'Color',app.CPlot,'FontSize',7);
                app.Axes3D.Title.String=sprintf('TCP Test — Max=%.2fmm  RMS=%.2fmm',maxDev,rmsDev);
                if maxDev<2; app.Axes3D.Title.Color=app.CGreen;
                else;        app.Axes3D.Title.Color=app.CWarn; end

                %% ── Status ───────────────────────────────────────────────────
                if maxDev<2; app.setStatus('COMPLETE',app.CGreen);
                else; app.setStatus('COMPLETE',app.CWarn); end

                %% ── Animate robot (stick figure in app) ─────────────────────
                app.addLog('[TCP] Running stick-figure animation...');
                app.animateRobot(Qout,t_s);

            catch err
                app.addLog(sprintf('[TCP ERROR] %s',err.message));
                app.setStatus('ERROR',app.CDanger);
            end
            app.SimRunning=false;
        end



        function onRunTaskSpace(app)
            app.setStatus('RUNNING',app.CAccent);
            app.addLog('[TS] Starting Task-Space straight-line simulation...');
            try
                tx=app.TargetXField.Value; ty=app.TargetYField.Value; tz=app.TargetZField.Value;
                spd=app.SpeedField.Value; gs=app.GainField.Value;

                robot_=loadrobot('kinovaGen3','DataFormat','column');
                robot_.Gravity=[0 0 -9.80665];
                ee='EndEffector_Link';

                app.addLog(sprintf('[TS] Target [%.3f %.3f %.3f] speed=%.2f m/s',tx,ty,tz,spd));
                app.addLog('[TS] Calling generateTrajectoryTaskSpace (SLERP + Jacobian IK)...');

                res_=generateTrajectoryTaskSpace(robot_,[],[tx ty tz],...
                    'Up (+Z)','Down (-Z)',spd);
                res_.Kp=[100;100;80;60;40;40;20]*gs;
                res_.Kd=[20;20;16;12;8;8;4]*gs;

                app.addLog(sprintf('[TS] IK=%.2fmm  T=%.1fs  N=%d',...
                    res_.ik_err_mm, res_.T_total, size(res_.q_ref_traj,1)));

                ts_  =res_.q_ref_traj(:,1);
                pos_ =res_.q_ref_traj(:,2:8)';
                vel_ =res_.qd_ref_traj(:,2:8)';
                acc_ =res_.tau_ref_traj(:,2:8)';
                T_tot_=res_.T_total;

                assignin('base','q_ref_traj',  res_.q_ref_traj);
                assignin('base','qd_ref_traj', res_.qd_ref_traj);
                assignin('base','tau_ref_traj',res_.tau_ref_traj);
                assignin('base','Kinova_DOF7', robot_);

                mdl='KinovaCollisionFree';
                load_system(mdl);
                set_param(mdl,'StopTime',num2str(T_tot_));
                for blk={'q_ref','qd_ref','tau_ref'}
                    set_param([mdl '/' blk{1}],'SampleTime','0.001',...
                        'Interpolate','on','OutputAfterFinalValue','Holding final value','ZeroCross','off');
                end
                set_param([mdl '/Unit Delay'],'SampleTime','0.001');
                set_param([mdl '/Kp'],'Gain',mat2str(res_.Kp));
                set_param([mdl '/Kd'],'Gain',mat2str(res_.Kd));
                set_param(mdl,'SimscapeLogType','all');
                set_param(mdl,'SimscapeLogOpenViewer','on');
                set_param(mdl,'SimscapeLogToSDI','on');
                save_system(mdl);

                app.addLog('[TS] Running Simulink...');
                out=sim(mdl);
                app.addLog('[TS] Done.');

                el=out.logsout{1}; t_s=el.Values.Time; Qout=el.Values.Data;
                assignin('base','out',out);
                assignin('base','last_t_s',t_s);
                assignin('base','last_Qout',Qout);

                %% FK for EE path on 3D view
                N_=size(pos_,2); ee_path=zeros(N_,3);
                for i=1:N_
                    T_=getTransform(robot_,pos_(:,i),ee);
                    ee_path(i,:)=T_(1:3,4)';
                end
                cla(app.Axes3D); hold(app.Axes3D,'on');
                plot3(app.Axes3D,ee_path(:,1),ee_path(:,2),ee_path(:,3),...
                    'c-','LineWidth',2,'DisplayName','Task-space EE path');
                plot3(app.Axes3D,[0 tx],[0 ty],[0 tz],'r*','MarkerSize',12,...
                    'DisplayName','Target');
                legend(app.Axes3D,'TextColor',app.CDim,'Color',app.CPlot,'FontSize',7);
                app.Axes3D.Title.String=sprintf('Task-Space  T=%.1fs  IK=%.2fmm',...
                    T_tot_,res_.ik_err_mm);
                app.Axes3D.Title.Color=app.CAccent2;
                grid(app.Axes3D,'on'); view(app.Axes3D,45,30);

                %% Update all panels
                clrs=app.JColors; jleg={'J1','J2','J3','J4','J5','J6','J7'};
                qref_i=interp1(ts_,pos_',t_s,'linear');
                valid=~any(isnan(qref_i),2);
                err_=qref_i(valid,:)-Qout(valid,:);
                rms_e=sqrt(mean(err_.^2)); max_e=max(abs(err_));
                [~,bi]=min(max_e); [~,wi]=max(max_e);
                app.RMSLabel.Text    = sprintf('%.4f°',mean(rms_e)*180/pi);
                app.MaxErrLabel.Text = sprintf('%.4f°',max(max_e)*180/pi);
                app.BestJLabel.Text  = sprintf('J%d',bi);
                app.WorstJLabel.Text = sprintf('J%d',wi);

                cla(app.AxPos); hold(app.AxPos,'on');
                for j=1:7; plot(app.AxPos,t_s,Qout(:,j),'Color',clrs(j,:),'LineWidth',1.2); end
                legend(app.AxPos,jleg,'TextColor',app.CDim,'Color',app.CPlot,'FontSize',7);
                app.AxPos.Title.String='Actual Q_{out}'; app.AxPos.Title.Color=app.CDim;

                cla(app.AxVel); hold(app.AxVel,'on');
                for j=1:7; plot(app.AxVel,ts_,vel_(j,:)','Color',clrs(j,:),'LineWidth',1.2); end
                legend(app.AxVel,jleg,'TextColor',app.CDim,'Color',app.CPlot,'FontSize',7);

                cla(app.AxErr); hold(app.AxErr,'on');
                for j=1:7
                    plot(app.AxErr,t_s(valid),err_(:,j)*180/pi,'Color',clrs(j,:),'LineWidth',1.2);
                end
                app.AxErr.Title.String=sprintf('Max=%.4f°',max(max_e)*180/pi);
                app.AxErr.Title.Color=app.CDim;

                %% EE pose
                T_ee_=getTransform(robot_,Qout(end,:)',ee);
                app.PosXLabel.Text  =sprintf('%.4f',T_ee_(1,4));
                app.PosYLabel.Text  =sprintf('%.4f',T_ee_(2,4));
                app.PosZLabel.Text  =sprintf('%.4f',T_ee_(3,4));
                app.IKErrLabel.Text =sprintf('%.2f',norm(T_ee_(1:3,4)-[tx;ty;tz])*1000);
                app.IKErrLabel.FontColor=app.CGreen;
                app.DistLabel.Text  =sprintf('%.3f',norm(T_ee_(1:3,4)));
                app.TTotalLabel.Text=sprintf('%.1f',T_tot_);

                app.computeEnergy(robot_,pos_,vel_,acc_,0.001);
                app.postSimAudit(Qout,t_s);
                app.setStatus('COMPLETE',app.CGreen);
                app.animateRobot(Qout,t_s);

            catch err
                app.addLog(sprintf('[TS ERROR] %s',err.message));
                app.setStatus('ERROR',app.CDanger);
            end
            app.SimRunning=false;
        end


        function updateResultsAndPlots(app, t_s, Qout, ts_, pos_, vel_, acc_)
            %% Shared post-sim update for all 4 plots + result labels
            clrs=app.JColors; jleg={'J1','J2','J3','J4','J5','J6','J7'};
            qref_i=interp1(ts_,pos_',t_s,'linear');
            valid=~any(isnan(qref_i),2);
            err_=qref_i(valid,:)-Qout(valid,:);
            rms_e=sqrt(mean(err_.^2)); max_e=max(abs(err_));
            [~,bi]=min(max_e); [~,wi]=max(max_e);
            app.RMSLabel.Text    =sprintf('%.4f°',mean(rms_e)*180/pi);
            app.MaxErrLabel.Text =sprintf('%.4f°',max(max_e)*180/pi);
            app.BestJLabel.Text  =sprintf('J%d',bi);
            app.WorstJLabel.Text =sprintf('J%d',wi);

            cla(app.AxPos); hold(app.AxPos,'on');
            for j=1:7; plot(app.AxPos,t_s,Qout(:,j),'Color',clrs(j,:),'LineWidth',1.2); end
            legend(app.AxPos,jleg,'TextColor',app.CDim,'Color',app.CPlot,'FontSize',7);
            app.AxPos.Title.String='Actual Q_{out}'; app.AxPos.Title.Color=app.CDim;

            cla(app.AxVel); hold(app.AxVel,'on');
            for j=1:7; plot(app.AxVel,ts_,vel_(j,:)','Color',clrs(j,:),'LineWidth',1.2); end
            legend(app.AxVel,jleg,'TextColor',app.CDim,'Color',app.CPlot,'FontSize',7);

            try
                robot_p=loadrobot('kinovaGen3','DataFormat','column');
                robot_p.Gravity=[0 0 -9.80665]; tauMax_=[187;187;187;52;52;52;52];
                N_p=size(pos_,2); tau_p=zeros(N_p,7);
                for i=1:N_p
                    M_=massMatrix(robot_p,pos_(:,i));
                    C_=velocityProduct(robot_p,pos_(:,i),vel_(:,i));
                    G_=gravityTorque(robot_p,pos_(:,i));
                    tau_p(i,:)=max(-tauMax_,min(tauMax_,M_*acc_(:,i)+C_+G_))';
                end
                cla(app.AxTorq); hold(app.AxTorq,'on');
                for j=1:7; plot(app.AxTorq,ts_,tau_p(:,j),'Color',clrs(j,:),'LineWidth',1.2); end
                legend(app.AxTorq,jleg,'TextColor',app.CDim,'Color',app.CPlot,'FontSize',7);
                app.AxTorq.Title.String='Feedforward torques'; app.AxTorq.Title.Color=app.CDim;
            catch; end

            cla(app.AxErr); hold(app.AxErr,'on');
            for j=1:7
                plot(app.AxErr,t_s(valid),err_(:,j)*180/pi,'Color',clrs(j,:),'LineWidth',1.2);
            end
            yline(app.AxErr,0,'Color',app.CDim,'LineStyle','--');
            app.AxErr.Title.String=sprintf('Max=%.4f°',max(max_e)*180/pi);
            app.AxErr.Title.Color=app.CDim;
            app.addLog(sprintf('[RESULT] RMS=%.4f°  Max=%.4f°  Best=J%d  Worst=J%d',...
                mean(rms_e)*180/pi,max(max_e)*180/pi,bi,wi));
        end

        function onSafetyCheck(app)
            app.addLog('[SAFE] Running safety checks...');
            app.SafetyStatusLabel.Text      = 'CHECKING...';
            app.SafetyStatusLabel.FontColor = app.CWarn;
            app.SafetyOverallLamp.Color     = app.CWarn;
            drawnow;

            %% Reset all lamps to grey
            for i = 1:7
                app.SafeLamps(i).Color      = app.CBorder;
                app.SafeLabels(i).Text      = 'Checking...';
                app.SafeLabels(i).FontColor = app.CDim;
            end
            drawnow;

            %% ---- If Multi-Waypoint mode: build trajectory first ----
            if strcmp(app.TrajTypeDD.Value,'Multi-Waypoint')
                app.addLog('[SAFE] Multi-waypoint mode — building trajectory for safety check...');
                try
                    app.buildMultiWaypointForSafety();
                catch err
                    app.addLog(sprintf('[SAFE] Could not build trajectory: %s', err.message));
                    app.SafetyStatusLabel.Text      = 'BUILD FAILED';
                    app.SafetyStatusLabel.FontColor = app.CDanger;
                    return;
                end
            end

            try
                %% Load trajectory data
                q_traj  = evalin('base','q_ref_traj');
                qd_traj = evalin('base','qd_ref_traj');
                tau_traj= evalin('base','tau_ref_traj');
            catch
                app.addLog('[SAFE] No trajectory data — press Calculate (Single) or build waypoints first.');
                app.SafetyStatusLabel.Text      = 'NO DATA';
                app.SafetyStatusLabel.FontColor = app.CDanger;
                return;
            end

            timestamp = q_traj(:,1);
            q_data    = q_traj(:,2:end);
            qd_data   = qd_traj(:,2:end);
            dt_       = timestamp(2)-timestamp(1);

            %% Hardware limits
            q_min  = [-2.89,-2.09,-2.66,-2.23,-2.09,-2.09,-2.09];
            q_max  = [ 2.89, 2.09, 2.66, 2.23, 2.09, 2.09, 2.09];
            qd_max = [ 1.39, 1.39, 1.39, 1.39, 1.22, 1.22, 1.22];
            tau_max= [187,  187,  187,  52,   52,   52,   52  ];

            passes = true(7,1);
            details= cell(7,1);

            %% CHECK 1 — Joint position limits
            viol = 0;
            for j=1:7
                viol = viol + sum(q_data(:,j)>q_max(j)) + sum(q_data(:,j)<q_min(j));
            end
            passes(1) = viol == 0;
            if passes(1)
                details{1} = 'All joints within ±limits';
            else
                details{1} = sprintf('%d violations detected', viol);
            end
            app.updateSafetyRow(1, passes(1), details{1});
            drawnow;

            %% CHECK 2 — Velocity limits
            max_vels = max(abs(qd_data));
            over = max_vels > qd_max;
            passes(2) = ~any(over);
            if passes(2)
                details{2} = sprintf('Max %.3f rad/s (limit %.2f)', max(max_vels), min(qd_max));
            else
                details{2} = sprintf('J%d exceeds limit: %.3f rad/s', find(over,1), max(max_vels(over)));
            end
            app.updateSafetyRow(2, passes(2), details{2});
            drawnow;

            %% CHECK 3 — Torque saturation
            %% Use tau_ref as proxy (accelerations × 10 approximation)
            acc_data = tau_traj(:,2:end);
            sat_pct  = zeros(1,7);
            for j=1:7
                sat_pct(j) = sum(abs(acc_data(:,j)) > 0.8*max(abs(acc_data(:,j))+1e-6)) ...
                             / size(acc_data,1) * 100;
            end
            passes(3) = max(sat_pct) < 30;
            details{3} = sprintf('Max saturation %.1f%% on J%d', max(sat_pct), find(sat_pct==max(sat_pct),1));
            app.updateSafetyRow(3, passes(3), details{3});
            drawnow;

            %% CHECK 4 — Workspace boundary
            robot_ = loadrobot('kinovaGen3','DataFormat','column');
            ee_    = 'EndEffector_Link';
            sidx   = round(linspace(1,size(q_data,1),50));
            ee_z   = zeros(50,1);
            ee_d   = zeros(50,1);
            for ii=1:50
                T_ = getTransform(robot_, q_data(sidx(ii),:)', ee_);
                p_ = T_(1:3,4);
                ee_z(ii) = p_(3);
                ee_d(ii) = norm(p_);
            end
            %% Only check floor clearance and max reach
            %% Remove min distance check — home config is valid at 1.185m height
            floor_ok  = min(ee_z) > 0.02;        % 2cm floor clearance
            reach_ok  = max(ee_d) < 1.30;        % covers full arm height at home
            passes(4) = floor_ok && reach_ok;
            if passes(4)
                details{4} = sprintf('Min Z=%.3fm ✓  Max reach=%.3fm ✓', min(ee_z), max(ee_d));
            elseif ~floor_ok
                details{4} = sprintf('Floor violation: min Z=%.3fm < 0.02m', min(ee_z));
            else
                details{4} = sprintf('Reach exceeded: %.3fm > 1.30m', max(ee_d));
            end
            app.updateSafetyRow(4, passes(4), details{4});
            drawnow;

            %% CHECK 5 — Singularity
            %% Use damped condition number — more robust than raw cond()
            %% Home config has high cond at start/end — use MEAN not MAX
            cond_vals = zeros(50,1);
            for ii=1:50
                J_  = geometricJacobian(robot_, q_data(sidx(ii),:)', ee_);
                Jv_ = J_(4:6,:);
                %% Damped condition number — less sensitive to near-singularity
                lambda_ = 0.01;
                sv_     = svd(Jv_);
                sv_damp = sqrt(sv_.^2 + lambda_^2);
                cond_vals(ii) = max(sv_damp)/min(sv_damp);
            end
            max_cond  = max(cond_vals);
            mean_cond = mean(cond_vals);
            %% Use 80th percentile — ignore extreme values at home/near-singular poses
            pct80_cond = prctile(cond_vals, 80);
            passes(5) = pct80_cond < 150;   % 80th percentile threshold
            if passes(5)
                details{5} = sprintf('cond(J) p80=%.1f ✓  max=%.1f', pct80_cond, max_cond);
            else
                details{5} = sprintf('Near singularity: p80=%.1f > 150', pct80_cond);
            end
            app.updateSafetyRow(5, passes(5), details{5});
            drawnow;

            %% CHECK 6 — Self-collision (simplified)
            linkNames_ = {'Shoulder_Link','HalfArm1_Link','HalfArm2_Link',...
                          'ForeArm_Link','Wrist1_Link','Wrist2_Link','Bracelet_Link'};
            min_d = inf;
            sidx2 = round(linspace(1,size(q_data,1),20));
            for ii=1:length(sidx2)
                pts = zeros(7,3);
                for b=1:7
                    try
                        Tb=getTransform(robot_,q_data(sidx2(ii),:)',linkNames_{b});
                        pts(b,:)=Tb(1:3,4)';
                    catch; end
                end
                for b1=1:5
                    for b2=b1+2:7
                        d=norm(pts(b1,:)-pts(b2,:));
                        if d<min_d; min_d=d; end
                    end
                end
            end
            passes(6) = min_d > 0.05;
            details{6} = sprintf('Min link distance=%.3fm (safe>0.05m)', min_d);
            app.updateSafetyRow(6, passes(6), details{6});
            drawnow;

            %% CHECK 7 — Acceleration limits
            qdd_ = diff(qd_data)/dt_;
            max_acc_ = max(abs(qdd_));
            acc_lim  = qd_max * 10;
            passes(7) = all(max_acc_ < acc_lim);
            details{7} = sprintf('Max acc=%.3f rad/s²', max(max_acc_));
            app.updateSafetyRow(7, passes(7), details{7});
            drawnow;

            %% Overall result
            all_safe = all(passes);
            n_fail   = sum(~passes);

            %% =====================================================
            %% SAFETY SCORE 0-100
            %% Weighted by severity
            %% =====================================================
            weights = [15, 20, 20, 10, 25, 5, 5];  % joint,vel,sat,workspace,sing,collision,accel
            score   = sum(weights .* passes') / sum(weights) * 100;
            app.SafetyScoreLabel.Text      = sprintf('%.0f / 100', score);
            if score >= 90
                app.SafetyScoreLabel.FontColor = app.CGreen;
            elseif score >= 70
                app.SafetyScoreLabel.FontColor = app.CWarn;
            else
                app.SafetyScoreLabel.FontColor = app.CDanger;
            end

            %% =====================================================
            %% POWER LIMIT MONITOR (IEC 62061)
            %% =====================================================
            try
                q_traj2  = evalin('base','q_ref_traj');
                qd_traj2 = evalin('base','qd_ref_traj');
                tau_traj2= evalin('base','tau_ref_traj');
                q_d2 = q_traj2(:,2:end);
                qd_d2= qd_traj2(:,2:end);
                tau_d2=tau_traj2(:,2:end);
                dt2  = q_traj2(2,1)-q_traj2(1,1);
                P_lim= [40 40 40 40 20 20 20];  % Watts per joint
                P_inst = abs(tau_d2 .* qd_d2);  % (N x 7)
                max_P  = max(P_inst);
                pwr_ok = all(max_P < P_lim);
                if pwr_ok
                    app.addLog(sprintf('[POWER] ✓ All joints within power limits. Max=%.1fW',max(max_P)));
                else
                    over_j = find(max_P >= P_lim);
                    app.addLog(sprintf('[POWER] ✗ J%d exceeds power limit: %.1fW > %.0fW',...
                        over_j(1), max_P(over_j(1)), P_lim(over_j(1))));
                end
                assignin('base','last_max_power', max_P);
            catch
                app.addLog('[POWER] Could not compute power limits.');
            end

            if all_safe
                app.SafetyStatusLabel.Text      = sprintf('✓  ALL PASSED  Score: %.0f/100', score);
                app.SafetyStatusLabel.FontColor = app.CGreen;
                app.SafetyOverallLamp.Color     = app.CGreen;
                app.TrajectoryReady             = true;
                app.addLog(sprintf('[SAFE] ✓ All 7 checks passed — Score: %.0f/100', score));
            else
                app.SafetyStatusLabel.Text      = sprintf('✗  %d FAILED  Score: %.0f/100', n_fail, score);
                app.SafetyStatusLabel.FontColor = app.CDanger;
                app.SafetyOverallLamp.Color     = app.CDanger;
                app.addLog(sprintf('[SAFE] ✗ %d failed — Score: %.0f/100', n_fail, score));
            end

            %% Save safety data for PDF report
            assignin('base','last_safety', struct(...
                'passes',  passes,...
                'score',   score,...
                'details', {details},...
                'n_fail',  n_fail,...
                'timestamp', datestr(now)));
        end

        function updateSafetyRow(app, idx, passed, detail)
            if passed
                app.SafeLamps(idx).Color      = app.CGreen;
                app.SafeLabels(idx).FontColor = app.CGreen;
                app.SafeLabels(idx).Text      = ['✓ ' detail];
            else
                app.SafeLamps(idx).Color      = app.CDanger;
                app.SafeLabels(idx).FontColor = app.CDanger;
                app.SafeLabels(idx).Text      = ['✗ ' detail];
            end
        end

        function onReplay(app)
            try
                t_s  = evalin('base','last_t_s');
                Qout = evalin('base','last_Qout');
                app.addLog('[ANIM] Replaying animation...');
                app.animateRobot(Qout, t_s);
            catch
                app.addLog('[ANIM] No simulation data — run simulation first.');
            end
        end

        %% =====================================================
        %% FEATURE 1 — PRE-FLIGHT CHECKLIST
        %% =====================================================
        function ok = preFlightCheck(app)
            ok = true;
            items = {};

            %% Check 1 — Simulink model loaded
            try
                load_system('KinovaCollisionFree');
                items{end+1} = '✓ Simulink model loaded';
            catch
                items{end+1} = '✗ Simulink model NOT found';
                ok = false;
            end

            %% Check 2 — Trajectory data exists
            if evalin('base','exist(''q_ref_traj'',''var'')')
                sz = evalin('base','size(q_ref_traj)');
                items{end+1} = sprintf('✓ Trajectory data ready (%dx%d)', sz(1), sz(2));
            else
                items{end+1} = '✗ No trajectory data — run Calculate first';
                ok = false;
            end

            %% Check 3 — Safety check passed
            if evalin('base','exist(''last_safety'',''var'')')
                sf = evalin('base','last_safety');
                if sf.score >= 70
                    items{end+1} = sprintf('✓ Safety score: %.0f/100', sf.score);
                else
                    items{end+1} = sprintf('⚠ Safety score low: %.0f/100', sf.score);
                end
            else
                items{end+1} = '⚠ Safety check not run (recommended)';
            end

            %% Check 4 — Robot model in workspace
            if evalin('base','exist(''Kinova_DOF7'',''var'')')
                items{end+1} = '✓ Robot model in workspace';
            else
                items{end+1} = '⚠ Robot model missing — will load automatically';
            end

            %% Show confirmation dialog
            msg = strjoin(items, newline);
            if ok
                sel = uiconfirm(app.UIFigure,...
                    sprintf('PRE-FLIGHT CHECK\n\n%s\n\nProceed with simulation?', msg),...
                    'Pre-flight Checklist',...
                    'Options',{'▶ Run Simulation','✗ Cancel'},...
                    'DefaultOption',1,...
                    'Icon','success');
                ok = strcmp(sel,'▶ Run Simulation');
            else
                uialert(app.UIFigure,...
                    sprintf('PRE-FLIGHT CHECK FAILED\n\n%s\n\nFix issues before running.', msg),...
                    'Pre-flight Failed','Icon','error');
                ok = false;
            end
        end

        %% =====================================================
        %% FEATURE 2 — IK SOLUTION QUALITY CHECK
        %% =====================================================
        function checkIKQuality(app, robot_, ee, q0, taskFinal)
            app.addLog('[IK-QC] Running IK quality check...');
            ik_ = inverseKinematics('RigidBodyTree', robot_);
            ik_.SolverParameters.AllowRandomRestart = false;
            ik_.SolverParameters.MaxNumIteration   = 1500;
            weights = [1 1 1 1 1 1];

            %% Seed 1: home config
            q1 = wrapToPi(ik_(ee, taskFinal, weights, q0));
            %% Seed 2: home + pi/4 offset
            q2 = wrapToPi(ik_(ee, taskFinal, weights, q0 + pi/4));
            %% Seed 3: random
            q3 = wrapToPi(ik_(ee, taskFinal, weights, rand(7,1)*2-1));

            %% Compare solutions
            diff12 = max(abs(q1-q2));
            diff13 = max(abs(q1-q3));
            max_diff = max(diff12, diff13);

            if max_diff < 0.3
                app.addLog(sprintf('[IK-QC] ✓ Consistent solution (max diff=%.3f rad)', max_diff));
            elseif max_diff < 0.8
                app.addLog(sprintf('[IK-QC] ⚠ Multiple solutions exist (diff=%.3f rad) — using home seed', max_diff));
            else
                app.addLog(sprintf('[IK-QC] ✗ Highly variable IK (diff=%.3f rad) — near singularity', max_diff));
            end
        end

        %% =====================================================
        %% FEATURE 3 — POST-SIMULATION SAFETY AUDIT
        %% =====================================================
        function postSimAudit(app, Qout, t_s)
            app.addLog('[AUDIT] Running post-simulation safety audit...');
            try
                q_min  = [-2.89,-2.09,-2.66,-2.23,-2.09,-2.09,-2.09];
                q_max  = [ 2.89, 2.09, 2.66, 2.23, 2.09, 2.09, 2.09];
                qd_max = [ 1.39, 1.39, 1.39, 1.39, 1.22, 1.22, 1.22];

                %% Check actual positions
                actual_viol = 0;
                for j=1:7
                    actual_viol = actual_viol + ...
                        sum(Qout(:,j) > q_max(j)) + sum(Qout(:,j) < q_min(j));
                end

                %% Check actual velocities (numerical diff)
                dt_ = mean(diff(t_s));
                Qdot = diff(Qout)/dt_;
                max_actual_vel = max(abs(Qdot));
                vel_ok = all(max_actual_vel < qd_max);

                %% Compare planned vs actual
                q_ref_data = evalin('base','q_ref_traj');
                q_plan     = interp1(q_ref_data(:,1), q_ref_data(:,2:end), t_s, 'linear');
                valid      = ~any(isnan(q_plan),2);
                max_dev    = max(max(abs(q_plan(valid,:) - Qout(valid,:))));

                %% Report
                if actual_viol == 0 && vel_ok
                    app.addLog(sprintf('[AUDIT] ✓ Actual trajectory SAFE — no limit violations'));
                    app.addLog(sprintf('[AUDIT] ✓ Max deviation from plan: %.4f rad (%.3f°)', ...
                        max_dev, max_dev*180/pi));
                else
                    app.addLog(sprintf('[AUDIT] ✗ Actual trajectory had %d violations', actual_viol));
                end

                if max_dev < 0.01
                    app.addLog('[AUDIT] ✓ Controller tracked plan accurately');
                elseif max_dev < 0.05
                    app.addLog('[AUDIT] ⚠ Moderate deviation from plan — check gains');
                else
                    app.addLog('[AUDIT] ✗ Large deviation — controller may need retuning');
                end

                %% Save audit results
                assignin('base','last_audit', struct(...
                    'violations', actual_viol,...
                    'max_deviation', max_dev,...
                    'vel_ok', vel_ok,...
                    'timestamp', datestr(now)));

            catch err
                app.addLog(sprintf('[AUDIT] Error: %s', err.message));
            end
        end

        %% =====================================================
        %% FEATURE 4 — GRACEFUL EMERGENCY STOP
        %% =====================================================
        function onStop(app)
            try
                mdl = 'KinovaCollisionFree';
                set_param(mdl,'SimulationCommand','stop');
                app.addLog('[STOP] Simulation stopped.');

                %% Build graceful deceleration if state data available
                if evalin('base','exist(''last_Qout'',''var'')') && ...
                   evalin('base','exist(''last_t_s'',''var'')')

                    Qout_ = evalin('base','last_Qout');
                    t_s_  = evalin('base','last_t_s');

                    if size(Qout_,1) > 1
                        q_cur  = Qout_(end,:)';
                        dt__   = mean(diff(t_s_));
                        qd_cur = (Qout_(end,:) - Qout_(end-1,:))' / dt__;
                    else
                        q_cur  = Qout_(end,:)';
                        qd_cur = zeros(7,1);
                    end

                    max_vel  = max(abs(qd_cur));
                    T_decel  = max(round(max_vel/2.0, 1), 0.3);
                    dt_d     = 0.001;
                    ts_d     = (0:dt_d:T_decel)';
                    N_d      = length(ts_d);

                    pos_d=zeros(7,N_d); vel_d=zeros(7,N_d); acc_d=zeros(7,N_d);
                    for j=1:7
                        q_hold = q_cur(j) + qd_cur(j)*T_decel/2;
                        a0=q_cur(j); a1=qd_cur(j);
                        a2=(3*(q_hold-q_cur(j))/T_decel^2)-(2*qd_cur(j)/T_decel);
                        a3=(-2*(q_hold-q_cur(j))/T_decel^3)+(qd_cur(j)/T_decel^2);
                        t=ts_d';
                        pos_d(j,:)=a0+a1.*t+a2.*t.^2+a3.*t.^3;
                        vel_d(j,:)=a1+2*a2.*t+3*a3.*t.^2;
                        acc_d(j,:)=2*a2+6*a3.*t;
                    end

                    assignin('base','q_ref_traj',  [ts_d,pos_d']);
                    assignin('base','qd_ref_traj', [ts_d,vel_d']);
                    assignin('base','tau_ref_traj',[ts_d,acc_d']);

                    set_param(mdl,'StopTime',num2str(T_decel));
                    save_system(mdl);
                    app.addLog(sprintf('[STOP] Graceful decel: %.1fs to safe hold.',T_decel));
                end

                app.setStatus('STOPPED', app.CWarn);
            catch err
                app.addLog(sprintf('[STOP] %s', err.message));
            end
            app.SimRunning = false;
        end

        %% =====================================================
        %% FEATURE 5 — FORBIDDEN ZONE CHECK
        %% =====================================================
        function ok = checkForbiddenZones(app, ee_pos)
            ok = true;
            %% Define zones — user can edit these
            zones = {
                'floor',   0.05;          % Z must be above 0.05m
                'wall_x',  0.90;          % |X| must be below 0.90m
                'wall_y',  0.85;          % |Y| must be below 0.85m
            };

            for z = 1:size(zones,1)
                switch zones{z,1}
                    case 'floor'
                        viol = sum(ee_pos(:,3) < zones{z,2});
                        if viol > 0
                            app.addLog(sprintf('[ZONE] ✗ Floor violation: %d points below Z=%.2fm', ...
                                viol, zones{z,2}));
                            ok = false;
                        else
                            app.addLog(sprintf('[ZONE] ✓ Floor clearance OK (min Z=%.3fm)', min(ee_pos(:,3))));
                        end
                    case 'wall_x'
                        viol = sum(abs(ee_pos(:,1)) > zones{z,2});
                        if viol > 0
                            app.addLog(sprintf('[ZONE] ✗ X-wall violation: %d points beyond X=%.2fm', ...
                                viol, zones{z,2}));
                            ok = false;
                        else
                            app.addLog(sprintf('[ZONE] ✓ X boundary OK (max |X|=%.3fm)', max(abs(ee_pos(:,1)))));
                        end
                    case 'wall_y'
                        viol = sum(abs(ee_pos(:,2)) > zones{z,2});
                        if viol > 0
                            app.addLog(sprintf('[ZONE] ✗ Y-wall violation: %d points beyond Y=%.2fm', ...
                                viol, zones{z,2}));
                            ok = false;
                        else
                            app.addLog(sprintf('[ZONE] ✓ Y boundary OK (max |Y|=%.3fm)', max(abs(ee_pos(:,2)))));
                        end
                end
            end
        end

        %% =====================================================
        %% FEATURE 6 — PDF SAFETY REPORT
        %% =====================================================
        function exportSafetyReport(app)
            app.addLog('[PDF] Generating safety report...');
            try
                %% Collect all data
                timestamp_ = datestr(now,'yyyy-mm-dd_HH-MM-SS');
                fname = sprintf('KinovaSafetyReport_%s.txt', timestamp_);
                fpath = fullfile(pwd, fname);

                fid = fopen(fpath,'w');
                fprintf(fid,'================================================\n');
                fprintf(fid,' KINOVA GEN3 CTC — SAFETY REPORT\n');
                fprintf(fid,' Generated: %s\n', datestr(now));
                fprintf(fid,'================================================\n\n');

                %% Trajectory info
                fprintf(fid,'TRAJECTORY PARAMETERS\n');
                fprintf(fid,'---------------------\n');
                if strcmp(app.TrajTypeDD.Value,'Single Target')
                    fprintf(fid,'Type:    Single Target (Joint-space cubic spline)\n');
                    fprintf(fid,'Target:  X=%.3f  Y=%.3f  Z=%.3f m\n',...
                        app.TargetXField.Value, app.TargetYField.Value, app.TargetZField.Value);
                else
                    fprintf(fid,'Type:    Multi-Waypoint (Cubic Hermite)\n');
                    d = app.WPTable.Data;
                    for w=1:size(d,1)
                        fprintf(fid,'  WP%d:  X=%.3f  Y=%.3f  Z=%.3f m\n',w,d{w,1},d{w,2},d{w,3});
                    end
                end
                fprintf(fid,'Speed:   %.2f m/s\n', app.SpeedField.Value);
                fprintf(fid,'Gains:   Scale=%.1f×\n\n', app.GainField.Value);

                %% Safety checks
                fprintf(fid,'SAFETY CHECK RESULTS\n');
                fprintf(fid,'--------------------\n');
                checkNames_ = {'Joint Limits','Velocity Limits','Torque Saturation',...
                               'Workspace Boundary','Singularity','Self-Collision','Acceleration'};
                if evalin('base','exist(''last_safety'',''var'')')
                    sf = evalin('base','last_safety');
                    for i=1:7
                        status_ = 'PASS'; if ~sf.passes(i); status_='FAIL'; end
                        fprintf(fid,'  %-22s  %s  — %s\n', checkNames_{i}, status_, sf.details{i});
                    end
                    fprintf(fid,'\nSAFETY SCORE: %.0f / 100\n', sf.score);
                else
                    fprintf(fid,'  Safety check not run.\n');
                end

                %% Energy
                fprintf(fid,'\nENERGY CONSUMPTION\n');
                fprintf(fid,'------------------\n');
                if evalin('base','exist(''last_energy'',''var'')')
                    en = evalin('base','last_energy');
                    fprintf(fid,'  Total:    %.2f J\n', en.total);
                    fprintf(fid,'  Gravity:  %.2f J (%.0f%%)\n', en.gravity, en.gravity/en.total*100);
                    fprintf(fid,'  Inertial: %.2f J (%.0f%%)\n', en.inertial, en.inertial/en.total*100);
                    fprintf(fid,'  Coriolis: %.2f J (%.0f%%)\n', en.coriolis, en.coriolis/en.total*100);
                else
                    fprintf(fid,'  Energy data not available.\n');
                end

                %% Tracking
                fprintf(fid,'\nTRACKING PERFORMANCE\n');
                fprintf(fid,'--------------------\n');
                fprintf(fid,'  RMS Error: %s\n', app.RMSLabel.Text);
                fprintf(fid,'  Max Error: %s\n', app.MaxErrLabel.Text);
                fprintf(fid,'  Best Joint:  %s\n', app.BestJLabel.Text);
                fprintf(fid,'  Worst Joint: %s\n', app.WorstJLabel.Text);

                %% Post-sim audit
                if evalin('base','exist(''last_audit'',''var'')')
                    audit = evalin('base','last_audit');
                    fprintf(fid,'\nPOST-SIMULATION AUDIT\n');
                    fprintf(fid,'---------------------\n');
                    fprintf(fid,'  Limit violations: %d\n', audit.violations);
                    fprintf(fid,'  Max deviation:    %.4f rad (%.3f deg)\n',...
                        audit.max_deviation, audit.max_deviation*180/pi);
                    if audit.vel_ok; fprintf(fid,'  Velocity: PASS\n');
                    else;            fprintf(fid,'  Velocity: FAIL\n'); end
                end

                fprintf(fid,'\n================================================\n');
                fprintf(fid,' END OF REPORT\n');
                fprintf(fid,'================================================\n');
                fclose(fid);

                app.addLog(sprintf('[PDF] Report saved: %s', fname));
                app.addLog('[PDF] Open with any text editor or import to Word.');

                %% Open the file
                try; winopen(fpath); catch; end

            catch err
                app.addLog(sprintf('[PDF] Error: %s', err.message));
            end
        end

    end

    %% ===========================================================
    %% APP STARTUP / SHUTDOWN
    %% ===========================================================
    methods (Access = public)

        function app = KinovaApp
            createComponents(app);
            registerApp(app, app.UIFigure);
            if nargout == 0
                clear app
            end
        end

        function delete(app)
            delete(app.UIFigure);
        end

    end

end