using EventArgsLibrary;
using MessageDecoder;
using MessageEncoder;
using MessageGeneratorNS;
using MessageProcessorNS;
using RobotInterface;
using System;
using System.Runtime.InteropServices;
using System.Threading;
using USBVendorNS;
using XBoxControllerNS;
using Constants;
using StrategyManagerProjetEtudiantNS;
using SciChart.Charting.Visuals;
using ConsoleFormatNS;
using WorldMap;
using Utilities;
using Lidar;
using LidarProcessNS;
using TrajectoryPlannerNs;

namespace RobotEurobot2Roues
{
    class RobotEurobot2Roues
    {
        static USBVendor usbDriver;
        static MsgDecoder msgDecoder;
        static MsgEncoder msgEncoder;
        static MsgGenerator msgGenerator;
        static MsgProcessor msgProcessor;
        static XBoxController xBoxManette;
        static StrategyGenerique strategyManager;
        static LidarDevice lidar;
        static LidarProcess lidarProcess;
        static LocalWorldMap localWorldMap;
        static TrajectoryPlanner trajectoryPlanner;

        static WpfRobot2RouesInterface interfaceRobot;
        static GameMode competition = GameMode.Eurobot;

        static bool usingXBoxController;
        
        static object ExitLock = new object();

        static void Main(string[] args)
        {
            ConsoleFormat.InitMainConsole();


            /// Enregistrement de la license SciChart en début de code
            
            SciChartSurface.SetRuntimeLicenseKey(ConstVar.SCICHART_RUNTIME_KEY);
            ConsoleFormat.SetupScichartLicenceKey();

            /// Initialisation des modules utilisés dans le robot
            int robotId = (int)RobotId.Robot1;
            int teamId = (int)TeamId.Team1;

            usbDriver = new USBVendor();
            msgDecoder = new MsgDecoder();
            msgEncoder = new MsgEncoder();
            msgGenerator = new MsgGenerator();
            msgProcessor = new MsgProcessor(robotId, competition);
            ConsoleFormat.SetupAllCommunication();

            xBoxManette = new XBoxController(robotId);
            ConsoleFormat.SetupXboxController();

            strategyManager = new StrategyEurobot(robotId, teamId, "224.16.32.79");

            #region Communication to Low Lvl
            /// Création des liens entre module, sauf depuis et vers l'interface graphique           
            usbDriver.OnUSBuffReceivedEvent += msgDecoder.BuffReceived;                                    // Transmission des messages reçus par l'USB au Message Decoder
            msgDecoder.OnCorrectMessageReceivedEvent += msgProcessor.ProcessRobotDecodedMessage;            // Transmission les messages décodés par le Message Decoder au Message Processor
            msgGenerator.OnMessageToRobotGeneratedEvent += msgEncoder.EncodeAndSendMessage;                 // Envoi des messages du générateur de message à l'encoder
            msgEncoder.OnSendMessageEvent += usbDriver.SendUSBMessage;                                      // Envoi des messages en USB depuis le message encoder
            #endregion

            #region Console
            // Control:
            bool hex_viewer = false;
            bool hex_sender = false;
            bool hex_viewer_error = true;
            bool hex_sender_error = false;
            bool hex_processor = false;
            bool hex_generator = false;
            #region USB Vendor
            usbDriver.OnDeviceAddedEvent += ConsoleFormat.PrintNewDeviceAdded;
            usbDriver.OnDeviceRemovedEvent += ConsoleFormat.PrintDeviceRemoved;
            usbDriver.OnUsbVendorExeptionEvent += ConsoleFormat.PrintUsbErrorExeption;
            #endregion

            #region Hex Viewer
            if (hex_viewer)
            {
                msgDecoder.OnUnknowByteEvent += ConsoleFormat.PrintUnknowByte;
                msgDecoder.OnSOFByteReceivedEvent += ConsoleFormat.PrintSOF;
                msgDecoder.OnFunctionMSBByteReceivedEvent += ConsoleFormat.PrintFunctionMSB;
                msgDecoder.OnFunctionLSBByteReceivedEvent += ConsoleFormat.PrintFunctionLSB;
                msgDecoder.OnPayloadLenghtMSBByteReceivedEvent += ConsoleFormat.PrintLenghtMSB;
                msgDecoder.OnPayloadLenghtLSBByteReceivedEvent += ConsoleFormat.PrintLenghtLSB;
                msgDecoder.OnPayloadByteReceivedEvent += ConsoleFormat.PrintPayloadByte;
                msgDecoder.OnCorrectMessageReceivedEvent += ConsoleFormat.PrintCorrectChecksum;
                msgDecoder.OnErrorMessageReceivedEvent += ConsoleFormat.PrintWrongChecksum;
            }
            #endregion

            #region Hex Viewer Error
            if (hex_viewer_error)
            {
                msgDecoder.OnOverLenghtMessageEvent += ConsoleFormat.PrintOverLenghtWarning;
                msgDecoder.OnUnknowFunctionEvent += ConsoleFormat.PrintUnknowFunctionReceived;
                msgDecoder.OnWrongLenghtFunctionEvent += ConsoleFormat.PrintWrongFonctionLenghtReceived;
            }
            #endregion

            #region Hex Sender
            if (hex_sender)
            {
                msgEncoder.OnSendMessageEvent += ConsoleFormat.PrintSendMsg;
            }
            
            #endregion

            #region Hex Sender Error
            if (hex_sender_error)
            {
                msgEncoder.OnSerialDisconnectedEvent += ConsoleFormat.PrintOnSerialDisconnectedError;
                msgEncoder.OnUnknownFunctionSentEvent += ConsoleFormat.PrintUnknowFunctionSent;
                msgEncoder.OnWrongPayloadSentEvent += ConsoleFormat.PrintWrongFunctionLenghtSent;
            }
            #endregion
            #endregion

            #region Lidar
            lidar = new SickLidar(17422959); // 18110177
            lidarProcess = new LidarProcess(robotId, teamId);

            lidar.OnLidarDeviceConnectedEvent += lidarProcess.OnNewLidarConnected;
            lidar.OnLidarDeviceConnectedEvent += ConsoleFormat.NewLidarDeviceConnected;
            lidar.PointsAvailable += lidarProcess.OnRawPointAvailable;
            
            lidar.Start();
            #endregion

            #region Local World Map
            localWorldMap = new LocalWorldMap(robotId, teamId);
            localWorldMap.OnUpdateRobotLocationEvent += lidarProcess.OnRobotLocation;
            

            lidarProcess.OnRawLidarPointXYEvent += localWorldMap.OnLidarRawPointReceived;
            lidarProcess.OnProcessLidarXYDataEvent += localWorldMap.OnLidarProcessedPointReceived;
            lidarProcess.OnProcessLidarLineDataEvent += localWorldMap.OnLidarProcessedLineReceived;
            lidarProcess.OnProcessLidarCupDataEvent += localWorldMap.OnLidarProcessedCupReceived;
            //lidarProcess.OnProcessLidarObjectsDataEvent += localWorldMap.OnLidarProcesObjectsReceived;

            localWorldMap.OnLocalWorldMapEvent += strategyManager.OnLocalWorldMapReceived;
            
            localWorldMap.Init();
            #endregion

            #region TrajectoryPlanner
            trajectoryPlanner = new TrajectoryPlanner(robotId);

            trajectoryPlanner.OnNewGhostLocationEvent += localWorldMap.OnGhostLocation;
            trajectoryPlanner.OnDestinationReachedEvent += strategyManager.OnGhostLocationReached;

            #endregion

            #region Strategy /!\ Need to be Last /!\
            strategyManager.On2WheelsToPolarMatrixSetupEvent += msgGenerator.GenerateMessage2WheelsToPolarMatrixSet;   //Transmission des messages de set-up de la matrice de transformation moteurindepeandt -> polaire en embarqué
            strategyManager.On2WheelsAngleSetupEvent += msgGenerator.GenerateMessage2WheelsAngleSet;                   //Transmission des messages de set-up de la config angulaire des roues en embarqué
            strategyManager.OnOdometryPointToMeterSetupEvent += msgGenerator.GenerateMessageOdometryPointToMeter;      //Transmission des messages de set-up du coeff pointToMeter en embarqué

            strategyManager.OnDestinationReachedEvent += localWorldMap.OnDestinationReached;
            strategyManager.OnWaypointsReachedEvent += localWorldMap.OnWaypointReached;

            strategyManager.OnSetActualLocationEvent += trajectoryPlanner.OnUpdateActualLocation;
            strategyManager.OnSetWantedLocationEvent += trajectoryPlanner.OnUpdateWantedDestination;
            strategyManager.OnGhostCalculationBeginEvent += trajectoryPlanner.OnLaunchCalculation;

            strategyManager.OnUpdateGhostCalculationOrderEvent += trajectoryPlanner.OnCalculateGhostMovement;

            ConsoleFormat.PrintStrategyBoot();
            strategyManager.InitStrategy(); //à faire après avoir abonné les events !
            #endregion
                 



            StartRobotInterface();            
            ConsoleFormat.EndMainBootSequence();

            




            while (!exitSystem)
            {
                Thread.Sleep(500);
            }
        }


        static Thread t1;
        static void StartRobotInterface()
        {
            ConsoleFormat.StartRobotInterface();
            t1 = new Thread(() =>
            {
                //Attention, il est nécessaire d'ajouter PresentationFramework, PresentationCore, WindowBase and your wpf window application aux ressources.
                interfaceRobot = new WpfRobot2RouesInterface(competition);
                interfaceRobot.Loaded += RegisterRobotInterfaceEvents;
                interfaceRobot.ShowDialog();


            });
            t1.SetApartmentState(ApartmentState.STA);
            t1.Start();

            
        }

        static void RegisterRobotInterfaceEvents(object sender, EventArgs e)
        {
            /// Principe général des events :
            /// Sur evenement xx        -->>        Action a effectuer
            /// 

            #region From uC
            /// Affichage des évènements en provenance du uC
            msgGenerator.OnMessageToDisplaySpeedPolarPidSetupEvent += interfaceRobot.OnMessageToDisplayPolarSpeedPidSetupReceived;
            msgGenerator.OnMessageToDisplaySpeedIndependantPidSetupEvent += interfaceRobot.OnMessageToDisplayIndependantSpeedPidSetupReceived;

            msgProcessor.OnMotorsCurrentsFromRobotGeneratedEvent += interfaceRobot.UpdateMotorsCurrentsOnGraph;
            msgProcessor.OnEncoderRawDataFromRobotGeneratedEvent += interfaceRobot.UpdateMotorsEncRawDataOnGraph;

            msgProcessor.OnEnableDisableMotorsACKFromRobotGeneratedEvent += interfaceRobot.ActualizeEnableDisableMotorsButton;

            msgProcessor.OnAsservissementModeStatusFromRobotGeneratedEvent += interfaceRobot.UpdateAsservissementMode;
            msgProcessor.OnSpeedPolarOdometryFromRobotEvent += interfaceRobot.UpdateSpeedPolarOdometryOnInterface;

            msgProcessor.OnIndependantOdometrySpeedFromRobotEvent += interfaceRobot.UpdateSpeedIndependantOdometryOnInterface;
            msgProcessor.OnSpeedPolarPidErrorCorrectionConsigneDataFromRobotGeneratedEvent += interfaceRobot.UpdateSpeedPolarPidErrorCorrectionConsigneDataOnGraph;
            msgProcessor.OnSpeedIndependantPidErrorCorrectionConsigneDataFromRobotGeneratedEvent += interfaceRobot.UpdateSpeedIndependantPidErrorCorrectionConsigneDataOnGraph;
            msgProcessor.OnSpeedPolarPidCorrectionDataFromRobotEvent += interfaceRobot.UpdateSpeedPolarPidCorrectionData;
            msgProcessor.OnSpeedIndependantPidCorrectionDataFromRobotEvent += interfaceRobot.UpdateSpeedIndependantPidCorrectionData;

            msgProcessor.OnErrorTextFromRobotGeneratedEvent += interfaceRobot.AppendConsole;
            msgProcessor.OnPowerMonitoringValuesFromRobotGeneratedEvent += interfaceRobot.UpdatePowerMonitoringValues;
            msgProcessor.OnEnableMotorCurrentACKFromRobotGeneratedEvent += interfaceRobot.ActualizeEnableMotorCurrentCheckBox;
            msgProcessor.OnEnableAsservissementDebugDataACKFromRobotEvent += interfaceRobot.ActualizeEnableAsservissementDebugDataCheckBox;
            msgProcessor.OnEnablePowerMonitoringDataACKFromRobotGeneratedEvent += interfaceRobot.ActualizEnablePowerMonitoringCheckBox;

            msgProcessor.OnMessageCounterEvent += interfaceRobot.MessageCounterReceived;
            msgGenerator.OnSetSpeedConsigneToRobotReceivedEvent += interfaceRobot.UpdatePolarSpeedConsigneOnGraph; //Valable quelque soit la source des consignes vitesse
            #endregion
            #region Order From GUI
            /// Envoi des ordres en provenance de l'interface graphique
            interfaceRobot.OnEnableDisableMotorsFromInterfaceGeneratedEvent += msgGenerator.GenerateMessageEnableDisableMotors;
            interfaceRobot.OnEnableDisableControlManetteFromInterfaceGeneratedEvent += ChangeUseOfXBoxController;
            interfaceRobot.OnSetAsservissementModeFromInterfaceGeneratedEvent += msgGenerator.GenerateMessageSetAsservissementMode;
            interfaceRobot.OnEnableEncodersRawDataFromInterfaceGeneratedEvent += msgGenerator.GenerateMessageEnableEncoderRawData;
            interfaceRobot.OnEnableMotorCurrentDataFromInterfaceGeneratedEvent += msgGenerator.GenerateMessageEnableMotorCurrentData;
            interfaceRobot.OnEnableMotorsSpeedConsigneDataFromInterfaceGeneratedEvent += msgGenerator.GenerateMessageEnableMotorSpeedConsigne;
            interfaceRobot.OnSetRobotPIDFromInterfaceGeneratedEvent += msgGenerator.GenerateMessageSetupSpeedPolarPIDToRobot;
            interfaceRobot.OnEnableSpeedPIDEnableDebugInternalFromInterfaceGeneratedEvent += msgGenerator.GenerateMessageSpeedPIDEnableDebugInternal;
            interfaceRobot.OnEnableSpeedPIDEnableDebugErrorCorrectionConsigneFromInterfaceEvent += msgGenerator.GenerateMessageSpeedPIDEnableDebugErrorCorrectionConsigne;
            interfaceRobot.OnEnablePowerMonitoringDataFromInterfaceGeneratedEvent += msgGenerator.GenerateMessageEnablePowerMonitoring;

            interfaceRobot.OnWaypointLeftDoubleClick += localWorldMap.AddNewWaypointsEvent;
            interfaceRobot.OnWaypointRightClick += localWorldMap.SetDestinationLocationEvent;
            interfaceRobot.OnWaypointWheelClick += localWorldMap.ResetWaypointDestinationEvent;
            #endregion

            #region Msg
            /// Affichage des infos en provenance du décodeur de message
            msgDecoder.OnCorrectMessageReceivedEvent += interfaceRobot.DisplayMessageDecoded;
            msgDecoder.OnErrorMessageReceivedEvent += interfaceRobot.DisplayMessageDecodedError;
            #endregion
            #region Lidar
            lidarProcess.OnRawLidarDataEvent += interfaceRobot.OnRawLidarDataReceived;
            lidarProcess.OnProcessLidarDataEvent += interfaceRobot.OnProcessLidarDataReceived;
            #endregion

            /// Affichage des infos en provenance du strategyManager
            strategyManager.OnTextMessageEvent += interfaceRobot.AppendConsole;

            
            localWorldMap.OnLocalWorldMapEvent += interfaceRobot.OnLocalWorldMapWayPointEvent;
            localWorldMap.OnLocalWorldMapEvent += interfaceRobot.OnLocalWorldMapStrategyEvent;

            interfaceRobot.OnGameStateEditionEvent += localWorldMap.OnGameStateChange;


        }

        static void ChangeUseOfXBoxController(object sender, BoolEventArgs e)
        {
            ConfigControlEvents(e.value);
        }

        private static void ConfigControlEvents(bool useXBoxController)
        {
            usingXBoxController = useXBoxController;
            if (usingXBoxController)
            {                
                xBoxManette.OnSpeedConsigneEvent += msgGenerator.GenerateMessageSetSpeedConsigneToRobot;                
            }
            else
            {
                xBoxManette.OnSpeedConsigneEvent -= msgGenerator.GenerateMessageSetSpeedConsigneToRobot;
            }
        }

        /******************************************* Trap app termination ***************************************/
        static bool exitSystem = false;
        [DllImport("Kernel32")]
        private static extern bool SetConsoleCtrlHandler(EventHandler handler, bool add);
        enum CtrlType
        {
            CTRL_C_EVENT = 0,
            CTRL_BREAK_EVENT = 1,
            CTRL_CLOSE_EVENT = 2,
            CTRL_LOGOFF_EVENT = 3,
            CTRL_SHUTDOWN_EVENT = 4
        }

        private delegate bool EventHandler(CtrlType sig);
        static EventHandler _handler;
        //Gestion de la terminaison de l'application de manière propre
        private static bool Handler(CtrlType sig)
        {
            // Console.WriteLine("Existing on CTRL+C or process kill or shutdown...");

            //Nettoyage des process à faire ici
            //serialPort1.Close();

            // Console.WriteLine("Nettoyage effectué");
            exitSystem = true;

            //Sortie
            Environment.Exit(-1);
            return true;
        }
    }
}
