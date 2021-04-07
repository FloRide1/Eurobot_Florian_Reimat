using Constants;
using EventArgsLibrary;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Timers;
using Utilities;
using WorldMap;


namespace StrategyManagerProjetEtudiantNS
{
    public class StrategyEurobot : StrategyGenerique
    {
        Stopwatch sw = new Stopwatch();

        public PointD robotDestination = new PointD(0, 0);
        PlayingSide playingSide = PlayingSide.Left;     

        TaskDemoMove taskDemoMove;
        TaskDemoMessage taskDemoMessage;
        TaskDestination taskDestination;

        int robotId, teamId;

        Timer GhostTimer;

        public StrategyEurobot(int robotId, int teamId, string multicastIpAddress) : base(robotId, teamId, multicastIpAddress)
        {
            taskDemoMove = new TaskDemoMove(this);
            taskDemoMessage = new TaskDemoMessage(this);
            taskDestination = new TaskDestination(this);

            this.robotId = robotId;
            this.teamId = teamId;
            localWorldMap = new LocalWorldMap(robotId, teamId);

            GhostTimer = new Timer(10);
            GhostTimer.Elapsed += OnGhostTimerCalculationOrder;
        }

        public override void InitStrategy()
        {
            /// Obtenus directement à partir du script Matlab
            OnOdometryPointToMeter(ConstVar.EUROBOT_ODOMETRY_POINT_TO_METER);
            On2WheelsAngleSetup(- ConstVar.EUROBOT_WHEELS_ANGLE, ConstVar.EUROBOT_WHEELS_ANGLE);
            On2WheelsToPolarMatrixSetup(ConstVar.EUROBOT_MATRIX_X_COEFF, - ConstVar.EUROBOT_MATRIX_X_COEFF, ConstVar.EUROBOT_MATRIX_THETA_COEFF, ConstVar.EUROBOT_MATRIX_THETA_COEFF);

            /// Use only when the robot is disconnect
            GhostTimer.Start();
        }




        /*********************************** Events reçus **********************************************/

        public override void OnGhostLocationReached(object sender, Location location)
        {
            if (taskDestination != null)
                taskDestination.TaskReached();
        }

        private void OnGhostTimerCalculationOrder(object sender, ElapsedEventArgs e)
        {
            OnUpdateGhostCalculationOrder();
        }

        /*********************************** Events de sortie **********************************************/
               
    }
}
