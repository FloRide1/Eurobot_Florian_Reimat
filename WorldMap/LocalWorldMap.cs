using Constants;
using System;
using System.Collections.Generic;
using Utilities;
using EventArgsLibrary;
using System.Windows;
using Lidar;

namespace WorldMap
{
    public class LocalWorldMap
    {
        #region Params
        private const double MINIMAL_WORLD_HISTORICAL_DIST = 0.010d;

        public virtual int RobotId { get; set; }
        public virtual int TeamId { get; set; }
        public virtual Location RobotLocation { get; set; }
        public virtual string messageDisplay { get; set; }

        public virtual PlayingSide playingSide { get; set; }
        public virtual Location RobotGhostLocation { get; set; }
        public virtual Location DestinationLocation { get; set; }
        public virtual List<PointD> WaypointLocations { get; set; }
        public virtual List<Location> RobotHistorical { get; set; }
        public virtual List<Location> BallLocationList { get; set; }
        public virtual List<LocationExtended> ObstaclesLocationList { get; set; }
        public virtual List<PolarPointRssi> LidarPoints { get; set; }
        public virtual List<PointD> LidarMapRaw { get; set; }
        public virtual List<PointD> LidarMapProcessed { get; set; }
        public virtual List<PointD> LidarLine { get; set; }
        public virtual List<Segment> LidarSegment { get; set; }
        public virtual List<PolarPointListExtended> LidarObjectList { get; set; }


        // public virtual List<PointD> lidarMap { get; set; }
        // public virtual List<PointD> lidarMapProcessed { get; set; }
        // public virtual Heatmap heatMapStrategy { get; set; }
        // public virtual Heatmap heatMapWaypoint { get; set; }
        #endregion
        #region Constructors
        public LocalWorldMap(int robotId, int teamId)
        {
            RobotId = robotId;
            TeamId = teamId;
            RobotLocation = new Location(0, 0, 0, 0, 0, 0);
            RobotGhostLocation = new Location(0, 0, 0, 0, 0, 0);
            WaypointLocations = new List<PointD> { };
            RobotHistorical = new List<Location> { RobotLocation };
            OnLocalWorldMapEvent?.Invoke(this, this);
        }
        #endregion
        #region Methods
        public void Init()
        {
            RobotLocation = new Location(0, 0, 0, 0, 0, 0);
            RobotGhostLocation = new Location(0, 0, 0, 0, 0, 0);
            WaypointLocations = new List<PointD> { };
            RobotHistorical = new List<Location> { RobotLocation };
            OnLocalWorldMapEvent?.Invoke(this, this);
        }

        public void OnUpdateRobotLocation(Location location)
        {
            RobotLocation = location;
            OnUpdateHistoricalLocation(location);
            OnUpdateRobotLocationEvent?.Invoke(this, location);
            OnLocalWorldMapEvent?.Invoke(this, this);
        }

        public void OnUpdateHistoricalLocation(Location location)
        {
            if (RobotHistorical.Count != 0)
            {
                Location lastHistorical = RobotHistorical[RobotHistorical.Count - 1];

                PointD p1 = new PointD(location.X, location.Y);
                PointD p2 = new PointD(lastHistorical.X, lastHistorical.Y);

                double distance = Toolbox.Distance(p1, p2);
                if (distance >= MINIMAL_WORLD_HISTORICAL_DIST)
                {
                    RobotHistorical.Add(location);
                    OnNewHistoricalPositionEvent?.Invoke(this, location);
                }

            }
            OnLocalWorldMapEvent?.Invoke(this, this);
        }

        public void AddNewWaypoints(PointD location)
        {
            WaypointLocations.Add(location);
            OnLocalWorldMapEvent?.Invoke(this, this);
        }

        public void SetDestinationLocation(Location location)
        {
            DestinationLocation = location;
            OnLocalWorldMapEvent?.Invoke(this, this);
        }

        public void ResetWaypoints()
        {
            WaypointLocations = new List<PointD> { };
            OnLocalWorldMapEvent?.Invoke(this, this);
        }

        public void ResetDestination()
        {
            DestinationLocation = new Location();
            OnLocalWorldMapEvent?.Invoke(this, this);
        }

        public void SetRobotLocation(Location location)
        {
            RobotLocation = location;
            OnLocalWorldMapEvent?.Invoke(this, this);
        }

        public void SetGhostRobotLocation(Location location)
        {
            RobotGhostLocation = location;
            OnLocalWorldMapEvent?.Invoke(this, this);
        }

        public void ResetRobot()
        {
            ResetRobot(new Location(0, 0, 0, 0, 0, 0));
        }

        public void ResetRobot(Location location)
        {
            RobotLocation = location;
            RobotGhostLocation = location;
            WaypointLocations = new List<PointD> { };
            RobotHistorical = new List<Location> { RobotLocation };
            OnResetRobotEvent?.Invoke(this, location);
            OnLocalWorldMapEvent?.Invoke(this, this);
        }
        #endregion
        #region Input Callback
        public void AddNewWaypointsEvent(object sender, PointD point)
        {
            AddNewWaypoints(point);
        }

        

        public void SetDestinationLocationEvent(object sender, PointD point)
        {
            SetDestinationLocation(new Location(point.X, point.Y,0,0,0,0));
        }

        public void ResetWaypointDestinationEvent(object sender, PointD point)
        {
            ResetWaypoints();
            ResetDestination();
        }

        

        public void OnLidarRawPointReceived(object sender, List<PointD> lidarPoints)
        {
            LidarMapRaw = lidarPoints;
            OnLocalWorldMapEvent?.Invoke(this, this);
        }

        public void OnLidarProcessedPointReceived(object sender, List<PointD> lidarPoints)
        {
            LidarMapProcessed = lidarPoints;
            OnLocalWorldMapEvent?.Invoke(this, this);
        }

        public void OnLidarProcessedLineReceived(object sender, List<Segment> segments)
        {
            //List<Segment> segments = new List<Segment>();
            //foreach (HoughLine line in lines)
            //{
            //    double x = line.rho * Math.Cos(line.theta);
            //    double y = line.rho * Math.Sin(line.theta);

            //    double slope = line.theta - (Math.PI / 2);
            //    double yintercept = y - slope * x;

            //    double x1 = x + 1;
            //    double y1 = slope * x1 + yintercept;

            //    double x2 = x - 1;
            //    double y2 = slope * x1 + yintercept;

            //    segments.Add(new Segment(x1, y1, x2, y2));
            //}
            LidarSegment = segments;
            OnLocalWorldMapEvent?.Invoke(this, this);
        }



        public void OnGameStateChange(object sender, GameState gameState_a)
        {
            OnLocalWorldMapEvent?.Invoke(this, this);
        }
        #endregion
        #region Others
        private bool InMaxSquare(double x, double y)
        {
            double max = Math.Sqrt(Math.Pow(3, 2) + Math.Pow(2, 2));
            double x_max = max;
            double x_min = -max;
            double y_max = max;
            double y_min = -max;
            return ((x >= x_min && x <= x_max) && (y >= y_min && y <= y_max));
        }
        #endregion


        #region Events
        public event EventHandler<Location> OnUpdateRobotLocationEvent;
        public event EventHandler<Location> OnUpdateGhostRobotLocation;
        public event EventHandler<Location> OnSetRobotLocationEvent;
        public event EventHandler<Location> OnSetGhostRobotLocationEvent;
        public event EventHandler<Location> OnNewHistoricalPositionEvent;
        public event EventHandler<Location> OnNewWaypointLocationEvent;
        public event EventHandler<Location> OnResetRobotEvent;
        public event EventHandler<LocalWorldMap> OnLocalWorldMapEvent;
        #endregion


        

    }

    public enum GameState
    {
        STOPPED,
        STOPPED_GAME_POSITIONING,
        PLAYING,
    }

    public enum StoppedGameAction
    {
        NONE,
        KICKOFF,
        KICKOFF_OPPONENT,
        FREEKICK,
        FREEKICK_OPPONENT,
        GOALKICK,
        GOALKICK_OPPONENT,
        THROWIN,
        THROWIN_OPPONENT,
        CORNER,
        CORNER_OPPONENT,
        PENALTY,
        PENALTY_OPPONENT,
        PARK,
        DROPBALL,
        GOTO,
        GOTO_OPPONENT,
    }
}
