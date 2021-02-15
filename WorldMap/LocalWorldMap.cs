using Constants;
using System;
using System.Collections.Generic;
using Utilities;

namespace WorldMap
{
    public class LocalWorldMap
    {
        private const double MINIMAL_WORLD_HISTORICAL_DIST = 0.010d;

        public virtual int RobotId { get; set; }
        public virtual int TeamId { get; set; }
        public virtual Location RobotLocation { get; set; }
        public virtual string messageDisplay { get; set; }

        public virtual PlayingSide playingSide { get; set; }
        public virtual Location RobotGhostLocation { get; set; }
        public virtual Location DestinationLocation { get; set; }
        public virtual List<Location> WaypointLocations { get; set; }
        public virtual List<Location> RobotHistorical { get; set; }
        public virtual List<Location> BallLocationList { get; set; }
        public virtual List<LocationExtended> ObstaclesLocationList { get; set; }
        public virtual List<PolarPointListExtended> LidarObjectList { get; set; }


        // public virtual List<PointD> lidarMap { get; set; }
        // public virtual List<PointD> lidarMapProcessed { get; set; }
        // public virtual Heatmap heatMapStrategy { get; set; }
        // public virtual Heatmap heatMapWaypoint { get; set; }

        public LocalWorldMap()
        {
            //Type = "LocalWorldMap";
        }

        public void Init(int robotId, int teamId)
        {
            RobotId = RobotId;
            TeamId = teamId;
            RobotLocation = new Location(0, 0, 0, 0, 0, 0);
            RobotGhostLocation = new Location(0, 0, 0, 0, 0, 0);
            WaypointLocations = new List<Location> { RobotLocation };
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
            WaypointLocations = new List<Location> { RobotLocation };
            RobotHistorical = new List<Location> { RobotLocation };
            OnResetRobotEvent?.Invoke(this, location);
            OnLocalWorldMapEvent?.Invoke(this, this);
        }



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
