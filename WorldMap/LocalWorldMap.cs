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
        public virtual List<LocationExtended> obstaclesLocationList { get; set; }
        public virtual List<PolarPointListExtended> lidarObjectList { get; set; }


        public virtual List<PointD> lidarMap { get; set; }
        public virtual List<PointD> lidarMapProcessed { get; set; }
        //public virtual Heatmap heatMapStrategy { get; set; }
        //public virtual Heatmap heatMapWaypoint { get; set; }

        public LocalWorldMap()
        {
            //Type = "LocalWorldMap";
        }

        public void Init()
        {
            RobotLocation = new Location(0, 0, 0, 0, 0, 0);
            RobotGhostLocation = new Location(0, 0, 0, 0, 0, 0);
            WaypointLocations = new List<Location> { RobotLocation };
            RobotHistorical = new List<Location> { RobotLocation };
        }

        public void SetRobotLocation(Location location)
        {
            RobotLocation = location;
        }

        public void SetGhostRobotLocation(Location location)
        {
            RobotGhostLocation = location;
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
        }

        #region Events
        public event EventHandler<Location> OnNewRobotLocationEvent;
        public event EventHandler<Location> OnNewGhostRobotLocationEvent;
        public event EventHandler<Location> OnNewHistoricalPositionEvent;
        public event EventHandler<Location> OnNewWaypointLocationEvent;
        public event EventHandler<Location> OnResetRobotEvent;
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
