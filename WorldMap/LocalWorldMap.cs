﻿using Constants;
using System;
using System.Collections.Generic;
using Utilities;
using EventArgsLibrary;
using System.Windows;
using Lidar;
using System.Linq;

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
        public virtual List<PointDExtended> LidarMapRaw { get; set; }
        public virtual List<PointDExtended> LidarMapProcessed { get; set; }
        public virtual List<PointD> LidarLine { get; set; }
        public virtual List<SegmentExtended> LidarSegment { get; set; }

        public virtual List<Cup> LidarCup { get; set; }
        public virtual List<LidarObjects> LidarObjectList { get; set; }


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
            LidarCup = new List<Cup>();
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
            DestinationLocation = null;
            OnLocalWorldMapEvent?.Invoke(this, this);
        }

        public void DeleteFirstWaypoint()
        {
            WaypointLocations.RemoveAt(0);
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

        public void SetDestinationLocationEvent(object sender, Location location)
        {
            SetDestinationLocation(location);
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

        public void ResetDestinationEvent(object sender, EventArgs e)
        {
            ResetDestination();
        }

        public void OnLidarRawPointReceived(object sender, List<PolarPointRssiExtended> lidarPoints)
        {
            

            LidarMapRaw = lidarPoints.Select(
                x => new PointDExtended(
                    new PointD( 
                        RobotLocation.X + (x.Pt.Distance * Math.Cos(RobotLocation.Theta + x.Pt.Angle)),
                        RobotLocation.Y + (x.Pt.Distance * Math.Sin(RobotLocation.Theta + x.Pt.Angle))
                    ),
                    x.Color,
                    x.Width
                )
            ).ToList();
            OnLocalWorldMapEvent?.Invoke(this, this);
        }

        public void OnLidarProcessedPointReceived(object sender, List<PolarPointRssiExtended> lidarPoints)
        {
            LidarMapProcessed = lidarPoints.Select(
                x => new PointDExtended(
                    new PointD(
                        RobotLocation.X + (x.Pt.Distance * Math.Cos(RobotLocation.Theta + x.Pt.Angle)),
                        RobotLocation.Y + (x.Pt.Distance * Math.Sin(RobotLocation.Theta + x.Pt.Angle))
                    ),
                    x.Color,
                    x.Width
                )
            ).ToList();
            OnLocalWorldMapEvent?.Invoke(this, this);
        }

        public void OnLidarProcessedLineReceived(object sender, List<SegmentExtended> list_of_segments)
        {
            List<SegmentExtended> corrected_list_segment = new List<SegmentExtended>();

            foreach(SegmentExtended segment in list_of_segments)
            {
                PolarPointRssi point_a = Toolbox.ConvertPointDToPolar(new PointD(segment.Segment.X1, segment.Segment.Y1));
                PolarPointRssi point_b = Toolbox.ConvertPointDToPolar(new PointD(segment.Segment.X2, segment.Segment.Y2));

                PointD correct_point_a = new PointD(
                        RobotLocation.X + (point_a.Distance * Math.Cos(RobotLocation.Theta + point_a.Angle)),
                        RobotLocation.Y + (point_a.Distance * Math.Sin(RobotLocation.Theta + point_a.Angle))
                    );

                PointD correct_point_b = new PointD(
                        RobotLocation.X + (point_b.Distance * Math.Cos(RobotLocation.Theta + point_b.Angle)),
                        RobotLocation.Y + (point_b.Distance * Math.Sin(RobotLocation.Theta + point_b.Angle))
                    );
                corrected_list_segment.Add(new SegmentExtended(correct_point_a, correct_point_b, segment.Color, segment.Width));
            }


            LidarSegment = corrected_list_segment;
            OnLocalWorldMapEvent?.Invoke(this, this);
        }

        public void OnProcessLidarAbsoluteLineReceived(object sender, List<PointDExtended> list_of_points)
        {
            if (LidarMapProcessed != null)
                LidarMapProcessed.AddRange(list_of_points);
            else
                LidarMapProcessed = list_of_points.ToList();
        }


        public void OnLidarProcessedCupReceived(object sender, List<Cup> cups)
        {
            LidarCup = cups;
            OnLocalWorldMapEvent?.Invoke(this, this);
        }

        public void OnLidarProcesObjectsReceived(object sender, List<LidarObjects> lidarObjects)
        {
            LidarObjectList = lidarObjects;
            OnLocalWorldMapEvent?.Invoke(this, this);
        }

        public void OnGameStateChange(object sender, GameState gameState_a)
        {
            OnLocalWorldMapEvent?.Invoke(this, this);
        }

        public void OnGhostLocation(object sender, Location location)
        {
            SetGhostRobotLocation(location);
        }

        public void OnRobotLocation(object sender, Location location)
        {
            OnUpdateRobotLocation(location);
        }

        public void OnRobotLocationArgs(object sender, LocationArgs location)
        {
            OnUpdateRobotLocation(location.Location);
        }

        public void OnWaypointReached(object sender, PointD point)
        {
            DeleteFirstWaypoint();
        }

        public void OnDestinationReached(object sender, Location location)
        {
            ResetDestination();
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
