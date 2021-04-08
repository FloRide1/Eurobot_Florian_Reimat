using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Utilities;
using EventArgsLibrary;
using Constants;


namespace TrajectoryPlannerNs
{
    public class TrajectoryPlanner
    {
        /// <summary>
        ///  Implementation base on: https://www.vgies.com/downloads/Cours/Enonce-Projet-electronique-Partie-Haut-Niveau.pdf
        /// </summary>
        #region Parameters
        int robotId;

        private enum TrajectoryState
        {
            Idle,
            Angular,
            Linear
        }

        TrajectoryState state = TrajectoryState.Idle;

        double samplingPeriod = 1d / 50.0d;

        double max_angular_acceleration = 0.5d * Math.PI * 2.0d; /// In rad.s^{-2} : 1d
        double max_linear_acceleration = 2d; /// 1d

        double max_angular_speed = 1.0d * Math.PI * 2.0d; /// In rad.s^{-1} : 1d
        double max_linear_speed = 2.4d; /// 1.4

        double toleration_distance_angular = 0.005;
        double toleration_distance_linear = 0.01;

        double toleration_speed_angular = 0.02;
        double toleration_speed_linear = 0.01;

        int number_of_overpass = 1;
        int overpass_coeff = 1;

        Location ActualLocation;
        Location GhostLocation;
        Location WantedDestination;
        #endregion

        #region Constructor
        public TrajectoryPlanner() { }

        public TrajectoryPlanner(int robotId)
        {
            this.robotId = robotId;
            ActualLocation = new Location();
            WantedDestination = new Location();
            GhostLocation = new Location();
        }
        #endregion

        #region Methods
        public void InitMovement(Location ActualLocation, Location WantedDestination)
        {
            this.ActualLocation = ActualLocation;
            this.WantedDestination = WantedDestination;
            Init();
        }

        public void Init()
        {
            GhostLocation = ActualLocation;
            state = TrajectoryState.Angular;
            number_of_overpass = 1;
            overpass_coeff = 1;
        }

        #region Global
        public void CalculateGhostMovement()
        {
            /// We rotate first and next we go forward until we end
            if (GhostLocation != WantedDestination && state != TrajectoryState.Idle)
            {
                if (state == TrajectoryState.Angular)
                {
                    if (CalculateGhostRotation())
                    {
                        state = TrajectoryState.Linear;
                        number_of_overpass = 1;
                        overpass_coeff = 1;
                    }
                }
                else if (state == TrajectoryState.Linear)
                {
                    if (GenerateGhostShifting())
                    {
                        state = TrajectoryState.Idle;
                        GhostLocation = new Location(WantedDestination.X, WantedDestination.Y, GhostLocation.Theta, 0, 0, 0);
                        OnNewGhostLocation(GhostLocation);
                        ActualLocation = GhostLocation; /// NEED TO DELETE JUST FOR FUN
                        OnNewRobotLocation(ActualLocation); /// NEED TO DELETE JUST FOR FUN
                        OnDestinationReached();
                        number_of_overpass = 1;
                        overpass_coeff = 1;
                    }
                }             

            }
        }
        #endregion

        #region Rotation 
        public bool CalculateGhostRotation()
        {
            /// Init
            double theta_ghost = GhostLocation.Theta;
            double theta_destination = Math.Atan2(WantedDestination.Y - GhostLocation.Y, WantedDestination.X - GhostLocation.X);
            double angular_speed = GhostLocation.Vtheta;

            /// Loop
            double theta_remaining = theta_destination - Toolbox.ModuloByAngle(theta_destination, theta_ghost);
            double theta_stop = Math.Pow(angular_speed, 2) / (2 * max_angular_acceleration);

            if (theta_remaining > 0)
            {
                if (angular_speed < 0)
                {
                    /// Unatural -> Brake
                    angular_speed = angular_speed + (max_angular_acceleration * samplingPeriod);
                    if (number_of_overpass > 0)
                    {
                        number_of_overpass *= -1;
                        number_of_overpass--;
                    }
                    else if (number_of_overpass == 0)
                    {
                        number_of_overpass = -1;
                    }
                }
                else
                {
                    if (theta_remaining > theta_stop)
                    {
                        if (angular_speed < max_angular_speed)
                        {
                            /// Speed Up
                            angular_speed = angular_speed + (max_angular_acceleration / overpass_coeff * samplingPeriod);
                        }
                    }
                    else
                    {
                        /// Brake
                        angular_speed = angular_speed - (max_angular_acceleration / overpass_coeff * samplingPeriod);
                    }
                }
            }
            else
            {
                if (angular_speed > 0)
                {
                    /// Unatural -> Brake
                    angular_speed = angular_speed - (max_angular_acceleration * samplingPeriod);
                    if (number_of_overpass < 0)
                    {
                        number_of_overpass *= -1;
                        number_of_overpass++;
                    }
                    else if (number_of_overpass == 0)
                    {
                        number_of_overpass = 1;
                    }
                }
                else
                {
                    if (Math.Abs(theta_remaining) > theta_stop)
                    {
                        if (angular_speed > -max_angular_speed)
                        {
                            /// Speed Up (Negatively)
                            angular_speed = angular_speed - (max_angular_acceleration / overpass_coeff * samplingPeriod);
                        }
                    }
                    else
                    {
                        /// Brake (Negatively)
                        angular_speed = angular_speed + (max_angular_acceleration / overpass_coeff * samplingPeriod);
                    }
                }
            }


            Console.WriteLine("Angular: " + number_of_overpass + " : " + overpass_coeff + " ; " + angular_speed + " - " + theta_remaining);
            theta_ghost = theta_ghost + angular_speed * samplingPeriod; /// Do i need to add 1/2 acceleration t^2 ?
            GhostLocation = new Location(GhostLocation.X, GhostLocation.Y, theta_ghost, 0, 0, angular_speed);
            OnNewGhostLocation(GhostLocation);
            if (Math.Abs(number_of_overpass) > 5)
            {
                /// Oscillation
                overpass_coeff++;
                number_of_overpass = 0;
            }


            if (Math.Abs(theta_remaining) < toleration_distance_angular && Math.Abs(angular_speed) <= toleration_speed_angular)
            {
                return true;
            }
            return false;

        }
        #endregion

        #region Linear
        public bool GenerateGhostShifting()
        {
            /// Init
            PointD xy_ghost = new PointD(GhostLocation.X, GhostLocation.Y);
            PointD xy_wanted = new PointD(WantedDestination.X, WantedDestination.Y);

            PointD project_point =new PointD(xy_ghost.X + Math.Cos(GhostLocation.Theta), xy_ghost.Y + Math.Sin(GhostLocation.Theta));

            PointD xy_destination = Toolbox.ProjectedPointOnLineFromWaypoint(xy_wanted, xy_ghost, project_point); /// Make A projection for avoiding infinity movement
            double linear_speed = GhostLocation.Vx;

            /// Loop
            double theta_destination = Math.Atan2(xy_destination.Y - xy_ghost.Y, xy_destination.X - xy_ghost.X);
            double theta_remaining = theta_destination - Toolbox.ModuloByAngle(theta_destination, GhostLocation.Theta);

            double linear_remaining = Toolbox.Distance(xy_ghost, xy_destination);
            linear_remaining *= (Math.Abs(theta_remaining) > Math.PI / 2) ? -1 : 1;
            double linear_stop = Math.Pow(linear_speed, 2) / (2 * max_linear_acceleration);

            if (linear_remaining > 0)
            {
                if (linear_speed < 0)
                {
                    /// Unatural -> Brake
                    linear_speed = linear_speed + (max_linear_acceleration * samplingPeriod);
                    if (number_of_overpass > 0)
                    {
                        number_of_overpass *= -1;
                        number_of_overpass--;
                    } else if (number_of_overpass == 0)
                    {
                        number_of_overpass = -1;
                    }
                }
                else
                {
                    if (linear_remaining > linear_stop)
                    {
                        if (linear_speed < max_linear_speed)
                        {
                            /// Speed Up
                            linear_speed = linear_speed + (max_linear_acceleration / overpass_coeff * samplingPeriod);
                        }
                    }
                    else
                    {
                        /// Brake
                        linear_speed = linear_speed - (max_linear_acceleration / overpass_coeff * samplingPeriod);
                    }
                }
            }
            else
            {
                /// We overpass the point
                if (linear_speed > 0)
                {
                    /// Unatural -> Brake
                    linear_speed = linear_speed - (max_linear_acceleration * samplingPeriod);
                    if (number_of_overpass < 0)
                    {
                        number_of_overpass *= -1;
                        number_of_overpass++;
                    }
                    else if (number_of_overpass == 0)
                    {
                        number_of_overpass = 1;
                    }

                }
                else
                {
                    if (Math.Abs(linear_remaining) > linear_stop)
                    {
                        if (linear_speed > -max_linear_speed)
                        {
                            /// Speed Up (Negatively)
                            linear_speed = linear_speed - (max_linear_acceleration / overpass_coeff * samplingPeriod);
                        }
                    }
                    else
                    {
                        /// Brake (Negatively)
                        linear_speed = linear_speed + (max_linear_acceleration / overpass_coeff * samplingPeriod);
                    }
                }
            }

            Console.WriteLine("Linear: " + number_of_overpass + " : " + overpass_coeff + " ; " + linear_speed + " - " + linear_remaining + " ? " + Toolbox.Distance(xy_destination, xy_wanted));
            if (Math.Abs(number_of_overpass) > 5)
            {
                /// Oscillation
                overpass_coeff++;
                number_of_overpass = 0;
            }

            double x_ghost = xy_ghost.X + linear_speed * samplingPeriod * Math.Cos(GhostLocation.Theta);
            double y_ghost = xy_ghost.Y + linear_speed * samplingPeriod * Math.Sin(GhostLocation.Theta);
            
            GhostLocation = new Location(x_ghost, y_ghost, GhostLocation.Theta, linear_speed, 0, 0);
            OnNewGhostLocation(GhostLocation);

            if (Math.Abs(linear_remaining) < toleration_distance_linear && Math.Abs(linear_speed) <= toleration_speed_linear)
            {
                
                return true;
            }

            return false;
        }
        #endregion

        #endregion

        #region Events
        public event EventHandler<Location> OnNewGhostLocationEvent;
        public event EventHandler<Location> OnNewRobotLocationEvent;
        public event EventHandler<Location> OnNewMovingOrderEvent;
        public event EventHandler<Location> OnDestinationReachedEvent;
        public event EventHandler<PolarSpeedArgs> OnPolarSpeedOrderEvent;
        #endregion

        #region Events Call
        public virtual void OnPolarSpeedOrder(PolarSpeedArgs polarSpeed)
        {
            OnPolarSpeedOrderEvent?.Invoke(this, polarSpeed);
        }

        public virtual void OnAngularSpeedOrder(double angular_speed)
        {
            OnPolarSpeedOrder(new PolarSpeedArgs() { RobotId = robotId, Vtheta = angular_speed });
        }

        public virtual void OnNewGhostLocation(Location location)
        {
            OnNewGhostLocationEvent?.Invoke(this, location);
        }

        public virtual void OnNewRobotLocation(Location location)
        {
            OnNewRobotLocationEvent?.Invoke(this, location);
        }
        #endregion

        #region Input CallBack
        public void OnUpdateActualLocation(object sender, Location location)
        {
            ActualLocation = location;
        }

        public void OnUpdateWantedDestination(object sender, Location location)
        {
            WantedDestination = location;
        }

        public void OnLaunchCalculation(object sender, EventArgs e)
        {
            Init();
        }

        public void OnPhysicalPositionReceived(object sender, LocationArgs e)
        {
            if (robotId == e.RobotId)
            {
                ActualLocation = e.Location;
                CalculateGhostMovement();
            }
        }

        public void OnCalculateGhostMovement(object sender, EventArgs e)
        {
            CalculateGhostMovement();
        }

        public void OnDestinationReached()
        {
            OnDestinationReachedEvent?.Invoke(this, WantedDestination);
        }


        #endregion

    }
}
