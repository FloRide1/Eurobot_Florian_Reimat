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
        bool CalculateTrajectory = false;

        double samplingPeriod = 1 / 50.0d;

        double max_angular_acceleration;
        double max_linear_acceleration;

        double max_angular_speed;
        double max_linear_speed;

        double toleration_angular;
        double toleration_linear;

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

            GhostLocation = ActualLocation;

            CalculateTrajectory = true;
        }

        public void CalculateGhostMovement()
        {
            /// We rotate first and next we go forward until we end
            if (GhostLocation != WantedDestination && CalculateTrajectory)
            {
                CalculateGhostRotation();
                // GenerateGhostShifting();
                CalculateTrajectory = false;
            }
        }

        public bool CalculateGhostRotation()
        {
            /// Init
            double theta_ghost = GhostLocation.Theta;
            double theta_destination = WantedDestination.Theta;
            double angular_speed = GhostLocation.Vtheta;

            /// Loop
            double theta_remaining = theta_destination - Toolbox.ModuloByAngle(theta_destination, theta_ghost);
            double theta_stop = Math.Pow(angular_speed, 2) / (2 * max_angular_acceleration);

            if (theta_remaining > 0)
            {
                if (angular_speed < 0)
                {
                    /// Unatural -> Brake
                    angular_speed = angular_speed - (max_angular_acceleration * samplingPeriod);
                }
                else
                {
                    if (theta_remaining > theta_stop)
                    {
                        if (angular_speed < max_angular_speed)
                        {
                            /// Speed Up
                            angular_speed = angular_speed + (max_angular_acceleration * samplingPeriod);
                        }
                    }
                    else
                    {
                        /// Brake
                        angular_speed = angular_speed - (max_angular_acceleration * samplingPeriod);
                    }
                }
            }
            else
            {
                if (angular_speed > 0)
                {
                    /// Unatural -> Brake (Negatively)
                    angular_speed = angular_speed + (max_angular_acceleration * samplingPeriod);
                }
                else
                {
                    if (Math.Abs(theta_remaining) > theta_stop)
                    {
                        if (angular_speed > -max_angular_speed)
                        {
                            /// Speed Up (Negatively)
                            angular_speed = angular_speed - (max_angular_acceleration * samplingPeriod);
                        }
                    }
                    else
                    {
                        /// Brake (Negatively)
                        angular_speed = angular_speed + (max_angular_acceleration * samplingPeriod);
                    }
                }
            }

            

            theta_ghost = theta_ghost + angular_speed * samplingPeriod; /// Do i need to add 1/2 acceleration t^2 ?
            GhostLocation = new Location(GhostLocation.X, GhostLocation.Y, theta_ghost, 0, 0, angular_speed);
            OnNewGhostLocation(GhostLocation);

            if (Math.Abs(theta_remaining) < toleration_angular)
            {
                GhostLocation = WantedDestination;
                return true;
            }
            return false;

        }


        public void GenerateGhostShifting()
        {

        }

        #endregion

        #region Events
        public event EventHandler<Location> OnNewGhostLocationEvent;
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
            CalculateTrajectory = true;
        }

        public void OnPhysicalPositionReceived(object sender, LocationArgs e)
        {
            if (robotId == e.RobotId)
            {
                ActualLocation = e.Location;
                CalculateGhostMovement();
            }
        }

        #endregion

    }
}
