using EventArgsLibrary;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Utilities;

namespace Positioning2WheelsNS
{
    public class Positioning2Wheels
    {
        public int robotID;
        Location ActualLocation;
        double Tech_Sec = 1 / 50f;

        public Positioning2Wheels(int robotId)
        {
            robotID = robotId;
        }

        public Positioning2Wheels(int robotID, double x = 0, double y = 0, double theta = 0)
        {
            this.robotID = robotID;
            ActualLocation = new Location(x, y, theta, 0, 0, 0);
        }

        public Positioning2Wheels(int robotID, Location location)
        {
            this.robotID = robotID;
            ActualLocation = location;
        }

        public void OnOdometryRobotSpeedReceived(object sender, PolarSpeedArgs e)
        {
            ActualLocation.Theta += ActualLocation.Vtheta * Tech_Sec;

            ActualLocation.Vx = e.Vx * Math.Cos(ActualLocation.Theta);
            ActualLocation.Vy = e.Vx * Math.Sin(ActualLocation.Theta);
            ActualLocation.Vtheta = e.Vtheta;

            ActualLocation.X += ActualLocation.Vx * Tech_Sec;
            ActualLocation.Y += ActualLocation.Vy * Tech_Sec;

            OnCalculatedLocation(ActualLocation);
        }

        #region Input CallBack
        public void SetPosition(LocationArgs location)
        {
            this.ActualLocation = location.Location;
        }
        #endregion

        //Output events
        public event EventHandler<LocationArgs> OnCalculatedLocationEvent;
        public virtual void OnCalculatedLocation(Location locationRefTerrain)
        {
            OnCalculatedLocationEvent?.Invoke(this, new LocationArgs { RobotId = robotID, Location = locationRefTerrain });
        }
    }
}