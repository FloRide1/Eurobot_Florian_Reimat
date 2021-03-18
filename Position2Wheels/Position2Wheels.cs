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
        public int robotID, robotTeam;
        Location location;

        public Positioning2Wheels(int robotId, int robotTeam_a)
        {
            robotID = robotId;
            robotTeam = robotTeam_a;
        }

        public void OnOdometryRobotSpeedReceived(object sender, PolarSpeedArgs e)
        {

        }

        public void SetPosition(LocationArgs location)
        {

        }

        #region Input CallBack

        #endregion

        //Output events
        public event EventHandler<LocationArgs> OnCalculatedLocationEvent;
        public virtual void OnCalculatedLocation(int id, Location locationRefTerrain)
        {
            OnCalculatedLocationEvent?.Invoke(this, new LocationArgs { RobotId = id, Location = locationRefTerrain });
        }
    }
}