using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Utilities;
using EventArgsLibrary;
using Constants;
using Lidar;

namespace LidarProcessNS
{
    public class LidarProcess
    {
        int robotId, teamId;
        public LidarProcess(int robot, int team)
        {
            robotId = robot;
            teamId = team;
        }

        public void OnNewLidarConnected(object sender, LidarDevice e)
        {


        }

        public void OnRawPointAvailable(object sender, LidarPointsReadyEventArgs lidarPoints)
        {
            ProcessLidarData(lidarPoints.LidarPoints);
        }

        public void ProcessLidarData(List<LidarPoint> lidarPoints)
        {
            //foreach (var point in lidarPoints)
            //{
            //    Console.WriteLine(point.Angle);
            //}
            OnProcessLidarDataEvent?.Invoke(this, lidarPoints);
        }

        public event EventHandler<List<LidarPoint>> OnProcessLidarDataEvent;

    }
}
