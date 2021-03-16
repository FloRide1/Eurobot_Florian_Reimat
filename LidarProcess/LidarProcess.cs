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
        int robotId, teamId, LidarFrame;
        Location robotLocation;
        List<PolarPointRssi> polarPointRssis;
        List<PointD> pointXYCoord;
        public LidarProcess(int robot, int team)
        {
            robotId = robot;
            teamId = team;
            robotLocation = new Location() { };
            LidarFrame = 0;
        }

        public void OnNewLidarConnected(object sender, LidarDevice e)
        {
            LidarFrame = 0;

        }


        public void OnRawPointAvailable(object sender, LidarPointsReadyEventArgs lidarPoints)
        {
            RawLidarArgs rawLidar = new RawLidarArgs() { RobotId = robotId, LidarFrameNumber = LidarFrame };
            List<PolarPointRssi> processlidarPoints = new List<PolarPointRssi> { };

            foreach (var point in lidarPoints.LidarPoints)
            {
                PolarPointRssi rssiPoint = new PolarPointRssi();
                rssiPoint.Distance = point.Distance;
                rssiPoint.Angle = point.Angle;
                rssiPoint.Rssi = point.RSSI;
                //if (rssiPoint.Distance <= Math.Sqrt(Math.Pow(3, 2) + Math.Pow(2, 2)))
                //{
                processlidarPoints.Add(rssiPoint);
                //}
            }
            rawLidar.PtList = processlidarPoints;
            OnRawLidarDataEvent?.Invoke(this, rawLidar);
            ProcessLidarData(processlidarPoints);
        }

        public void OnRobotLocation(object sender, Location robot)
        {
            robotLocation = robot;
        }


        public event EventHandler<RawLidarArgs> OnRawLidarDataEvent;
        public event EventHandler<List<PointD>> OnProcessLidarXYDataEvent;
        public event EventHandler<List<PolarPointRssi>> OnProcessLidarRssiDataEvent;
        public event EventHandler<List<Segment>> OnProcessLidarLineDataEvent;

        public void ProcessLidarData(List<PolarPointRssi> lidarRssi)
        {

            polarPointRssis = lidarRssi;
            pointXYCoord = ConvertRssiToXYCoord(lidarRssi);

            List<Segment> Lines = DetectLine(lidarRssi, 0.02, 100);
            OnProcessLidarLineDataEvent?.Invoke(this, Lines);

            OnProcessLidarRssiDataEvent?.Invoke(this, lidarRssi);
            OnProcessLidarXYDataEvent?.Invoke(this, pointXYCoord);
        }

        public List<PointD> ConvertRssiToXYCoord(List<PolarPointRssi> lidarPoints)
        {
            double X = robotLocation.X;
            double Y = robotLocation.Y;
            double Theta = robotLocation.Theta;

            List<PointD> XYPoints = new List<PointD> { };
            foreach (PolarPointRssi point in lidarPoints)
            {
                PointD pointD = ConvertPolarToRelativeCoord(point.Angle, point.Distance);
                XYPoints.Add(pointD);
            }
            return XYPoints;
        }

        /// Using Hough Transform /!\ Currently Not Working
        public List<HoughLine> DetectLineHough(List<PointD> pointList, uint thresold = 15, double delta = 1)
        {
            List<HoughLine> Lines = new List<HoughLine>() { };
            Dictionary<double, Dictionary<double, uint>> Matrix = new Dictionary<double, Dictionary<double, uint>>() { };
            foreach (PointD point in pointList)
            {
                double theta;
                for (theta = 0; theta < 180; theta += delta)
                {
                    double rho = point.X * Math.Cos(theta) + point.Y * Math.Sin(theta);
                    if (!Matrix.ContainsKey(rho))
                    {
                        Matrix.Add(rho, new Dictionary<double, uint>() { });
                    }
                    if (!Matrix[rho].ContainsKey(theta))
                    {
                        Matrix[rho].Add(theta, 0);
                    }
                    Matrix[rho][theta]++;
                }
            }

            foreach (KeyValuePair<double, Dictionary<double, uint>> rho in Matrix)
            {
                foreach (KeyValuePair<double, uint> theta in rho.Value)
                {
                    if (theta.Value >= thresold)
                    {
                        Lines.Add(new HoughLine(rho.Key, theta.Key));
                    }
                }
            }
            
            return Lines;
        }

        public List<Segment> DetectLine(List<PolarPointRssi> pointList, double thresold, double alignNbr)
        {
            
            List<double>  derivate1 = new List<double>();
            List<double>  derivate2 = new List<double>();
            List<Segment> lines = new List<Segment>();

            double X = robotLocation.X;
            double Y = robotLocation.Y;
            double Theta = robotLocation.Theta;

            int i;
            for (i = 0; i < pointList.Count - 1; i++)
            {
                derivate1.Add(Math.Abs(pointList[i].Distance - pointList[i + 1].Distance));
            }
            for (i = 0; i < derivate1.Count - 1; i++)
            {
                derivate2.Add(Math.Abs(derivate1[i] - derivate1[i + 1]));
            }

            uint nbrOfCurrentAlign = 0;
            int indexOfFirst = 0;
            for (i = 0; i < derivate2.Count - 1; i++)
            {
                if (derivate2[i] >= 0 && derivate2[i] <= thresold)
                {
                    nbrOfCurrentAlign++;
                    if (nbrOfCurrentAlign == 1)
                    {
                        indexOfFirst = i;
                    }
                    
                }
                else if (nbrOfCurrentAlign > 0) { 
                    if (nbrOfCurrentAlign >= alignNbr)
                    {
                        double x1, y1, x2, y2;
                        PointD point1 = ConvertPolarToRelativeCoord(pointList[indexOfFirst + 1].Angle, pointList[indexOfFirst + 1].Distance);
                        PointD point2 = ConvertPolarToRelativeCoord(pointList[i - 1].Angle, pointList[i - 1].Distance);

                        x1 = point1.X;
                        y1 = point1.Y;
                        x2 = point2.X;
                        y2 = point2.Y;
                        lines.Add(new Segment(x1, y1, x2, y2));
                    }
                    nbrOfCurrentAlign = 0;
                }

            }

                return lines;
        }

        private PointD ConvertPolarToRelativeCoord(double angle, double distance)
        {
            double X = robotLocation.X;
            double Y = robotLocation.Y;
            double Theta = robotLocation.Theta;
            double pointDX = X + (distance * Math.Cos(angle - Theta));
            double pointDY = Y + (distance * Math.Sin(angle - Theta));
            return new PointD(pointDX, pointDY);
        }

    }
}
