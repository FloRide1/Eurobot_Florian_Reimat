using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Utilities;
using EventArgsLibrary;
using Constants;
using Lidar;
using MathNet.Numerics;
using MathNet.Numerics.LinearRegression;

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
            List<PolarPointRssi> rawLidarPoints = new List<PolarPointRssi> { };
            LidarFrame++;
            foreach (var point in lidarPoints.LidarPoints)
            {
                PolarPointRssi rssiPoint = new PolarPointRssi();
                rssiPoint.Distance = point.Distance;
                rssiPoint.Angle = point.Angle;
                rssiPoint.Rssi = point.RSSI;
                
                rawLidarPoints.Add(rssiPoint);
                
            }
            List<PolarPointRssi> FloPoint = new List<PolarPointRssi>();
            foreach (PolarPointRssi points in rawLidarPoints)
            {
                FloPoint.Add(new PolarPointRssi(points.Angle, points.Distance * Math.Cos(points.Angle), points.Rssi));
            }
            rawLidar.PtList = rawLidarPoints;
            OnRawLidarDataEvent?.Invoke(this, rawLidar);
            OnRawLidarPointPolarEvent?.Invoke(this, rawLidarPoints);
            OnRawLidarPointXYEvent?.Invoke(this, ConvertRssiToXYCoord(rawLidarPoints));
            ProcessLidarData(rawLidarPoints);
        }

        public void OnRobotLocation(object sender, Location robot)
        {
            robotLocation = robot;
        }


        public event EventHandler<RawLidarArgs> OnRawLidarDataEvent;
        public event EventHandler<RawLidarArgs> OnProcessLidarDataEvent;
        public event EventHandler<List<PolarPointRssi>> OnRawLidarPointPolarEvent;
        public event EventHandler<List<PointD>> OnRawLidarPointXYEvent;
        public event EventHandler<List<PointD>> OnProcessLidarXYDataEvent;
        public event EventHandler<List<PolarPointRssi>> OnProcessLidarPolarDataEvent;
        public event EventHandler<List<Segment>> OnProcessLidarLineDataEvent;

        public void ProcessLidarData(List<PolarPointRssi> polarPointRssi)
        {
            List<PolarPointRssi> validPoint = new List<PolarPointRssi>();
            foreach(PolarPointRssi p in polarPointRssi)
            {
                if (p.Distance <= Math.Sqrt(Math.Pow(3, 2) + Math.Pow(2, 2)))
                {
                    validPoint.Add(p);
                }
            }


            List<BlobObject> Blobs = DetectBlob(validPoint, 0.03, 5);
            List<PolarPointRssi> processedPoints = new List<PolarPointRssi>();
            List<PolarPointRssi> pointFlo = new List<PolarPointRssi>();
            List<PolarPointRssi> fl0Derivative = new List<PolarPointRssi>();
            foreach (BlobObject b in Blobs)
            {
                foreach (PolarPointRssi p in b.points)
                {
                    processedPoints.Add(p);
                    pointFlo.Add(new PolarPointRssi(p.Angle, p.Distance * Math.Cos(p.Angle), p.Rssi));
                }
            }

            

            int i;
            for (i = 0; i < pointFlo.Count - 1; i++)
            {
                double point = pointFlo[i + 1].Distance - pointFlo[i].Distance;
                fl0Derivative.Add(new PolarPointRssi(pointFlo[i].Angle, point, pointFlo[i].Rssi));
            }

            List<Segment> Lines = new List<Segment>();

            

            OnProcessLidarLineDataEvent?.Invoke(this, Lines);
            // /!\ Warning NEED TO REMOVE


            RawLidarArgs processLidar = new RawLidarArgs() { RobotId = robotId, LidarFrameNumber = LidarFrame, PtList = pointFlo };
            OnProcessLidarDataEvent?.Invoke(this, processLidar);

            

            processLidar.PtList =  fl0Derivative;
            OnRawLidarDataEvent?.Invoke(this, processLidar);




            
            OnProcessLidarPolarDataEvent?.Invoke(this, processedPoints);
            OnProcessLidarXYDataEvent?.Invoke(this, ConvertRssiToXYCoord(processedPoints));
        }

        

        public List<ClusterObjects> DetectClusterOfPoint(List<PolarPointRssi> pointsList , double thresold)
        {
            /// ABD Stand for Adaptative breakpoint Detection
            List<ClusterObjects> listOfClusters = new List<ClusterObjects>();
            int i;
            for (i = 1; i < pointsList.Count - 1; i++)
            {
                PolarPointRssi point_n_minus_1 = pointsList[i - 1];
                PolarPointRssi point_n_plus_1 = pointsList[i + 1];
                PolarPointRssi point_n = pointsList[i];

                double dist_n_minus_1 = point_n_minus_1.Distance;
                double delta_theta = Math.Abs(point_n_minus_1.Angle - point_n.Angle);
                // double 

                // double ABD_Thresold = 
            }

            return listOfClusters;
        }

        public List<Segment> DetectGlobalLine(List<PolarPointRssi> pointsList, double thresold)
        {
            /// For faster seach of angle in array -> NEED TO EDIT
            double [] angle_array = { };
            foreach (PolarPointRssi point in pointsList)
            {
                angle_array.Append(point.Angle);
            }

            /// Begin 
            int i;
            double angle = 0d;
            double angle_step = 5d;

            int center_index = GetIndexOfAngle(angle_array, angle);
            
            PointD center_point = ConvertPolarToXYAbsoluteCoord(pointsList[center_index]);
            PointD left_center_point = ConvertPolarToXYAbsoluteCoord(pointsList[center_index - 1]);
            PointD right_center_pointsList = ConvertPolarToXYAbsoluteCoord(pointsList[center_index + 1]);

            double[] xPoints = new double[] { left_center_point.X, center_point.X, right_center_pointsList.X };
            double[] yPoints = new double[] { left_center_point.Y, center_point.Y, right_center_pointsList.Y };
            
            /// Get First Linear Regression
            Tuple<double, double> line = Fit.Line(xPoints, yPoints);
            double slope = line.Item1;
            double y_intercept = line.Item2;

            int erreur_count = 0;

            while (Math.Abs(angle) <= Math.PI / 2)
            {
                angle += angle_step;
                /// Calculate estimated point
                double angle_slope = Toolbox.DegToRad(angle);
                double angle_y_intercept = 0;

                double estimated_X = (y_intercept - angle_y_intercept) / (angle_slope - slope);
                double estimated_Y = slope * estimated_X + y_intercept;

                PointD estimated_point = new PointD(estimated_X, estimated_Y);

                /// Calculate Distance with measured point
                int angle_index = GetIndexOfAngle(angle_array, angle);
                PointD measured_point = ConvertPolarToXYAbsoluteCoord(pointsList[angle_index]);

                double delta = CalculateXYDistancePoint(estimated_point, measured_point);

                if (delta <= thresold) 
                {
                    xPoints.Append(measured_point.X);
                    yPoints.Append(measured_point.Y);

                    line = Fit.Line(xPoints, yPoints);
                    slope = line.Item1;
                    y_intercept = line.Item2;
                }
                else
                {
                    erreur_count++;
                    if (erreur_count == 3)
                    {
                        /// The line end 
                        angle -= 3 * angle_step;

                    }
                }

            }

            List<Segment> list_of_segments = new List<Segment>();
            return list_of_segments;
        }

        private int GetIndexOfAngle(double[] angle_array, double angle) 
        {
            int i, index = 0;
            double min = Math.Abs(angle - angle_array[0]);
            for (i = 0; i < angle_array.Count(); i++)
            {
                double delta = Math.Abs(angle - angle_array[i]);
                if (delta < min)
                {
                    min = delta;
                    index = i;
                }
            }
            return i;
        }

        private double CalculatePolarDistancePoint(PolarPointRssi p1, PolarPointRssi p2)
        {
            double r1 = p1.Distance;
            double r2 = p2.Distance;

            double t1 = p1.Angle;
            double t2 = p2.Angle;


            /// NEED TO REMAKE (SORRY MERGE FAIL HERE)
            return 0d;
        }
        private double CalculateXYDistancePoint(PointD p1 , PointD p2)
        {
            double x1 = p1.X;
            double y1 = p1.Y;
            double x2 = p2.X;
            double y2 = p2.Y;

            return Math.Sqrt(Math.Pow(x1 - x2, 2) + Math.Pow(y1 - y2, 2));
        }

        public Segment DetectLine(ClusterObjects blob, double thresold, double alignNbr, int moy= 5)
        {
            List<PolarPointRssi> pointList = blob.points;

            List<double>  derivate1 = new List<double>();
            List<double>  derivate2 = new List<double>();
            Segment line = new Segment();

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
            for (i = 0; i < derivate2.Count - 1; i++)
            {
                if (derivate2[i] >= 0 && derivate2[i] <= thresold)
                {
                    nbrOfCurrentAlign++;                    
                }
                else if (nbrOfCurrentAlign > 0) { 
                    if (nbrOfCurrentAlign >= alignNbr)
                    {
                        return CreateLineSegment(pointList, moy);
                    }
                    nbrOfCurrentAlign = 0;
                }

            }

            if (nbrOfCurrentAlign >= alignNbr)
            {
                return CreateLineSegment(pointList, moy);
            }

            return line;
        }

       

        private Segment CreateLineSegment(List<PolarPointRssi> pointList, double moy)
        {
            int j;
            double X1 = 0, Y1 = 0, X2 = 0, Y2 = 0;
            for (j = 0; j < moy; j++)
            {
                PointD point = ConvertPolarToRelativeCoord(pointList[j].Angle, pointList[j].Distance);
                X1 += point.X;
                Y1 += point.Y;
            }
            for (j = 0; j < moy; j++)
            {
                PointD point = ConvertPolarToRelativeCoord(pointList[pointList.Count - 1 - j].Angle, pointList[pointList.Count - 1 - j].Distance);
                X2 += point.X;
                Y2 += point.Y;
            }
            X1 /= moy;
            Y1 /= moy;
            X2 /= moy;
            Y2 /= moy;
            return new Segment(X1, Y1, X2, Y2);
        }
        private Segment CreateLineSegment(List<PolarPointRssi> pointList, int first,int last, double moy)
        {
            
            int i;
            double X1 = 0, Y1 = 0, X2 = 0, Y2 = 0;
            for (i = 0; i < moy; i++)
            {
                PointD point = ConvertPolarToRelativeCoord(pointList[first + i].Angle, pointList[first + i].Distance);
                X1 += point.X;
                Y1 += point.Y;
            }
            for (i = 0; i < moy; i++)
            {
                PointD point = ConvertPolarToRelativeCoord(pointList[last - i].Angle, pointList[last - i].Distance);
                X2 += point.X;
                Y2 += point.Y;
            }
            X1 /= moy;
            Y1 /= moy;
            X2 /= moy;
            Y2 /= moy;
            return new Segment(X1, Y1, X2, Y2);
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

        private PointD ConvertPolarToRelativeCoord(double angle, double distance)
        {
            double X = robotLocation.X;
            double Y = robotLocation.Y;
            double Theta = robotLocation.Theta;
            double pointDX = X + (distance * Math.Cos(angle - Theta));
            double pointDY = Y + (distance * Math.Sin(angle - Theta));
            return new PointD(pointDX, pointDY);
        }
        
        private PointD ConvertPolarToXYAbsoluteCoord(PolarPointRssi point)
        {
            double angle = point.Angle;
            double distance = point.Distance;

            double pointDX = distance * Math.Cos(angle);
            double pointDY = distance * Math.Sin(angle);
            return new PointD(pointDX, pointDY);
        }

        private PointD ConvertPolarToRelativeCoord(PolarPointRssi point)
        {
            double angle = point.Angle;
            double distance = point.Distance;
            double X = robotLocation.X;
            double Y = robotLocation.Y;
            double Theta = robotLocation.Theta;
            double pointDX = X + (distance * Math.Cos(angle - Theta));
            double pointDY = Y + (distance * Math.Sin(angle - Theta));
            return new PointD(pointDX, pointDY);
        }

    }
}
