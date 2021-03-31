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
using System.Drawing;

/// <summary>
/// Notes for myself Medium Kalman Filter
/// </summary>

namespace LidarProcessNS
{
    public class LidarProcess
    {
        #region Parameters
        int robotId, teamId, LidarFrame;
        Location robotLocation;
        List<PolarPointRssi> polarPointRssis;
        List<PointD> pointXYCoord;
        #endregion

        #region Constructor
        public LidarProcess(int robot, int team)
        {
            robotId = robot;
            teamId = team;
            robotLocation = new Location() { };
            LidarFrame = 0;
        }
        #endregion

        #region Callback
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
        #endregion

        #region Event
        public event EventHandler<RawLidarArgs> OnRawLidarDataEvent;
        public event EventHandler<RawLidarArgs> OnProcessLidarDataEvent;
        public event EventHandler<List<PolarPointRssi>> OnRawLidarPointPolarEvent;
        public event EventHandler<List<PointD>> OnRawLidarPointXYEvent;
        public event EventHandler<List<PointD>> OnProcessLidarXYDataEvent;
        public event EventHandler<List<PolarPointRssi>> OnProcessLidarPolarDataEvent;
        public event EventHandler<List<Segment>> OnProcessLidarLineDataEvent;
        public event EventHandler<List<Cup>> OnProcessLidarCupDataEvent;
        #endregion


        public void ProcessLidarData(List<PolarPointRssi> polarPointRssi)
        {
            List<PolarPointRssi> validPoint = new List<PolarPointRssi>();
            foreach (PolarPointRssi p in polarPointRssi)
            {
                if (p.Distance <= Math.Sqrt(Math.Pow(3, 2) + Math.Pow(2, 2)))
                {
                    validPoint.Add(p);
                }
            }



            List<ClusterObjects> clusterObjects = DetectClusterOfPoint(validPoint, 0.035, 3);
            
            List<PolarPointRssi> processedPoints = new List<PolarPointRssi>();
            List<Cup> list_of_cups = new List<Cup>();
            foreach (ClusterObjects c in clusterObjects)
            {
                foreach (PolarPointRssi p in c.points)
                {
                    processedPoints.Add(p);
                }

                Cup cup = DetectCup(c);
                // The null condition is Bad need to edit
                if (cup != null)
                {
                    list_of_cups.Add(cup);
                }
            }




            List<Segment> Lines = new List<Segment>();
            // Lines.Add(DetectGlobalLine(polarPointRssi, 1d, 0d, 5d, 3, 0.2d));
            OnProcessLidarLineDataEvent?.Invoke(this, Lines);

            OnProcessLidarCupDataEvent?.Invoke(this, list_of_cups);


            RawLidarArgs processLidar = new RawLidarArgs() { RobotId = robotId, LidarFrameNumber = LidarFrame, PtList = processedPoints };
            OnProcessLidarDataEvent?.Invoke(this, processLidar);

            OnProcessLidarPolarDataEvent?.Invoke(this, processedPoints);
            OnProcessLidarXYDataEvent?.Invoke(this, ConvertRssiToXYCoord(processedPoints));
        }



        public List<ClusterObjects> DetectClusterOfPoint(List<PolarPointRssi> pointsList, double thresold, int mininum_amount_of_points)
        {
            /// ABD Stand for Adaptative breakpoint Detection
            List<ClusterObjects> listOfClusters = new List<ClusterObjects>();
            ClusterObjects cluster = new ClusterObjects();
            int i;
            for (i = 1; i < pointsList.Count - 1 ; i++)
            {
                PolarPointRssi point_n_minus_1 = pointsList[i - 1];
                PolarPointRssi point_n_plus_1 = pointsList[i + 1];
                PolarPointRssi point_n = pointsList[i];

                double dist_n_minus_1 = point_n_minus_1.Distance;
                double delta_theta = Math.Abs(point_n_minus_1.Angle - point_n.Angle);
                double lambda = point_n_plus_1.Angle - point_n_minus_1.Angle;

                double ABD_Thresold = thresold; // dist_n_minus_1 * (Math.Sin(delta_theta) / Math.Sin(lambda - delta_theta));
                double distance_between_point = CalculatePolarDistancePoint(point_n, point_n_minus_1);
                if (distance_between_point < ABD_Thresold)
                {
                    cluster.points.Add(point_n);
                } else
                {
                    if (cluster.points.Count() > mininum_amount_of_points)
                    {
                        listOfClusters.Add(cluster);
                    }
                    cluster = new ClusterObjects();
                }


            }
            if (cluster.points.Count() > mininum_amount_of_points)
            {
                listOfClusters.Add(cluster);
            }

            return listOfClusters;
        }

        #region Global Line Detection
        /// <summary>
        /// Detect Line in a global frame: fast + resolution less
        /// </summary>
        /// <param name="pointsList"> The list of point by the Lidar</param>
        /// <param name="thresold"> The distance error allowed </param>
        /// <param name="angle_of_start"> Default: 0 degree </param>
        /// <param name="angle_step"> Default: 5 degree </param>
        /// <param name="error_max"> The max error countdown before stopping the segment </param>
        /// <returns></returns>
        public Segment DetectGlobalLine(List<PolarPointRssi> pointsList, double thresold, double angle_of_start = 0d,
            double angle_step = 5d, int error_max = 3, double lenght_minimum = 1d)
        {
            /// Note: Remember to add Array Overflow Security
            /// Note: Avoid 0 Angle (Infinite bug) or implement security

            /// For faster seach of angle in array -> NEED TO EDIT
            List<double> angle_array = new List<double>();
            foreach (PolarPointRssi point in pointsList)
            {
                if (point.Distance != 0d)
                {
                    angle_array.Add(point.Angle);
                }
                
            }

            /// Init
            int i;
            double angle = angle_of_start;


            int center_index = GetIndexOfAngle(angle_array, angle);

            /// Get First three Points
            PointD center_point = ConvertPolarToXYAbsoluteCoord(pointsList[center_index]);
            PointD left_center_point = ConvertPolarToXYAbsoluteCoord(pointsList[center_index - 1]);
            PointD right_center_pointsList = ConvertPolarToXYAbsoluteCoord(pointsList[center_index + 1]);

            /// Points for Linear Regression
            List<double> xPoints = new List<double>() { left_center_point.X, center_point.X, right_center_pointsList.X };
            List<double> yPoints = new List<double>() { left_center_point.Y, center_point.Y, right_center_pointsList.Y };

            /// Get First Linear Regression
            Tuple<double, double> line = Fit.Line(xPoints.ToArray(), yPoints.ToArray());
            double slope = line.Item1;
            double y_intercept = line.Item2;
            
            /// Init Error
            int error_count = 0;

            /// Extremity of Segment
            PointD[] extremity_of_segment = new PointD[2];
            int side;
            for (side = 0; side < 2; side++)
            {
                angle = Toolbox.DegToRad(angle_of_start);
                /// This loop is only for making Right (0) + Left (1) Side
                bool side_is_finish = false;
                while (Math.Abs(angle) < Math.PI / 2 && !side_is_finish)
                {
                    /// If the side is Left (1) we invert the step
                    angle += (side % 2 == 0?1:-1) * Toolbox.DegToRad(angle_step);

                    /// Calculate estimated point
                    double angle_slope = angle;
                    double angle_y_intercept = 0;

                    PointD estimated_point = GetCrossingPoint(slope, y_intercept, angle_slope, angle_y_intercept);

                    /// Calculate Distance with measured point
                    int angle_index = GetIndexOfAngle(angle_array, angle);
                    PointD measured_point = ConvertPolarToXYAbsoluteCoord(pointsList[angle_index]);

                    double delta = CalculateXYDistancePoint(estimated_point, measured_point);

                    if (delta <= thresold)
                    {
                        /// Enhance Linear Regression
                        xPoints.Add(measured_point.X);
                        yPoints.Add(measured_point.Y);

                        line = Fit.Line(xPoints.ToArray(), yPoints.ToArray());
                        slope = line.Item1;
                        y_intercept = line.Item2;
                    }
                    else
                    {
                        /// Measured point does'nt coincide with Estimate point
                        error_count++;
                        if (error_count == error_max)
                        {
                            /// The line end 
                            angle -= ((side % 2 == 0) ? 1 : -1) * error_max * Toolbox.DegToRad(angle_step);


                            /// Try to find the end of the segment
                            /// Make a for loop from the end valid angle and continue until the thresold is uncorrect
                            int index_of_last_valid_point = 0;
                            bool point_is_valid = true;
                            int j = GetIndexOfAngle(angle_array, angle);

                            while (point_is_valid)
                            {
                                j += (side % 2 == 0 ? 1 : -1);
                                if (j < 0 || j >= pointsList.Count)
                                {
                                    point_is_valid = false;
                                }
                                else
                                {
                                    PolarPointRssi actual_point = pointsList[j];

                                    double measured_angle = actual_point.Angle;
                                    measured_point = ConvertPolarToXYAbsoluteCoord(actual_point);

                                    estimated_point = GetCrossingPoint(slope, y_intercept, measured_angle, 0);

                                    if (CalculateXYDistancePoint(measured_point, estimated_point) > thresold)
                                    {
                                        point_is_valid = false;
                                        j -= (side % 2 == 0 ? 2 : -2);
                                        index_of_last_valid_point = j;
                                    }
                                    else
                                    {
                                        /// Enhance Linear Regression
                                        xPoints.Add(measured_point.X);
                                        yPoints.Add(measured_point.Y);

                                        line = Fit.Line(xPoints.ToArray(), yPoints.ToArray());
                                        slope = line.Item1;
                                        y_intercept = line.Item2;
                                    }
                                }
                            }
                            /// Now that we have the last measured index of the segment end the line 
                            /// with the parallel of the measured point and the estimated line 

                            ///  Note: that line is useless but for safety i write it during Algorithm implementation
                            measured_point = ConvertPolarToRelativeCoord(pointsList[index_of_last_valid_point]);
                            extremity_of_segment[side] = FinishTheSegment(measured_point, slope, y_intercept);

                            /// Now Switch with the reverse side by ending the while loop
                            side_is_finish = true;
                        }
                    }


                    if (Math.Abs(angle) >= Math.PI / 2)
                    {
                        /// The loop end without finding the end of the line
                        side_is_finish = true;
                        angle_index = GetIndexOfAngle(angle_array, angle);
                        measured_point = ConvertPolarToXYAbsoluteCoord(pointsList[angle_index]);
                        estimated_point = GetCrossingPoint(slope, y_intercept, pointsList[angle_index].Angle, 0);
                        if (CalculateXYDistancePoint(measured_point, estimated_point) <= thresold)
                        {
                            extremity_of_segment[side] = FinishTheSegment(measured_point, slope, y_intercept);
                        }


                        
                    }
                }

            }
            if (extremity_of_segment[0] == null || extremity_of_segment[1] == null)
            {
                return new Segment();
            }
            /// Check the lenght of the segment and if it to small abort it 
            if (CalculateXYDistancePoint(extremity_of_segment[0], extremity_of_segment[1]) < lenght_minimum) 
            {
                return new Segment();
            }
            if (xPoints.Count <= 3)
            {
                return new Segment();
            }

            Segment segment = new Segment(extremity_of_segment[0], extremity_of_segment[1]);
            return segment;
        }

        private PointD FinishTheSegment(PointD measured_point, double slope, double y_intercept)
        {
            double perpendicular_slope = -slope;
            double perpendicular_y_intercept = measured_point.Y - (perpendicular_slope * measured_point.X);

            PointD point = measured_point;// GetCrossingPoint(slope, y_intercept, perpendicular_slope, perpendicular_y_intercept);
            return ConvertAbsoluteToRelativeCoord(point);
        }

        private PointD GetCrossingPoint(double slope_a, double y_intercept_a, double slope_b, double y_intercept_b)
        {
            double estimated_X = (y_intercept_a - y_intercept_b) / (slope_b - slope_a);
            double estimated_Y = slope_a * estimated_X + y_intercept_a;

            return new PointD(estimated_X, estimated_Y);
        }
        #endregion


        public Segment DetectLine(ClusterObjects blob, double thresold, double alignNbr, int moy = 5)
        {
            List<PolarPointRssi> pointList = blob.points;

            List<double> derivate1 = new List<double>();
            List<double> derivate2 = new List<double>();
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
                else if (nbrOfCurrentAlign > 0)
                {
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

        public Cup DetectCup(ClusterObjects cluster)
        {
            /// TEMPORARY NEED TO EDIT: ONLY FOR DEBUG PURPOSE

            PolarPointRssi begin_point = cluster.points[0];
            PolarPointRssi end_point = cluster.points[cluster.points.Count - 1];

            double lenght_of_cluster = CalculatePolarDistancePoint(begin_point, end_point);

            if (lenght_of_cluster >= 0.040 && lenght_of_cluster <= 0.08)
            {
                List<PointD> pointDs = new List<PointD>();
                double min_rssi = cluster.points[0].Rssi;
                double max_rssi = cluster.points[0].Rssi;

                foreach (PolarPointRssi point in cluster.points)
                {
                    if (point.Rssi > max_rssi)
                    {
                        max_rssi = point.Rssi;
                    }
                    if (point.Rssi < min_rssi)
                    {
                        min_rssi = point.Rssi;
                    }
                    pointDs.Add(ConvertPolarToRelativeCoord(point));
                }
                //moyenne /= cluster.points.Count();
                double median = 0.80;
                double b = cluster.points[(int)(cluster.points.Count() * median)].Rssi;
                double e = cluster.points[(int)(cluster.points.Count() * (1 - median))].Rssi;
                double moyenne = (b + e) / 2;
                Color color = Color.White;
                if (moyenne >= 9000 && moyenne <= 12000)
                {
                    color = Color.Green;
                }
                else if (moyenne >= 12000 && moyenne <= 14000)
                {
                    color = Color.Red;
                } 
                else
                {
                    return new Cup();
                }
                Console.WriteLine(moyenne);
                PointD center_point = GetMediumPoint(pointDs);
                return new Cup(center_point, 0.065, color);
            } 
            else
            {
                return new Cup();
            }

        }

        #region Utils
        #region Segments
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
        private Segment CreateLineSegment(List<PolarPointRssi> pointList, int first, int last, double moy)
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
        #endregion
        #region Distance Calculate
        private double CalculatePolarDistancePoint(PolarPointRssi p1, PolarPointRssi p2)
        {
            double r1 = p1.Distance;
            double r2 = p2.Distance;

            double t1 = p1.Angle;
            double t2 = p2.Angle;

            return Math.Sqrt(r1 * r1 + r2 * r2 - 2 * r1 * r2 * Math.Cos(t1 - t2));
        }
        private double CalculateXYDistancePoint(PointD p1, PointD p2)
        {
            double x1 = p1.X;
            double y1 = p1.Y;
            double x2 = p2.X;
            double y2 = p2.Y;

            return Math.Sqrt(Math.Pow(x1 - x2, 2) + Math.Pow(y1 - y2, 2));
        }
        #endregion
        #region Conversion
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

        private PolarPointRssi ConvertXYToPolarCoord(PointD point)
        {
            double X = point.X;
            double Y = point.Y;


            double distance = Math.Sqrt(X * X + Y * Y);
            double angle;
            if (X == Y)
            {
                angle = Math.PI / 2;
            }
            else
            {
                angle = Math.Atan(Y / X);
            }
            angle += (X < 0) ? Math.PI : 0;
            return new PolarPointRssi(angle, distance, 0);
        }

        private PointD ConvertAbsoluteToRelativeCoord(PointD point)
        {
            PolarPointRssi pointRssi = ConvertXYToPolarCoord(point);
            return ConvertPolarToRelativeCoord(pointRssi);
        }

        #endregion
        #region Others
        private int GetIndexOfAngle(List<double> angle_array, double angle)
        {
            if (angle_array.Count == 0)
            {
                return 1;
            }
            int i, index = 0;
            double min = Math.Abs(angle - angle_array[0]);
            for (i = 0; i < angle_array.Count ; i++)
            {
                double delta = Math.Abs(angle - angle_array[i]);
                if (delta < min)
                {
                    min = delta;
                    index = i;
                }
            }
            return index;
        }

        private PointD GetMediumPoint(List<PointD> points)
        {
            double X = 0;
            double Y = 0;
            foreach(PointD point in points)
            {
                X += point.X;
                Y += point.Y;
            }
            X /= points.Count();
            Y /= points.Count();

            return new PointD(X, Y);
        }
        #endregion
        #endregion
    }
}
