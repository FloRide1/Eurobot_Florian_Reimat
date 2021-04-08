﻿using System;
using System.Drawing;
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

            List<Cup> cups = FloCupDetector(validPoint,validPoint.Count(), 75);
            List<PolarPointRssi> processedPoints = new List<PolarPointRssi>();

            foreach (Cup c in cups)
            {
                processedPoints.Add(c.pos);
            }

            List<Segment> Lines = new List<Segment>();
            Lines.Add(DetectGlobalLine(polarPointRssi, 1d, 0d, 5d, 3, 0.2d));
            OnProcessLidarLineDataEvent?.Invoke(this, Lines);

            RawLidarArgs processLidar = new RawLidarArgs() { RobotId = robotId, LidarFrameNumber = LidarFrame, PtList = processedPoints };
            OnProcessLidarDataEvent?.Invoke(this, processLidar);

            OnProcessLidarPolarDataEvent?.Invoke(this, processedPoints);
            OnProcessLidarXYDataEvent?.Invoke(this, ConvertRssiToXYCoord(processedPoints));
        }

        public List<Cup> FloCupDetector(List<PolarPointRssi> points, int lidarRes, int lowRes)
        {
            List<PolarPointRssi> lowResPoints = new List<PolarPointRssi> { };
            List<PolarPointRssi> moddedLowResPoints = new List<PolarPointRssi> { };

            for (int i = 0; i < lowRes; i++)
            {
                lowResPoints.Add(points[(lidarRes / lowRes) * i]);
            }

            foreach(PolarPointRssi pieceOfShit in lowResPoints)
            {
                moddedLowResPoints.Add(new PolarPointRssi(pieceOfShit.Angle, pieceOfShit.Distance, pieceOfShit.Rssi));
            }

            for (int i = 2; i < lowRes - 2 ; i++)
            {
                double avrgDist = 0;
                for(int ii = -2 ; ii < 3 ; ii++)
                {
                    avrgDist += lowResPoints[i + ii].Distance;
                }
                moddedLowResPoints[i].Distance = avrgDist / 5;
            }

            moddedLowResPoints[0].Distance = 0;
            moddedLowResPoints[1].Distance = 0;
            moddedLowResPoints[lowRes - 1].Distance = 0;
            moddedLowResPoints[lowRes - 2].Distance = 0;

            List<List<PolarPointRssi>> clusters = new List<List<PolarPointRssi>> { };
            List<Cup> cups = new List<Cup> { };
            List<PolarPointRssi> pois = new List<PolarPointRssi> { };

            for (int i = 0; i < lowRes; i++)
            {
                if(points[i * (lidarRes/lowRes)].Distance < moddedLowResPoints[i].Distance * 0.97)
                {
                    for (int ii = (lidarRes / lowRes) * (i - 1); ii < (lidarRes / lowRes) * (i + 1); ii++)
                    {
                        //if (points[ii].Distance < moddedLowResPoints[i].Distance)
                        //{
                            pois.Add(points[ii]);
                        //}
                    }
                }
                else
                {
                    if (pois.Count() > 0)
                    {
                        clusters.Add(pois);
                    }
                    pois.Clear();
                }
            }

            foreach (List<PolarPointRssi> things in clusters)
            {
                cups.Add(new Cup(things[0], Color.White));
            }

            return cups;
        }

        public List<ClusterObjects> DetectClusterOfPoint(List<PolarPointRssi> pointsList, double thresold)
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

        #region Gies Detection
        List<PolarCourbure> ExtractCurvature(List<PolarPointRssi> ptList)
        {

            /// Implanation basée sur 
            /// "Natural landmark extraction for mobile robot navigation based on an adaptive curvature estimation"
            /// P. Nunez, R. Vazquez-Martın, J.C. del Toro, A. Bandera, F. Sandoval
            /// Robotics and Autonomous Systems 56 (2008) 247–264
            /// 

            List<double> ptListX = new List<double>();
            List<double> ptListY = new List<double>();
            for (int i = 0; i < ptList.Count; i++)
            {
                ptListX.Add(ptList[i].Distance * Math.Cos(ptList[i].Angle));
                ptListY.Add(ptList[i].Distance * Math.Sin(ptList[i].Angle));
            }

            double[] DiffX_iPlus1_i_List = new double[ptList.Count];
            double[] DiffY_iPlus1_i_List = new double[ptList.Count];
            double[] DiffX_i_iMoins1_List = new double[ptList.Count];
            double[] DiffY_i_iMoins1_List = new double[ptList.Count];
            double[] Dist_iPlus1_i_List = new double[ptList.Count];
            double[] Dist_i_iMoins1_List = new double[ptList.Count];

            bool[] DiscontinuityList = new bool[ptList.Count];

            /// On commence par calculer les deltaX delatY et distances entre deux pts lidar successifs
            for (int i = 0; i < ptList.Count - 1; i++)
            {
                int iPlus1 = i + 1;
                if (iPlus1 > +ptList.Count)
                    iPlus1 -= ptList.Count;
                int iMoins1 = i - 1;
                if (iMoins1 < 0)
                    iMoins1 += ptList.Count;

                DiffX_iPlus1_i_List[i] = ptListX[iPlus1] - ptListX[i];
                DiffY_iPlus1_i_List[i] = ptListY[iPlus1] - ptListY[i];
                DiffX_i_iMoins1_List[i] = ptListX[i] - ptListX[iMoins1];
                DiffY_i_iMoins1_List[i] = ptListY[i] - ptListY[iMoins1];
                Dist_iPlus1_i_List[i] = Math.Sqrt(DiffX_iPlus1_i_List[i] * DiffX_iPlus1_i_List[i] + DiffY_iPlus1_i_List[i] * DiffY_iPlus1_i_List[i]);
                Dist_i_iMoins1_List[i] = Math.Sqrt(DiffX_i_iMoins1_List[i] * DiffX_i_iMoins1_List[i] + DiffY_i_iMoins1_List[i] * DiffY_i_iMoins1_List[i]);

                if (Dist_iPlus1_i_List[i] < 0.5)
                    DiscontinuityList[i] = false;
                else
                    DiscontinuityList[i] = true;
            }

            /// Pour chaque point lidar, on regarde son voisinage en s'écartant du point vers la droite dans un premier temps
            /// puis vers la gauche dans un second temps
            /// on compare la somme des distances entre les pts lidar successifs (sui décrit la longueur de la courbe lidar)
            /// et la distance directe entre le pt considéré et le voisinage en s'écartant
            /// Cette distance doit être à peu près la même si le voisinage est un segment de droite
            /// Si on a une courbe, il doit augmenter fortement
            /// Quand la différence entre les deux devient supérieure à un certain seuil, on considère qu'on a une discontinuité.
            /// 

            int[] neighbourIndexSupList = new int[ptList.Count];
            int[] neighbourIndexInfList = new int[ptList.Count];

            double K = 0.01;
            double lidarNoise = 0.005;
            int maxNeighboor = 20;
            for (int i = 0; i < ptList.Count; i++)
            {
                /// On part vers le indice croissant en premier
                double distanceSum = 0;
                double distanceDirecte = 0;
                int j = i;
                int nbNeighboor = 0;
                do
                {
                    j++;
                    nbNeighboor++;
                    if (j >= ptList.Count) /// Gestion des index dépassant la taille du tableau
                        j -= ptList.Count;
                    distanceSum += Dist_i_iMoins1_List[j]; /// On ajoute la distance entre les pts j et j-1
                    distanceDirecte = Toolbox.Distance(new PointD(ptListX[i], ptListY[i]), new PointD(ptListX[j], ptListY[j]));
                }
                while (distanceSum - distanceDirecte < K + nbNeighboor * lidarNoise
                        && nbNeighboor < maxNeighboor);
                neighbourIndexSupList[i] = j - 1; /// On enlève 1 pour éviter de prendre en compte le pt qui pose problème
                if (neighbourIndexSupList[i] < 0) /// Gestion des index dépassant la taille du tableau
                    neighbourIndexSupList[i] += ptList.Count;

                /// On part vers le indice decroissant en second
                distanceSum = 0;
                distanceDirecte = 0;
                j = i;
                nbNeighboor = 0;
                do
                {
                    j--;
                    nbNeighboor++;
                    if (j < 0) /// Gestion des index dépassant la taille du tableau
                        j += ptList.Count;
                    distanceSum += Dist_iPlus1_i_List[j];
                    distanceDirecte = Toolbox.Distance(new PointD(ptListX[i], ptListY[i]), new PointD(ptListX[j], ptListY[j]));
                }
                while (distanceSum - distanceDirecte < K + nbNeighboor * lidarNoise
                            && nbNeighboor < maxNeighboor);
                neighbourIndexInfList[i] = j + 1; /// On rajoute 1 pour éviter de prendre en compte le pt qui pose problème
                if (neighbourIndexInfList[i] >= ptList.Count) /// Gestion des index dépassant la taille du tableau
                    neighbourIndexInfList[i] -= ptList.Count;
            }

            /// Calcul de la courbure en tout point
            /// Pour chacun des points, on calcul les deux vecteurs pt-> extrémité du voisinage à droite et à gauche
            /// On approxime la courbure comme étant le cos-1 du produit scalaire divisé par la norme des vecteurs
            List<PolarCourbure> curvatureList = new List<PolarCourbure>();
            List<PolarPointRssi> curvatureListDebug = new List<PolarPointRssi>();

            for (int i = 0; i < ptList.Count; i++)
            {
                double vectVoisinageMaxSupX = ptListX[neighbourIndexSupList[i]] - ptListX[i];
                double vectVoisinageMaxSupY = ptListY[neighbourIndexSupList[i]] - ptListY[i];
                double vectVoisinageMaxInfX = ptListX[i] - ptListX[neighbourIndexInfList[i]];
                double vectVoisinageMaxInfY = ptListY[i] - ptListY[neighbourIndexInfList[i]];

                double dotProduct = vectVoisinageMaxInfX * vectVoisinageMaxSupX + vectVoisinageMaxInfY * vectVoisinageMaxSupY;
                double normVectVoisinageMaxSup = Math.Sqrt(vectVoisinageMaxSupX * vectVoisinageMaxSupX + vectVoisinageMaxSupY * vectVoisinageMaxSupY);
                double normVectVoisinageMaxInf = Math.Sqrt(vectVoisinageMaxInfX * vectVoisinageMaxInfX + vectVoisinageMaxInfY * vectVoisinageMaxInfY);
                double courbure = Math.Acos(dotProduct / (normVectVoisinageMaxInf * normVectVoisinageMaxSup));

                courbure = Toolbox.ModuloPiAngleRadian(courbure);

                curvatureList.Add(new PolarCourbure(ptList[i].Angle, courbure, DiscontinuityList[i]));
                curvatureListDebug.Add(new PolarPointRssi(ptList[i].Angle, courbure, 0));
            }
            
            // OnLidarBalisePointListForDebug(robotId, curvatureListDebug);
            return curvatureList;

        }

        List<PolarPointRssi> ExtractLinesFromCurvature(List<PolarPointRssi> ptList, List<PolarCourbure> curvatureList)
        {
            bool isLineStarted = false;
            bool isLineDiscontinuous = true;
            int lineBeginIndex = 0;

            List<PolarPointRssi> linePoints = new List<PolarPointRssi>();

            double thetaMinCourbure = 0.3;

            for (int i = 0; i < curvatureList.Count; i++)
            {
                if (!isLineStarted)
                {
                    //On n'est pas dans une ligne
                    if (curvatureList[i].Courbure < thetaMinCourbure)
                    {
                        isLineStarted = true;
                        lineBeginIndex = i;
                        isLineDiscontinuous = curvatureList[i].Discontinuity;
                    }
                }
                else
                {
                    //On est dans une ligne
                    if (curvatureList[i].Discontinuity)
                        isLineDiscontinuous = true;
                    if (curvatureList[i].Courbure >= thetaMinCourbure || curvatureList[i].Discontinuity)
                    {
                        isLineStarted = false;
                        /// On termine la ligne, il faut la valider ou pas
                        /// On la valide si sa longueur est supérieure à 10 pts successifs
                        /// Sinon on la jette
                        /// 
                        var ptInit = new PointD(ptList[lineBeginIndex].Distance * Math.Cos(ptList[lineBeginIndex].Angle), ptList[lineBeginIndex].Distance * Math.Sin(ptList[lineBeginIndex].Angle));
                        var ptEnd = new PointD(ptList[i - 1].Distance * Math.Cos(ptList[i - 1].Angle), ptList[i - 1].Distance * Math.Sin(ptList[i - 1].Angle));
                        double longueurLigne = Toolbox.Distance(ptInit, ptEnd);

                        double angleLine = Math.Atan2(ptEnd.Y - ptInit.Y, ptEnd.X - ptInit.X);
                        double capLinePtInit = Math.Atan2(ptInit.Y, ptInit.X);

                        double ecartAngleLine = Toolbox.ModuloPiDivTwoAngleRadian(angleLine - capLinePtInit);
                        bool lineAlignedWithPoint = false;
                        if (Math.Abs(ecartAngleLine) < Toolbox.DegToRad(10))
                            lineAlignedWithPoint = true;

                        if (longueurLigne > 0.2 && !lineAlignedWithPoint && !isLineDiscontinuous)// On garde les lignes ayant une taille minimum
                        {
                            /// On valide la ligne
                            /// 
                            //Console.WriteLine("Longueur ligne : " + longueurLigne + " - ecartLigne : " + ecartAngleLine);
                            for (int j = lineBeginIndex; j < i; j++)
                            {
                                linePoints.Add(ptList[j]);
                            }
                        }

                    }
                }
            }
            return linePoints;
        }

        List<PolarPointRssi> ExtractCornersFromCurvature(List<PolarPointRssi> ptList, List<PolarCourbure> curvatureList)
        {
            List<PolarPointRssi> cornerPoints = new List<PolarPointRssi>();
            for (int i = 0; i < curvatureList.Count; i++)
            {
                int i_Moins1 = i - 1;
                if (i_Moins1 < 0)
                    i_Moins1 += ptList.Count;
                int i_Plus1 = i + 1;
                if (i_Plus1 >= ptList.Count)
                    i_Plus1 -= ptList.Count;
                if (curvatureList[i].Courbure > curvatureList[i_Moins1].Courbure && curvatureList[i].Courbure > curvatureList[i_Plus1].Courbure && curvatureList[i].Courbure > 1) //On a maximum local de courbure
                {
                    cornerPoints.Add(ptList[i]);
                }
            }
            return cornerPoints;
        }

        #endregion


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


            /// NEED TO REMAKE (SORRY MERGE FAIL HERE)
            return 0d;
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
        #endregion
        #endregion
    }
}
