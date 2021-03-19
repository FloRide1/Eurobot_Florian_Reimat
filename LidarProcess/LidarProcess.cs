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

            
            foreach (BlobObject Blob in Blobs)
            {
                List<PolarPointRssi> blobPointFlo = new List<PolarPointRssi>();
                foreach (PolarPointRssi point in Blob.points)
                {
                    blobPointFlo.Add(new PolarPointRssi(point.Angle, point.Distance * Math.Cos(point.Angle), point.Rssi));
                }

                BlobObject BlobFlo = new BlobObject() { points = blobPointFlo };
                List<Segment> Line = DetectLineFlo(Blob, BlobFlo ,0.015, 35, 5);
                if (Line.Count != 0)
                {
                    Lines.AddRange(Line);
                }
            }
            

            OnProcessLidarLineDataEvent?.Invoke(this, Lines);
            // /!\ Warning NEED TO REMOVE


            RawLidarArgs processLidar = new RawLidarArgs() { RobotId = robotId, LidarFrameNumber = LidarFrame, PtList = pointFlo };
            OnProcessLidarDataEvent?.Invoke(this, processLidar);

            

            processLidar.PtList =  fl0Derivative;
            OnRawLidarDataEvent?.Invoke(this, processLidar);




            
            OnProcessLidarPolarDataEvent?.Invoke(this, processedPoints);
            OnProcessLidarXYDataEvent?.Invoke(this, ConvertRssiToXYCoord(processedPoints));
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
        public Segment DetectLineHough(BlobObject blob, uint thresold = 15, double delta = 0.1, int moy = 5)
        {
            List<PointD> pointList = ConvertRssiToXYCoord(blob.points);

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

            HoughLine Line = new HoughLine();
            foreach (KeyValuePair<double, Dictionary<double, uint>> rho in Matrix)
            {
                foreach (KeyValuePair<double, uint> theta in rho.Value)
                {
                    if (theta.Value >= thresold)
                    {
                        Line = new HoughLine(rho.Key, theta.Key);
                    }
                }
            }

            Segment segment = new Segment();
            if (Line.rho != 0 || Line.theta != 0)
            {
                int i;
                double X1 = 0, Y1 = 0, X2 = 0, Y2 = 0;
                for (i = 0; i < moy; i++)
                {
                    X1 += pointList[i].X;
                    Y1 += pointList[i].Y;
                }
                for (i = 0; i < moy; i++)
                {
                    X2 += pointList[pointList.Count - 1 - i].X;
                    Y2 += pointList[pointList.Count - 1 - i].Y;
                }
                X1 /= moy;
                Y1 /= moy;
                X2 /= moy;
                Y2 /= moy;
                segment = new Segment(X1, Y1, X2, Y2);
            }

            return segment;
        }

        public List<BlobObject> DetectBlob(List<PolarPointRssi> pointList, double thresold, int minSize)
        {
            List<double> derivate = new List<double>();
            
            int i;
            for (i = 0; i < pointList.Count - 1; i++)
            {
                derivate.Add(Math.Abs(pointList[i + 1].Distance - pointList[i].Distance));
            }

            List<BlobObject> validBlob = new List<BlobObject>();
            BlobObject testBlob= new BlobObject();
            for (i = 0; i < derivate.Count; i++)
            {
                
                if (derivate[i] < thresold)
                {
                    testBlob.points.Add(pointList[i]);
                } 
                else
                {
                    if (testBlob.points.Count > minSize)
                    {
                        validBlob.Add(testBlob);
                    }
                    testBlob = new BlobObject();
                }
            }

            return validBlob;
        }

        public Segment DetectLine(BlobObject blob, double thresold, double alignNbr, int moy= 5)
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

        public List<Segment> DetectLineFlo(BlobObject sourceBlob, BlobObject fl0Blob, double thresold, int alignNbr, int moy)
        {
            List<Segment> listSegments = new List<Segment>();
            List<double> fl0Derivative = new List<double>();

            int i;
            for (i = 0; i < fl0Blob.points.Count - 1; i++)
            {
                double point = fl0Blob.points[i+ 1].Distance - fl0Blob.points[i].Distance;
                fl0Derivative.Add(point);
            }
            int indexOfFirst = 0;
            uint nbrOfCurrentAlign = 0;
            for (i = 0; i < fl0Derivative.Count - 1; i++)
            {
                double previousLvl = fl0Derivative[i];
                if (previousLvl + thresold >= fl0Derivative[i+1] && previousLvl - thresold <= fl0Derivative[i + 1])
                {
                    
                    nbrOfCurrentAlign++;

                    if (nbrOfCurrentAlign == 1)
                    {
                        indexOfFirst = i;
                    }
                }
                else if (nbrOfCurrentAlign > 0)
                {
                    if (nbrOfCurrentAlign >= alignNbr)
                    {
                        if (i < sourceBlob.points.Count)
                        {
                            listSegments.Add(CreateLineSegment(sourceBlob.points, indexOfFirst, i, moy));
                        }
                        else
                        {
                            listSegments.Add(CreateLineSegment(sourceBlob.points, indexOfFirst, sourceBlob.points.Count - 1, moy));
                        }
                        
                    }
                    nbrOfCurrentAlign = 0;
                }

            }

            if (nbrOfCurrentAlign >= alignNbr)
            {

                listSegments.Add(CreateLineSegment(sourceBlob.points, indexOfFirst, sourceBlob.points.Count - 1, moy));
            }

            return listSegments;
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
