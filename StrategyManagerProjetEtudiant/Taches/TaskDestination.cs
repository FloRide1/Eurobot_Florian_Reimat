using Constants;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using Utilities;

namespace StrategyManagerProjetEtudiantNS
{
    public enum TaskDestinationState
    {
        Arret,
        Waypoint,
        Destination,
        AvanceEnCours,
    }

    public class TaskDestination
    {
        StrategyEurobot parent;
        Thread TaskThread;
        public TaskDestinationState state = TaskDestinationState.Arret;

        Stopwatch sw = new Stopwatch();
        
        public TaskDestination(StrategyEurobot parent)
        {
            this.parent = parent;
            TaskThread = new Thread(TaskThreadProcess);
            TaskThread.IsBackground = true;
            TaskThread.Start();
            sw.Stop();
            sw.Reset();
        }

        public void SetTaskState(TaskDestinationState state)
        {
            this.state = state;
        }
             

        void TaskThreadProcess()
        {
            while (true)
            {
                switch (state)
                {
                    case TaskDestinationState.Arret:
                        sw.Restart();
                        if (parent.localWorldMap.DestinationLocation != null)
                        {
                            state = TaskDestinationState.Destination;
                        }
                        else if (parent.localWorldMap.WaypointLocations.Count >= 1)
                        {
                            state = TaskDestinationState.Waypoint;
                        }
                        break;

                    case TaskDestinationState.Waypoint:
                        sw.Restart();

                        if (parent.localWorldMap.DestinationLocation != null)
                        {
                            state = TaskDestinationState.Destination;
                        }
                        else
                        {

                            if (parent.localWorldMap.WaypointLocations.Count >= 1)
                            {
                                state = TaskDestinationState.AvanceEnCours;
                            }
                        }
                        break;
                    case TaskDestinationState.Destination:
                        sw.Restart();
                        break;
                    case TaskDestinationState.AvanceEnCours:
                        if (sw.ElapsedMilliseconds > 500)
                            state = TaskDestinationState.Arret;                           
                        break;
                    default:
                        break;
                }
                Thread.Sleep(100);
            }
        }
    }
}
