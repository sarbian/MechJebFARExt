using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using MuMech;
using UnityEngine;
using ferram4;
using System.Reflection;

namespace MuMech
{
    public class MechJebFARExt : ComputerModule
    {
        public MechJebFARExt(MechJebCore core) : base(core) { }

        bool isFarLoaded = false;

        private void partModuleUpdate(PartModule pm)
        {
            if (isFarLoaded && pm is FARControllableSurface)
            {
                // Sarbian : TODO limit fcs.maxdeflect when all axes are not used ?
                FARControllableSurface fcs = (FARControllableSurface)pm;

                Vector3d forcePosition = fcs.AerodynamicCenter - vesselState.CoM;
                Vector3 velocity = fcs.GetVelocity(fcs.AerodynamicCenter);

                if (velocity.magnitude > 0.1f)         //Don't Bother if it's not moving or in space
                {
                    double stall = fcs.GetStall();
                    double cl = fcs.GetCl();
                    double cd = fcs.GetCd();
                    // ClIncrementFromRear = fcs.ClIncrementFromRear;

                    double ClIncrementFromRear = (double)(typeof(FARWingAerodynamicModel).GetField("ClIncrementFromRear", BindingFlags.NonPublic | BindingFlags.GetField | BindingFlags.Instance).GetValue(fcs));
                    // PartInFrontOf = fcs.PartInFrontOf
                    Part PartInFrontOf = (Part)(typeof(FARWingAerodynamicModel).GetField("PartInFrontOf", BindingFlags.NonPublic | BindingFlags.GetField | BindingFlags.Instance).GetValue(fcs));

                    double WClIncrementFromRear = 0;
                    if (PartInFrontOf != null)
                    {
                        FARWingAerodynamicModel w = PartInFrontOf.GetComponent<FARWingAerodynamicModel>();
                        WClIncrementFromRear = (double)(typeof(FARWingAerodynamicModel).GetField("ClIncrementFromRear", BindingFlags.NonPublic | BindingFlags.GetField | BindingFlags.Instance).GetValue(w));
                    }

                    double machNumber = fcs.GetMachNumber(mainBody, (float)vessel.altitude, velocity);
                    double AoA = fcs.CalculateAoA(velocity, fcs.maxdeflect);
                    vesselState.ctrlTorqueAvailable.Add(vessel.GetTransform().InverseTransformDirection(Vector3.Cross(forcePosition, fcs.CalculateForces(velocity, machNumber, AoA))));

                    //fcs.stall = stall
                    typeof(FARWingAerodynamicModel).GetField("stall", BindingFlags.NonPublic | BindingFlags.GetField | BindingFlags.Instance).SetValue(fcs as FARWingAerodynamicModel, stall);
                    fcs.Cl = cl;
                    fcs.Cd = cd;

                    // fcs.ClIncrementFromRear = ClIncrementFromRear;
                    typeof(FARWingAerodynamicModel).GetField("ClIncrementFromRear", BindingFlags.NonPublic | BindingFlags.GetField | BindingFlags.Instance).SetValue(fcs, ClIncrementFromRear);
                    if (PartInFrontOf != null)
                    {
                        FARWingAerodynamicModel w = PartInFrontOf.GetComponent<FARWingAerodynamicModel>();
                        // fcs.PartInFrontOf.ClIncrementFromRear = WClIncrementFromRear
                        typeof(FARWingAerodynamicModel).GetField("ClIncrementFromRear", BindingFlags.NonPublic | BindingFlags.GetField | BindingFlags.Instance).SetValue(w, WClIncrementFromRear);
                    }

                    AoA = fcs.CalculateAoA(velocity, -fcs.maxdeflect);
                    vesselState.ctrlTorqueAvailable.Add(vessel.GetTransform().InverseTransformDirection(Vector3.Cross(forcePosition, fcs.CalculateForces(velocity, machNumber, AoA))));

                    //fcs.stall = stall
                    typeof(FARWingAerodynamicModel).GetField("stall", BindingFlags.NonPublic | BindingFlags.GetField | BindingFlags.Instance).SetValue(fcs as FARWingAerodynamicModel, stall);
                    fcs.Cl = cl;
                    fcs.Cd = cd;

                    // fcs.ClIncrementFromRear = ClIncrementFromRear;
                    typeof(FARWingAerodynamicModel).GetField("ClIncrementFromRear", BindingFlags.NonPublic | BindingFlags.GetField | BindingFlags.Instance).SetValue(fcs, ClIncrementFromRear);
                    if (PartInFrontOf != null)
                    {
                        FARWingAerodynamicModel w = PartInFrontOf.GetComponent<FARWingAerodynamicModel>();
                        // fcs.PartInFrontOf.ClIncrementFromRear = WClIncrementFromRear
                        typeof(FARWingAerodynamicModel).GetField("ClIncrementFromRear", BindingFlags.NonPublic | BindingFlags.GetField | BindingFlags.Instance).SetValue(w, WClIncrementFromRear);
                    }
                }
            }
        }

        public double TerminalVelocityFAR()
        {
            if (vesselState.altitudeASL > mainBody.RealMaxAtmosphereAltitude()) return double.PositiveInfinity;

            return FARAPI.GetActiveControlSys_TermVel();
        }


        public override void OnStart(PartModule.StartState state)
        {
            isFarLoaded = AssemblyLoader.loadedAssemblies.Any(a => a.assembly.GetName().Name == "FerramAerospaceResearch");

            if (isFarLoaded)
            {
                print("MechJebFARExt adding MJ2 callback");
                vesselState.vesselStatePartModuleExtensions.Add(partModuleUpdate);

                vesselState.TerminalVelocityCall = TerminalVelocityFAR;
            }
        }
    }
}
