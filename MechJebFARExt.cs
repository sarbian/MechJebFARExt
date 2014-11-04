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

        FieldInfo FieldClIncrementFromRear;
        FieldInfo FieldStall;

        FieldInfo FieldCurrentLift;
        FieldInfo FieldCurrentDrag;

        FieldInfo FieldPitchLocation;
        FieldInfo FieldYawLocation;
        FieldInfo FieldRollLocation;

        private void partModuleUpdate(PartModule pm)
        {
            if (isFarLoaded && pm is FARControllableSurface)
            {
                if (vessel.staticPressure > 0)
                {
                    FARControllableSurface fcs = (FARControllableSurface)pm;

                    Vector3d forcePosition = fcs.AerodynamicCenter - vesselState.CoM;
                    Vector3 velocity = fcs.GetVelocity(fcs.AerodynamicCenter);

                    // First we save the curent state of the part

                    double stall = fcs.GetStall();
                    double cl = fcs.Cl;
                    double cd = fcs.Cd;
                    double cm = fcs.Cm;
                    double YmaxForce = fcs.YmaxForce;
                    double XZmaxForce = fcs.XZmaxForce;

                    float currentLift = (float)(FieldCurrentLift.GetValue(fcs));
                    float currentDrag = (float)(FieldCurrentDrag.GetValue(fcs));

                    double maxdeflect = 0;
                    if (fcs.pitchaxis)
                    {
                        maxdeflect += (double)(FieldPitchLocation.GetValue(fcs));
                    }
                    if (fcs.yawaxis)
                    {
                        maxdeflect += (double)(FieldPitchLocation.GetValue(fcs));
                    }
                    if (fcs.rollaxis)
                    {
                        maxdeflect += (double)(FieldPitchLocation.GetValue(fcs));
                    }

                    maxdeflect = FARMathUtil.Clamp(maxdeflect, -Math.Abs(fcs.maxdeflect), Math.Abs(fcs.maxdeflect));
                    
                    double ClIncrementFromRear = (double)(FieldClIncrementFromRear.GetValue(fcs));

                    double machNumber = fcs.GetMachNumber(mainBody, (float)vessel.altitude, velocity);
                    fcs.YmaxForce = double.MaxValue;
                    fcs.XZmaxForce = double.MaxValue;

                    // Then we turn it one way
                    double AoA = fcs.CalculateAoA(velocity, maxdeflect);
                    vesselState.ctrlTorqueAvailable.Add(vessel.GetTransform().InverseTransformDirection(Vector3.Cross(forcePosition, fcs.CalculateForces(velocity, machNumber, AoA))));

                    // We restore it to the initial state
                    FieldStall.SetValue(fcs as FARWingAerodynamicModel, stall);
                    fcs.Cl = cl;
                    fcs.Cd = cd;
                    fcs.Cm = cm;

                    FieldClIncrementFromRear.SetValue(fcs, ClIncrementFromRear);

                    // And the other way
                    AoA = fcs.CalculateAoA(velocity, -maxdeflect);
                    vesselState.ctrlTorqueAvailable.Add(vessel.GetTransform().InverseTransformDirection(Vector3.Cross(forcePosition, fcs.CalculateForces(velocity, machNumber, AoA))));

                    // And in the end we restore its initial state

                    FieldStall.SetValue(fcs as FARWingAerodynamicModel, stall);
                    fcs.Cl = cl;
                    fcs.Cd = cd;
                    fcs.Cm = cm;
                    fcs.YmaxForce = YmaxForce;
                    fcs.XZmaxForce = XZmaxForce;

                    FieldCurrentLift.SetValue(fcs as FARWingAerodynamicModel, currentLift);
                    FieldCurrentDrag.SetValue(fcs as FARWingAerodynamicModel, currentDrag);

                    FieldClIncrementFromRear.SetValue(fcs, ClIncrementFromRear);
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

            if (!isFarLoaded)
                return;

            FieldClIncrementFromRear = typeof(FARWingAerodynamicModel).GetField("ClIncrementFromRear", BindingFlags.NonPublic | BindingFlags.GetField | BindingFlags.Instance);
            FieldStall = typeof(FARWingAerodynamicModel).GetField("stall", BindingFlags.NonPublic | BindingFlags.GetField | BindingFlags.Instance);

            FieldCurrentLift = typeof(FARWingAerodynamicModel).GetField("currentLift", BindingFlags.NonPublic | BindingFlags.GetField | BindingFlags.Instance);
            FieldCurrentDrag = typeof(FARWingAerodynamicModel).GetField("currentDrag", BindingFlags.NonPublic | BindingFlags.GetField | BindingFlags.Instance);

            FieldPitchLocation = typeof(FARControllableSurface).GetField("PitchLocation", BindingFlags.NonPublic | BindingFlags.GetField | BindingFlags.Instance);
            FieldYawLocation = typeof(FARControllableSurface).GetField("YawLocation", BindingFlags.NonPublic | BindingFlags.GetField | BindingFlags.Instance);
            FieldRollLocation = typeof(FARControllableSurface).GetField("RollLocation", BindingFlags.NonPublic | BindingFlags.GetField | BindingFlags.Instance);;

            if (FieldClIncrementFromRear != null && FieldStall != null &&
                FieldCurrentLift != null && FieldCurrentDrag != null &&
                FieldPitchLocation != null && FieldYawLocation != null && FieldRollLocation != null)
            {
                print("MechJebFARExt adding MJ2 callback");
                vesselState.vesselStatePartModuleExtensions.Add(partModuleUpdate);

                vesselState.TerminalVelocityCall = TerminalVelocityFAR;
            }
            else
            {
                isFarLoaded = false;
                string status = "MechJebFARExt : failure to initialize reflection calls, a new version may be required";
                ScreenMessages.PostScreenMessage(status, 10, ScreenMessageStyle.UPPER_CENTER);
                print(status);
            }
        }
    }
}
