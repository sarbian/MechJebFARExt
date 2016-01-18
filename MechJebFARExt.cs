using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using MuMech;
using UnityEngine;
using ferram4;
using FerramAerospaceResearch;
using System.Reflection;

namespace MuMech
{
    public class MechJebFARExt : ComputerModule
    {
        public MechJebFARExt(MechJebCore core) : base(core) { }

        bool isFarLoaded = false;

        FieldInfo FieldPitchLocation;
        FieldInfo FieldYawLocation;
        FieldInfo FieldRollLocation;
        FieldInfo FieldAoAsign;
        FieldInfo FieldAoAcurrentFlap;

        private void partModuleUpdate(PartModule pm)
        {
            if (isFarLoaded && pm is FARControllableSurface)
            {
                FARControllableSurface fcs = (FARControllableSurface)pm;
                
                if (vessel.atmDensity > 0)
                {
                    Vector3d forcePosition = fcs.AerodynamicCenter - vesselState.CoM;
                    Vector3 velocity = fcs.GetVelocity();
                    
                    double v_scalar = velocity.magnitude;
                    double rho = FARAeroUtil.GetCurrentDensity(vessel);
                    if (rho <= 0.0 || v_scalar <= 0.1 || fcs.isShielded)
                        return;

                    // First we save the curent state of the part
                    double YmaxForce = fcs.YmaxForce;
                    double XZmaxForce = fcs.XZmaxForce;

                    double AoAcurrentFlap = (double)(FieldAoAcurrentFlap.GetValue(fcs));

                    double MaxAoAdesiredControl = 0;
                    if (fcs.pitchaxis != 0.0)
                    {
                        MaxAoAdesiredControl += (double)(FieldPitchLocation.GetValue(fcs)) * fcs.pitchaxis * 0.01;
                    }
                    if (fcs.yawaxis != 0.0)
                    {
                        MaxAoAdesiredControl += (double)(FieldYawLocation.GetValue(fcs)) * fcs.yawaxis * 0.01; ;
                    }
                    if (fcs.rollaxis != 0.0)
                    {
                        MaxAoAdesiredControl += (double)(FieldRollLocation.GetValue(fcs)) * fcs.rollaxis * 0.01; ;
                    }
                    MaxAoAdesiredControl *= fcs.maxdeflect;
                    if (fcs.pitchaxisDueToAoA != 0.0)
                    {
                        double _AoA = (fcs as FARWingAerodynamicModel).CalculateAoA(velocity.normalized);
                        _AoA = FARMathUtil.rad2deg * Math.Asin(_AoA);
                        if (double.IsNaN(_AoA))
                            _AoA = 0;
                        MaxAoAdesiredControl += _AoA * fcs.pitchaxisDueToAoA * 0.01;
                    }

                    MaxAoAdesiredControl *= (double) (FieldAoAsign.GetValue(fcs));
                    
                    MaxAoAdesiredControl = FARMathUtil.Clamp(MaxAoAdesiredControl, -Math.Abs(fcs.maxdeflect), Math.Abs(fcs.maxdeflect));

                    fcs.YmaxForce = double.MaxValue;
                    fcs.XZmaxForce = double.MaxValue;

                    Vector3 CoM = vessel.findWorldCenterOfMass();
                    Vector3d partPosition = pm.part.Rigidbody.worldCenterOfMass - CoM;
                    Vector3 relpos = vessel.transform.InverseTransformDirection(partPosition);
                    
                    // Then we turn it one way
                    double AoA = fcs.CalculateAoA(velocity, AoAcurrentFlap +  MaxAoAdesiredControl);
                    Vector3 ctrlTorquePos = Vector3.Scale(vessel.GetTransform().InverseTransformDirection(Vector3.Cross(forcePosition, fcs.CalculateForces(velocity, vessel.mach, AoA, rho))), relpos);
                    
                    // We restore it to the initial state
                    AoA = fcs.CalculateAoA(velocity);
                    fcs.CalculateForces(velocity, vessel.mach, AoA, rho);
                    
                    // And we turn it the other way
                    AoA = fcs.CalculateAoA(velocity, AoAcurrentFlap - MaxAoAdesiredControl);
                    Vector3 ctrlTorqueNeg = Vector3.Scale(vessel.GetTransform().InverseTransformDirection(Vector3.Cross(forcePosition, fcs.CalculateForces(velocity, vessel.mach, AoA, rho))), relpos);
                    
                    // And in the end we restore its initial state
                    AoA = fcs.CalculateAoA(velocity);
                    fcs.CalculateForces(velocity, vessel.mach, AoA, rho);
                    
                    fcs.YmaxForce = YmaxForce;
                    fcs.XZmaxForce = XZmaxForce;

                    // TODO : change the value with a factor related to the current part position

                    vesselState.ctrlTorqueAvailablePos += ctrlTorquePos;
                    vesselState.ctrlTorqueAvailableNeg += ctrlTorqueNeg;

                }
            }
        }

        public double TerminalVelocityFAR()
        {
            if (vesselState.altitudeASL > mainBody.RealMaxAtmosphereAltitude()) 
                return double.PositiveInfinity;

            return FARAPI.ActiveVesselTermVelEst();
        }


        public override void OnStart(PartModule.StartState state)
        {
            isFarLoaded = AssemblyLoader.loadedAssemblies.Any(a => a.assembly.GetName().Name == "FerramAerospaceResearch");

            if (!isFarLoaded)
                return;

            FieldPitchLocation = typeof(FARControllableSurface).GetField("PitchLocation", BindingFlags.NonPublic | BindingFlags.GetField | BindingFlags.Instance);
            FieldYawLocation = typeof(FARControllableSurface).GetField("YawLocation", BindingFlags.NonPublic | BindingFlags.GetField | BindingFlags.Instance);
            FieldRollLocation = typeof(FARControllableSurface).GetField("RollLocation", BindingFlags.NonPublic | BindingFlags.GetField | BindingFlags.Instance);

            FieldAoAsign = typeof(FARControllableSurface).GetField("AoAsign", BindingFlags.NonPublic | BindingFlags.GetField | BindingFlags.Instance);

            FieldAoAcurrentFlap = typeof(FARControllableSurface).GetField("AoAcurrentFlap", BindingFlags.NonPublic | BindingFlags.GetField | BindingFlags.Instance);

            if (FieldPitchLocation != null && FieldYawLocation != null && FieldRollLocation != null && FieldAoAcurrentFlap != null)
            {
                print("MechJebFARExt adding MJ2 callback");
                vesselState.vesselStatePartModuleExtensions.Add(partModuleUpdate);

                vesselState.TerminalVelocityCall = TerminalVelocityFAR;
            }
            else
            {
                isFarLoaded = false;
                const string status = "MechJebFARExt : failure to initialize reflection calls, a new version may be required";
                ScreenMessages.PostScreenMessage(status, 10, ScreenMessageStyle.UPPER_CENTER);
                print(status);
            }
        }
    }
}
