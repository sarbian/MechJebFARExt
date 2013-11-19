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

                float stall = fcs.GetStall();
                float cl = fcs.GetCl();
                float cd = fcs.GetCd();
                // ClIncrementFromRear = fcs.ClIncrementFromRear;
                float ClIncrementFromRear = (float)(typeof(FARWingAerodynamicModel).GetField("ClIncrementFromRear", BindingFlags.NonPublic | BindingFlags.GetField | BindingFlags.Instance).GetValue(fcs));
                Part PartInFrontOf = (Part)(typeof(FARWingAerodynamicModel).GetField("PartInFrontOf", BindingFlags.NonPublic | BindingFlags.GetField | BindingFlags.Instance).GetValue(fcs));

                float WClIncrementFromRear=0;
                if (PartInFrontOf != null)
                {
                    FARWingAerodynamicModel w = PartInFrontOf.GetComponent<FARWingAerodynamicModel>();
                    WClIncrementFromRear = (float)(typeof(FARWingAerodynamicModel).GetField("ClIncrementFromRear", BindingFlags.NonPublic | BindingFlags.GetField | BindingFlags.Instance).GetValue(w));
                }

                //print("A Stall " + fcs.GetStall().ToString("F3") + " Cl " + fcs.GetCl() + " Cd " + fcs.GetCd());

                Vector3d forcePosition = fcs.AerodynamicCenter - vesselState.CoM;
                Vector3 velocity = fcs.GetVelocity(fcs.AerodynamicCenter);
                float machNumber = fcs.GetMachNumber(mainBody, (float)vessel.altitude, velocity);
                float AoA = fcs.CalculateAoA(velocity, fcs.maxdeflect);
                vesselState.ctrlTorqueAvailable.Add(vessel.GetTransform().InverseTransformDirection(Vector3.Cross(forcePosition, fcs.CalculateForces(velocity, machNumber, AoA))));
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

                //print("B Stall " + fcs.GetStall().ToString("F3") + " Cl " + fcs.GetCl() + " Cd " + fcs.GetCd());

            }
        }
        
        public override void OnStart(PartModule.StartState state)
        {
            isFarLoaded = AssemblyLoader.loadedAssemblies.Any(a => a.assembly.GetName().Name == "FerramAerospaceResearch");
            print("MechJebFARExt adding MJ2 callback");
            vesselState.vesselStatePartModuleExtensions.Add(partModuleUpdate);
            print("This vessel now has " + vesselState.vesselStatePartModuleExtensions.Count() + " VS PM Ext" );

        }
    }
}
