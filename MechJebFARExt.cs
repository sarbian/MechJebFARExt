using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using MuMech;
using UnityEngine;

namespace MuMech 
{
    public class MechJebFARExt : ComputerModule
    {
        public MechJebFARExt(MechJebCore core) : base(core) { }

        bool isFarLoaded = false;

        private void partModuleUpdate(PartModule pm)
        {
            if (isFarLoaded && pm is ferram4.FARControllableSurface)
            {
                // Sarbian : TODO limit fcs.maxdeflect when all axes are not used ?
                ferram4.FARControllableSurface fcs = (ferram4.FARControllableSurface)pm;
                Vector3d forcePosition = fcs.AerodynamicCenter - vesselState.CoM;
                Vector3 velocity = fcs.GetVelocity(fcs.AerodynamicCenter);
                float machNumber = fcs.GetMachNumber(mainBody, (float)vessel.altitude, velocity);
                float AoA = fcs.CalculateAoA(velocity, fcs.maxdeflect);
                vesselState.ctrlTorqueAvailable.Add(vessel.GetTransform().InverseTransformDirection(Vector3.Cross(forcePosition, fcs.CalculateForces(velocity, machNumber, AoA))));
                AoA = fcs.CalculateAoA(velocity, -fcs.maxdeflect);
                vesselState.ctrlTorqueAvailable.Add(vessel.GetTransform().InverseTransformDirection(Vector3.Cross(forcePosition, fcs.CalculateForces(velocity, machNumber, AoA))));
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
