
#include "QFMComponent.h"


//Actual simulation
void UQuadcopterFlightModel::Simulate(FBodyInstance* bodyInst, float DeltaTime) {

    // only do something if time ellapsed
    if (DeltaTime <= 0.0f) { return; }

	// read new Pilot Input
    PilotInput.Tock(DeltaTime);

	// Update the Attitude & Heading Reference System
	AHRS.Tock(DeltaTime);

	// Call Flight Controller to calculate angine outputs based on actual attitude and pilot input
	AttitudeController.Tock(DeltaTime, DesiredPilotInput);

	// update Position Controller
	PositionController.Tock(DeltaTime);

	// update EngineController
	EngineController.Tock(DeltaTime, PilotInput.GetDesiredInput(), FrameMode);

	// And Apply Forces calculated in Engine Control
	AddLocalForceZ(EmgineController.GetTotalThrust());
	AddLocalTorqueRad(EngineController.GetTotalTorque());

    #ifdef WITH_EDITOR
    
        if (Debug.DebugScreen) {
		

			if (Debug.PrintAHRS) {
				AHRS.Debug(Debug.Color, Debug.FontSize);
            }

			if (Debug.PrintVehicle) {
				GEngine->AddOnScreenDebugMessage(-1, 0, Debug.Color, FString::Printf(TEXT("Center of Mass (m): X=%f Y=%f Z=%f"), CenterOfMass.X, CenterOfMass.Y, CenterOfMass.Z), true, Debug.FontSize);
				GEngine->AddOnScreenDebugMessage(-1, 0, Debug.Color, FString::Printf(TEXT("Moment of intertia (kg*m^2): X=%f Y=%f Z=%f"), InertiaTensor.X / 10000.0f, InertiaTensor.Y / 10000.0f, InertiaTensor.Z / 10000.0f), true, Debug.FontSize);
				GEngine->AddOnScreenDebugMessage(-1, 0, Debug.Color, FString::Printf(TEXT("Mass (kg): %f"), Mass), true, Debug.FontSize);

			}

			if (Debug.PrintInput) {
				PilotInput.Debug(Debug.Color, Debug.FontSize);
			}

			if (Debug.AttitudeControl) {
				AttitudeController.Debug(Debug.Color, Debug.FontSize);
			}
			
			if (Debug.PositionControl) {
				PositionController.Debug(Debug.Color, Debug.FontSize);
			}

			if (Debug.PrintEngineControl) {
				EngineControl.Debug(Debug.Color, Debug.FontSize)
			}
			
			//Debug.Debug(Debug.Color, Debug.FontSize);

			GEngine->AddOnScreenDebugMessage(-1, 0, Debug.Color, TEXT("Phyics Object") + Parent->GetName(), true, Debug.FontSize);

        }
    #endif
    
    
}




/* --- Vehicle Forces Related Stuff ---*/


// Call this to add Linear force to our parent
void UQuadcopterFlightModel::AddLocalForceZ(FVector forceToApply)
{
	FVector finalLocalForce = Parent->GetUpVector() * forceToApply.Z;
	BodyInstance->AddForce(finalLocalForce, false, false);
	//Trajectory.LinearAcceleration = forceToApply.Z / Mass / 100.0f; // To Set it in m/s
}

// Call this to add Angular force to our parent
void UQuadcopterFlightModel::AddLocalTorqueRad(FVector torqueToApply)
{
	FVector finalLocalTorque = Parent->GetComponentQuat().RotateVector(torqueToApply);
	BodyInstance->AddTorqueInRadians(finalLocalTorque, false, false);   
	//Trajectory.AngularAcceleration = FVector(torqueToApply / InertiaTensor); // How to set it in deg / s^2 ???????
}


/*
// Calculate forces to apply to get a desired result
///
FVector UQuadcopterFlightModel::GetLinearImpulseToApply(FVector vDeltaInUEU)
{
	return (Vehicle->Mass * vDeltaInUEU);
}
FVector UQuadcopterFlightModel::GetLinearForceToApply(FVector dVelocityInUEU, float dTimeInSec)
{
	return (Vehicle->Mass * dVelocityInUEU / dTimeInSec);
}
FVector UQuadcopterFlightModel::GetAngularImpuleToApplyRad(FVector dAngularVerlocityInRad)
{
	return (Vehicle->InertiaTensor * dAngularVerlocityInRad);
}
FVector UQuadcopterFlightModel::GetAngularForceToApplyRad(FVector dAngularVerlocityInRad, float dTimeInSec)
{
	return (Vehicle->InertiaTensor * dAngularVerlocityInRad / dTimeInSec);
}
FVector UQuadcopterFlightModel::GetRadAngularImpuleToApplyFromDegVelocityDelta(FVector dAngularVerlocityInDeg)
{
	return (Vehicle->InertiaTensor * FMath::DegreesToRadians(dAngularVerlocityInDeg));
}
FVector UQuadcopterFlightModel::GetRadAngularForceToApplyFromDegVelocityDelta(FVector dAngularVerlocityInDeg, float dTimeInSec)
{
	return (Vehicle->InertiaTensor * FMath::DegreesToRadians(dAngularVerlocityInDeg) / dTimeInSec);
}

*/

