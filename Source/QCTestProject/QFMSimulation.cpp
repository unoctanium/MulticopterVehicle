
#include "QFMComponent.h"


//Actual simulation
void UQuadcopterFlightModel::Simulate(float DeltaTime, FBodyInstance* bodyInst) {

    // only do something if time ellapsed
    if (DeltaTime <= 0.0f) { return; }

	//double start = FPlatformTime::Seconds();
	//UE_LOG(LogTemp, Warning, TEXT("timestamp %f"), FPlatformTime::Seconds());


	// read new Pilot Input
    PilotInput.Tock(DeltaTime);

	// Update the Attitude & Heading Reference System
	AHRS.Tock(DeltaTime);

	// Call Flight Controller to calculate angine outputs based on actual attitude and pilot input
	AttitudeController.Tock(DeltaTime);

	// update Position Controller
	PositionController.Tock(DeltaTime);

	// update EngineController
	EngineController.Tock(DeltaTime);

	// And Apply Forces calculated in Engine Control
	//AddLocalForceZ(EngineController.GetTotalThrust());
	//AddLocalTorqueRad(EngineController.GetTotalTorque());


    #ifdef WITH_EDITOR
    
        if (Debug.DebugScreen) {
		

			if (Debug.PrintAHRS) {
				AHRS.Debug(Debug.Color, Debug.FontSize);
            }

			if (Debug.PrintVehicle) {
				Vehicle.Debug(Debug.Color, Debug.FontSize);
			}

			if (Debug.PrintInput) {
				PilotInput.Debug(Debug.Color, Debug.FontSize);
			}

			if (Debug.PrintAttitudeControl) {
				AttitudeController.Debug(Debug.Color, Debug.FontSize);
			}
			
			if (Debug.PrintPositionControl) {
				PositionController.Debug(Debug.Color, Debug.FontSize);
			}

			if (Debug.PrintEngineControl) {
				EngineController.Debug(Debug.Color, Debug.FontSize);
			}
			
			//Debug.Debug(Debug.Color, Debug.FontSize);

			//GEngine->AddOnScreenDebugMessage(-1, 0, Debug.Color, TEXT("Phyics Object") + Parent->GetName(), true, Debug.FontSize);

        }
    #endif
    
	//double end = FPlatformTime::Seconds();
	//UE_LOG(LogTemp, Warning, TEXT("code executed in %f seconds."), end-start);

    
}




/* --- Vehicle Forces Related Stuff ---*/


// Call this to add Linear force to our parent
void UQuadcopterFlightModel::AddLocalForceZ(FVector forceToApply)
{
	FVector finalLocalForce = Parent->GetUpVector() * forceToApply.Z * 100.0f; // F = ma, so kg * ((cm/s)/s), so it's actually kg cm s^-2 => multiply by 100 to convert from m to cm
	BodyInstance->AddForce(finalLocalForce, false, false);
	//Trajectory.LinearAcceleration = forceToApply.Z / Mass / 100.0f; // To Set it in m/s
}

// Call this to add Angular force to our parent
void UQuadcopterFlightModel::AddLocalTorqueRad(FVector torqueToApply)
{
	FVector finalLocalTorque = Parent->GetComponentQuat().RotateVector(torqueToApply) * 10000.0f; // T = r x F, so kg cm^2 s^-2 => multiply by 10000 to convert from m^2 to cm^2
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

