
#include "QFMComponent.h"


//Actual simulation
void UQuadcopterFlightModel::Simulate(FBodyInstance* bodyInst, float DeltaTime) {

    // only do something if time ellapsed
    if (DeltaTime <= 0.0f) { return; }

    
	// calc the actual Trajectory
	FTransform bodyTransform = Parent->GetComponentTransform();
	Trajectory.Position = bodyTransform.GetTranslation() / 100.0f; // in m
	Trajectory.Rotation = bodyTransform.GetRotation().Rotator(); // in deg
	float OldLinearVelocity = Trajectory.LinearVelocity;
	Trajectory.LinearVelocity = bodyInst->GetUnrealWorldVelocity().Size() / 100.0f; // TAS in m/s
	Trajectory.VelocityVector = bodyInst->GetUnrealWorldVelocity() / 100.0f; // in m / s
	Trajectory.LinearVelocity2D = bodyInst->GetUnrealWorldVelocity().Size2D() / 100.0f; // Speed over ground
	FVector OldAngularVelocity = FVector(Trajectory.AngularVelocity);
	Trajectory.AngularVelocity = FMath::RadiansToDegrees(bodyInst->GetUnrealWorldAngularVelocityInRadians());
	Trajectory.LinearAcceleration = (OldLinearVelocity - Trajectory.LinearVelocity) / DeltaTime;
	Trajectory.AngularAcceleration = (OldAngularVelocity - Trajectory.AngularVelocity) / DeltaTime;



	// Convert Pilot Input to standard intervals (R,P,Y: -1..1, T 0..1)
	FVector4 NewInputStick = FVector4(
		GetMappedAndClampedValueNormal(RollAxisInputInterval, RollAxisInput * InputAxisScale.X),
		GetMappedAndClampedValueNormal(PitchAxisInputInterval, PitchAxisInput * InputAxisScale.Y),
		GetMappedAndClampedValueNormal(YawAxisInputInterval, YawAxisInput * InputAxisScale.Z),
		GetMappedAndClampedValueNormal(ThrottleAxisInputInterval, ThrottleAxisInput * InputAxisScale.W)
	);
	DesiredPilotInput = NewInputStick;
	DesiredPilotInput.W = DesiredPilotInput.W / 2.0f + 0.5f; 



	///////////////////////////
	// THIS IS WORK IN PROGRESS
	///////////////////////////

	///////////////////////////

	///////////////////////////

	FlightController.Tock(DeltaTime, &DesiredPilotInput);

	///////////////////////////

	///////////////////////////





	// Mix Pilot Input to meet Frame Mode 
	MixEngines();
	

	// Apply Engine RPM ... Now: Only Mixed but no Attitude Controller
	SetEnginesFromMixer();
	
	// Control the Engine with Mixer Input
	GetEngineForces();
	
	// And Apply Forces calculated in Engine Control
	AddLocalForceZ(GetEngineThrust());
	AddLocalTorqueRad(GetEngineTorque());


	  
    #ifdef WITH_EDITOR
    
        if (Debug.DebugScreen) {
		
			FVector2D DebugFontSize = FVector2D(1, 1);

			if (Debug.PrintTrajectory) {
				Trajectory.Debug(FColor::White, Debug.DebugFontSize);
            }

			if (Debug.PrintVehicle) {
				GEngine->AddOnScreenDebugMessage(-1, 0, FColor::White, FString::Printf(TEXT("Center of Mass (m): X=%f Y=%f Z=%f"), CenterOfMass.X, CenterOfMass.Y, CenterOfMass.Z), true, DebugFontSize);
				GEngine->AddOnScreenDebugMessage(-1, 0, FColor::White, FString::Printf(TEXT("Moment of intertia (kg*m^2): X=%f Y=%f Z=%f"), InertiaTensor.X / 10000.0f, InertiaTensor.Y / 10000.0f, InertiaTensor.Z / 10000.0f), true, DebugFontSize);
				GEngine->AddOnScreenDebugMessage(-1, 0, FColor::White, FString::Printf(TEXT("Mass (kg): %f"), Mass), true, DebugFontSize);

			}

			if (Debug.PrintInput) {
				GEngine->AddOnScreenDebugMessage(-1, 0, FColor::White, TEXT("Pilot Input: (R,P,Y,T) ") + DesiredPilotInput.ToString(), true, DebugFontSize);
				GEngine->AddOnScreenDebugMessage(-1, 0, FColor::White, FString::Printf(TEXT("Pilot Input: R=%f P=%f Y=%f T=%f"), DesiredPilotInput.X, DesiredPilotInput.Y, DesiredPilotInput.Z, DesiredPilotInput.W), true, DebugFontSize);
				GEngine->AddOnScreenDebugMessage(-1, 0, FColor::White, FString::Printf(TEXT("Raw Axis Input: R=%f P=%f Y=%f T=%f"), RollAxisInput, PitchAxisInput, YawAxisInput, ThrottleAxisInput), true, DebugFontSize);
			}

			if (Debug.PrintMixer) {
				GEngine->AddOnScreenDebugMessage(-1, 0, FColor::White, FString::Printf(TEXT("Mixer %%  : 1=%f 2=%f 3=%f 4=%f"), EngineMixPercent[0], EngineMixPercent[1], EngineMixPercent[2], EngineMixPercent[3]), true, DebugFontSize);
			}
			
			if (Debug.PrintEngineControl) {
				GEngine->AddOnScreenDebugMessage(-1, 0, FColor::White, FString::Printf(TEXT("Engines %%  : 1=%f 2=%f 3=%f 4=%f"), GetEnginePercent(0), GetEnginePercent(1), GetEnginePercent(2), GetEnginePercent(3)), true, DebugFontSize);
				GEngine->AddOnScreenDebugMessage(-1, 0, FColor::White, FString::Printf(TEXT("Engines RPM: 1=%f 2=%f 3=%f 4=%f"), GetEngineRPM(0), GetEngineRPM(1), GetEngineRPM(2), GetEngineRPM(3)), true, DebugFontSize);
				GEngine->AddOnScreenDebugMessage(-1, 0, FColor::White, FString::Printf(TEXT("Thrust / Torque: %s / %s"), *TotalThrust.ToString(), *TotalTorque.ToString()), true, DebugFontSize);
			}
			
			Debug.Debug(FColor::White, Debug.DebugFontSize);

			GEngine->AddOnScreenDebugMessage(-1, 0, FColor::White, TEXT("Phyics Object") + Parent->GetName(), true, Debug.DebugFontSize);

        }
    #endif
    
    
}









/* --- Pilot Input And Attitude Related Stuff ---*/

// Helper Function to Map Pilot Input to [-1..1]
float UQuadcopterFlightModel::GetMappedAndClampedValueNormal(const FVector2D& InputRange, const float Value)
{
    return ( ( Value - InputRange.X ) / ( InputRange.Y - InputRange.X ) ) * 2.0f -1.0f;
	
}
// Helper Function to Map Pilot Input to [OutRange]
float UQuadcopterFlightModel::GetMappedAndClampedValue(const FVector2D& InputRange, const FVector2D& OutputRange, const float Value)
{
	return ((Value - InputRange.X) / (InputRange.Y - InputRange.X)) * (OutputRange.Y - OutputRange.X) + OutputRange.X;
}


/*void UQuadcopterFlightModel::ResetPiloInput()
{
	DesiredPilotInput = FVector4(0.0f, 0.0f, 0.0f, 0.0f);
}*/






/* --- Mixer Related Stuff ---*/


void UQuadcopterFlightModel::MixEngines(void) 
{

	if (FrameMode == EFrameMode::FrameModeCross)
	{
		for (int i = 0; i < 4; i++) {
			EngineMixPercent[i] = (
				DesiredPilotInput.W * MixerQuadCross[i].Throttle +
				DesiredPilotInput.X * MixerQuadCross[i].Roll +
				DesiredPilotInput.Y * MixerQuadCross[i].Pitch +
				DesiredPilotInput.Z * MixerQuadCross[i].Yaw
				);
			
		}
	}
	else if (FrameMode == EFrameMode::FrameModePlus)
	{
		for (int i = 0; i < 4; i++) {
			EngineMixPercent[i] = (
				DesiredPilotInput.W * MixerQuadPlus[i].Throttle +
				DesiredPilotInput.X * MixerQuadPlus[i].Roll +
				DesiredPilotInput.Y * MixerQuadPlus[i].Pitch +
				DesiredPilotInput.Z * MixerQuadPlus[i].Yaw
				);
		}
	}



	float maxMotorPercent = EngineMixPercent[0];

    for (int i = 1; i < 4; i++)
    	if (EngineMixPercent[i] > maxMotorPercent)
    		maxMotorPercent = EngineMixPercent[i];

	for (int i = 0; i < 4; i++)
	{
		// This is a way to still have good gyro corrections if at least one motor reaches its max
        if (EngineMixPercent[i] > 1) 
        {
        	EngineMixPercent[i] -= EngineMixPercent[i] - 1;
        }

        // Keep motor values in interval [0,1]
		EngineMixPercent[i] = FMath::Clamp<float>(EngineMixPercent[i], 0, 1);
    }
	
	
}


void UQuadcopterFlightModel::SetEnginesFromMixer(void)
{
	for (int i = 0; i < 4; i++)
	{
		EngineSpeed[i] = EngineMixPercent[i];
	}
	

}








/* --- Engine Related Stuff ---*/


void UQuadcopterFlightModel::SetEnginePercent(int engineNumber, float inValue)
{
	EngineSpeed[engineNumber] = inValue;
	EngineSpeed[engineNumber] = FMath::Clamp<float>(EngineSpeed[engineNumber], 0.0f, 1.0f);
}


void UQuadcopterFlightModel::SetEngineRPM(int engineNumber, float inValue)
{
	EngineSpeed[engineNumber] = inValue / EngineMaxRPM;
	EngineSpeed[engineNumber] = FMath::Clamp<float>(EngineSpeed[engineNumber], 0.0f, 1.0f);
}



void UQuadcopterFlightModel::GetEngineForces()
{


	// Calculate Thrust from all Engines
	float sum = 0;
	for (int i = 0; i<4; i++)
		sum += FMath::Pow(EngineSpeed[i], Engine_Q);
	TotalThrust = FVector(0.0f, 0.0f, Engine_K * sum);



	// ODO: Verallgemeinern!!!
	// Und in die Propertries
	// Genauso: L (ArmLength: Array mit Wert pro Engine. Und in die Properties
	float EngineAlpha[4];

	if (FrameMode == EFrameMode::FrameModeCross)
	{
		EngineAlpha[0] = FMath::DegreesToRadians<float>(45);
		EngineAlpha[1] = FMath::DegreesToRadians<float>(45);
		EngineAlpha[2] = FMath::DegreesToRadians<float>(45);
		EngineAlpha[3] = FMath::DegreesToRadians<float>(45);
	}
	else if (FrameMode == EFrameMode::FrameModePlus)
	{
		EngineAlpha[0] = FMath::DegreesToRadians<float>(0);
		EngineAlpha[1] = FMath::DegreesToRadians<float>(45);
		EngineAlpha[2] = FMath::DegreesToRadians<float>(0);
		EngineAlpha[3] = FMath::DegreesToRadians<float>(45);
	}

	

	
	// Calculate Torque from all Engines
	
	float SpeedToThrust[4];
	float SpeedToTorque[4];
	for (int i = 0; i<4; i++)
	{
		SpeedToThrust[i] = FMath::Pow(EngineSpeed[i], Engine_Q);
		SpeedToTorque[i] = FMath::Pow(EngineSpeed[i], Engine_QQ);
	}
	
	
	FVector EngineTorque = FVector(0.0f, 0.0f, 0.0f);

	if (FrameMode == EFrameMode::FrameModeCross)
	{
		for (int i = 0; i < 4; i++)
		{
			EngineTorque.X += MixerQuadCross[i].Roll * SpeedToThrust[i] * sinf(EngineAlpha[i]) * Engine_L * Engine_K;
			EngineTorque.Y += MixerQuadCross[i].Pitch * SpeedToThrust[i] * cosf(EngineAlpha[i]) * Engine_L * Engine_K;
			EngineTorque.Z += MixerQuadCross[i].Yaw * SpeedToTorque[i] * Engine_B;
		}

	}
	else if (FrameMode == EFrameMode::FrameModePlus)
	{
		for (int i = 0; i < 4; i++)
		{
			EngineTorque.X += MixerQuadPlus[i].Roll * SpeedToThrust[i] * sinf(EngineAlpha[i]) * Engine_L * Engine_K;
			EngineTorque.Y += MixerQuadPlus[i].Pitch * SpeedToThrust[i] * cosf(EngineAlpha[i]) * Engine_L * Engine_K;
			EngineTorque.Z += MixerQuadPlus[i].Yaw * SpeedToTorque[i] * Engine_B;
		}

	}
	TotalTorque = EngineTorque;

	
}


/*
void UQuadcopterFlightModel::GetEngineForces()
{
	// Get new Engine Settings from Mixer
	
	for (int i=0; i<4; i++)
		EngineSpeed[i] = EngineMixPercent[i];
	
	// Calculate Thrust from all Engines
	float sum = 0;
	for (int i = 0; i<4; i++)
		sum += FMath::Pow(EngineSpeed[i], Engine_Q);
	TotalThrust = FVector(0.0f, 0.0f, Engine_K * sum);
	
	// Calculate Torque from all Engines
	if (FrameMode == EFrameMode::FrameModeCross)
	{
		TotalTorque = GetTorquesCross4();
	}
	else if (FrameMode== EFrameMode::FrameModePlus)
	{
		TotalTorque = GetTorquesPlus4();
	}

}


FVector UQuadcopterFlightModel::GetTorquesPlus4()
{
	float SpeedToThrust[4];
	float SpeedToTorque[4];
	for (int i = 0; i<4; i++)
	{
		SpeedToThrust[i] = FMath::Pow(EngineSpeed[i], Engine_Q);
		SpeedToTorque[i] = FMath::Pow(EngineSpeed[i], Engine_QQ);
	}
	FVector EngineTorque = FVector(
		Engine_L * Engine_K * (SpeedToThrust[2] - SpeedToThrust[0]),
		Engine_L * Engine_K * (SpeedToThrust[3] - SpeedToThrust[1]),
		Engine_B * (SpeedToTorque[0] - SpeedToTorque[1] + SpeedToTorque[2] - SpeedToTorque[3])
	);
	return EngineTorque;
	// rem: front = 0, then clockwise
}

FVector UQuadcopterFlightModel::GetTorquesCross4()
{
	float SpeedToThrust[4];
	float SpeedToTorque[4];
	for (int i = 0; i<4; i++)
	{
		SpeedToThrust[i] = FMath::Pow(EngineSpeed[i], Engine_Q);
		SpeedToTorque[i] = FMath::Pow(EngineSpeed[i], Engine_QQ);
	}
	FVector EngineTorque = FVector(		
		Engine_L / FPlatformMath::Sqrt(2) * Engine_K * (SpeedToThrust[3] + SpeedToThrust[2] - SpeedToThrust[0] - SpeedToThrust[1]),
		Engine_L / FPlatformMath::Sqrt(2) * Engine_K * (SpeedToThrust[0] + SpeedToThrust[3] - SpeedToThrust[1] - SpeedToThrust[2]),
		Engine_B * (SpeedToTorque[0] - SpeedToTorque[1] + SpeedToTorque[2] - SpeedToTorque[3])
	);
	return EngineTorque;
	// rem: front right = 0, then clockwise
}


*/


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

