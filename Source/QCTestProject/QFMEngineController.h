#pragma once

#include "CoreMinimal.h"

#include "Components/SceneComponent.h"
#include "Components/PrimitiveComponent.h"
#include "PhysicsEngine/BodyInstance.h"

#include "QFMTypes.h"
#include "QFMVehicle.h" // I must read Properties like FrameType and Gravity 


#include "QFMEngineController.generated.h"


// https://quadcopterproject.wordpress.com/static-thrust-calculation/
// => m = K n w^f / g : ThrustMass
// Watch: https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_Motors/AP_MotorsMatrix.h
// https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_Motors/AP_MotorsMatrix.cpp
// https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_Motors/AP_MotorsMulticopter.h
// https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_Motors/AP_MotorsMulticopter.cpp
// https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_Motors/AP_Motors_Class.h
// https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_Motors/AP_Motors_Class.cpp


// Roll right = +
// pitch up = +
// yaw right = +

USTRUCT()
struct FQuadcopterFlightModelMixerStruct
{
	GENERATED_BODY()

	UPROPERTY() float Throttle = 0.0f;
	UPROPERTY() float Roll = 0.0f;
	UPROPERTY() float Pitch = 0.0f;
	UPROPERTY() float Yaw = 0.0f;

	// constructor that takes parameters
	FQuadcopterFlightModelMixerStruct(float InThrottle, float InRoll, float InPitch, float InYaw)
		: Throttle(InThrottle), Roll(InRoll), Pitch(InPitch), Yaw(InYaw)
	{};

	// constructor that takes no parameters
	FQuadcopterFlightModelMixerStruct()
	{};
	
};




/*--- Implementation of the EngineController ---*/
USTRUCT(BlueprintType)
struct FEngineController
{
	GENERATED_BODY()

	UPROPERTY() 
	float RollRequest; // -1..1

	UPROPERTY() 
	float PitchRequest; // -1..1

	UPROPERTY() 
	float YawRequest; // -1..1

	UPROPERTY() 
	float ThrottleRequest; // 0..1



	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "QuadcopterEngineSettings", meta = (ToolTip = "Engine Max RPM")) 
	float EngineMaxRPM = 1000.0f;
	
	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "QuadcopterEngineSettings", meta = (ToolTip = "Engine Thrust Coefficient K")) 
	float Engine_K = 147.0; // 294 
	
	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "QuadcopterEngineSettings", meta = (ToolTip = "Engine Thrust Exponent")) 
	float Engine_Q = 2.0f;
	
	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "QuadcopterEngineSettings", meta = (ToolTip = "Engine Torque Yaw Coefficient")) 
	float Engine_B = 25.0f;
	
	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "QuadcopterEngineSettings", meta = (ToolTip = "Engine Torque Exponent")) 
	float Engine_QQ = 2.0f;
	
	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "QuadcopterEngineSettings", meta = (ToolTip = "Calculate Engine_K with Thrust To lift PlanMaxLift * weight")) 
	bool CalculateEngine_K = true;

	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "QuadcopterEngineSettings", meta = (ToolTip = "Calculate Engine_K with Thrust To lift PlanMaxLift * weight")) 
	float PlanMaxLift = 2.0f;
	
	
	// Mixer
	UPROPERTY() 
	FQuadcopterFlightModelMixerStruct MixerQuadCross[4] = {
		{ +1, -1,  -1, -1 }, // RF-
		{ +1, -1,  +1, +1 }, // RB+
		{ +1, +1,  +1, -1 }, // LB-
		{ +1, +1,  -1, +1 } // LF+
	};
		
	UPROPERTY()
	FQuadcopterFlightModelMixerStruct MixerQuadPlus[4] = {
		{ +1, 0,  -1, -1 }, // F-
		{ +1, -1,  +1, +1 }, // R+
		{ +1, 0,  +1, -1 }, // B-
		{ +1, +1,  -1, +1 } // L+
	};
	
	UPROPERTY()
	float EngineMixPercent[4] = {0.0f, 0.0f, 0.0f, 0.0f};
	
	UPROPERTY()
	float EngineSpeed[4] = {0.0f, 0.0f, 0.0f, 0.0f}; // 0..1

	UPROPERTY()
	FVector TotalThrust = FVector(0.0f, 0.0f, 0.0f);

	UPROPERTY()
	FVector TotalTorque = FVector(0.0f, 0.0f, 0.0f);;



	/*--- INTERFACE DATA ---*/
	// Copy of Parent Data. Put inside here during Init and Tock 
	float DeltaTime;
	FBodyInstance *BodyInstance;
	UPrimitiveComponent *PrimitiveComponent;
	FVehicle *Vehicle;



	void Init(FBodyInstance *BodyInstanceIn, UPrimitiveComponent *PrimitiveComponentIn, FVehicle * VehicleIn)
	{
		BodyInstance = BodyInstanceIn;
		PrimitiveComponent = PrimitiveComponentIn;
		Vehicle = VehicleIn;
		if(CalculateEngine_K)
		{
			Engine_K = (PlanMaxLift * Vehicle->Mass * -Vehicle->Gravity) / ( Vehicle->NumberOfEngines * FMath::Pow(1.0f, Engine_Q));
		}
	}

	
	void Reset()
	{

	}

	
	void Tock(float DeltaTimeIn)
	{
		DeltaTime = DeltaTimeIn;
		
		// Mix Pilot Input to meet Frame Mode 
		MixEngines();
	
		// Apply Engine RPM ... Now: Only Mixed but no Attitude Controller
		SetEnginesFromMixer();

		// Calculate new Forces
		GetEngineForces();

		//PrimitiveComponent->SetPhysicsAngularVelocityInRadians(TotalTorque, true, NAME_None);

	}

	

	void SetEnginePercent(int engineNumber, float inValue)
	{
		EngineSpeed[engineNumber] = inValue;
		EngineSpeed[engineNumber] = FMath::Clamp<float>(EngineSpeed[engineNumber], 0.0f, 1.0f);
	}


	void SetEngineRPM(int engineNumber, float inValue)
	{
		EngineSpeed[engineNumber] = inValue / EngineMaxRPM;
		EngineSpeed[engineNumber] = FMath::Clamp<float>(EngineSpeed[engineNumber], 0.0f, 1.0f);
	}



	void GetEngineForces()
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

		if (Vehicle->FrameMode == EFrameMode::FrameModeCross)
		{
			EngineAlpha[0] = FMath::DegreesToRadians<float>(45);
			EngineAlpha[1] = FMath::DegreesToRadians<float>(45);
			EngineAlpha[2] = FMath::DegreesToRadians<float>(45);
			EngineAlpha[3] = FMath::DegreesToRadians<float>(45);
		}
		else if (Vehicle->FrameMode == EFrameMode::FrameModePlus)
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

		if (Vehicle->FrameMode == EFrameMode::FrameModeCross)
		{
			for (int i = 0; i < 4; i++)
			{
				EngineTorque.X += MixerQuadCross[i].Roll * SpeedToThrust[i] * sinf(EngineAlpha[i]) * Vehicle->ArmLength * Engine_K / 2000.0f;
				EngineTorque.Y += MixerQuadCross[i].Pitch * SpeedToThrust[i] * cosf(EngineAlpha[i]) * Vehicle->ArmLength * Engine_K / 2000.0f;
				EngineTorque.Z += MixerQuadCross[i].Yaw * SpeedToTorque[i] * Engine_B / 2000.0f;
			}

		}
		else if (Vehicle->FrameMode == EFrameMode::FrameModePlus)
		{
			for (int i = 0; i < 4; i++)
			{
				EngineTorque.X += MixerQuadPlus[i].Roll * SpeedToThrust[i] * sinf(EngineAlpha[i]) * Vehicle->ArmLength * Engine_K / 2000.0f;
				EngineTorque.Y += MixerQuadPlus[i].Pitch * SpeedToThrust[i] * cosf(EngineAlpha[i]) * Vehicle->ArmLength * Engine_K / 2000.0f;
				EngineTorque.Z += MixerQuadPlus[i].Yaw * SpeedToTorque[i] * Engine_B / 2000.0f
				;
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
			Vehicle->ArmLength * Engine_K * (SpeedToThrust[2] - SpeedToThrust[0]),
			Vehicle->ArmLength * Engine_K * (SpeedToThrust[3] - SpeedToThrust[1]),
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
			Vehicle->ArmLength / FPlatformMath::Sqrt(2) * Engine_K * (SpeedToThrust[3] + SpeedToThrust[2] - SpeedToThrust[0] - SpeedToThrust[1]),
			Vehicle->ArmLength / FPlatformMath::Sqrt(2) * Engine_K * (SpeedToThrust[0] + SpeedToThrust[3] - SpeedToThrust[1] - SpeedToThrust[2]),
			Engine_B * (SpeedToTorque[0] - SpeedToTorque[1] + SpeedToTorque[2] - SpeedToTorque[3])
		);
		return EngineTorque;
		// rem: front right = 0, then clockwise
	}


	*/






	




	// Return Hover Throttle in range 0..1
	// ODO: This comes from calculation ??? or a parameter ???
	// MUST!! be in ]0..1]
	float GetThrottleHover()
	{
		return FMath::Pow( (Vehicle->Mass * -Vehicle->Gravity) / (Vehicle->NumberOfEngines * Engine_K) , (1.0f / Engine_Q));     
	}


	//float SetThrottleOut(float ThrottleIn, bool ResetAttitudeController = true)
	void SetThrottleOut(float ThrottleIn)
	{
		ThrottleRequest = ThrottleIn;
	}


	// Set Roll -1..1
	void SetRollRate(float ValueIn)
	{
		RollRequest = ValueIn;
	}


	// Set Pitch -1..1
	void SetPitchRate(float ValueIn)
	{
		PitchRequest = ValueIn;
	}


	// Set Yaw -1..1
	void SetYawRate(float ValueIn)
	{
		YawRequest = ValueIn;
	}


	bool IsLimitRollPitch()
	{
		return false;
	}



	/* --- RETURN ENGINE DATA --- */

	float GetEnginePercent(int engineNumber) 
	{ 
		return EngineSpeed[engineNumber]; 
	}
	
	
	float GetEngineRPM(int engineNumber) 
	{ 
		return EngineSpeed[engineNumber] * EngineMaxRPM; 
	}
	
	
	FVector GetTotalThrust() 
	{	
		return TotalThrust; 
	}
	
	
	FVector GetTotalTorque() 
	{	
		return TotalTorque; 
	}



	/* --- MIXER --- */

	void MixEngines() 
	{

		FVector4 DesiredPilotInput = FVector4(RollRequest, PitchRequest, YawRequest, ThrottleRequest );

		if (Vehicle->FrameMode == EFrameMode::FrameModeCross)
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
		else if (Vehicle->FrameMode == EFrameMode::FrameModePlus)
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


	void SetEnginesFromMixer(void)
	{
		for (int i = 0; i < 4; i++)
		{
			EngineSpeed[i] = EngineMixPercent[i];
		}
		
	}


	// Update Throttle Mix to stay in controllable range
	void UpdateThrottleRPYMix()
	{

	}


	void Debug(FColor ColorIn, FVector2D DebugFontSizeIn)
	{
		GEngine->AddOnScreenDebugMessage(-1, 0, ColorIn, FString::Printf(TEXT("Mixer %%  : 1=%f 2=%f 3=%f 4=%f"), EngineMixPercent[0], EngineMixPercent[1], EngineMixPercent[2], EngineMixPercent[3]), true, DebugFontSizeIn);

		GEngine->AddOnScreenDebugMessage(-1, 0, ColorIn, FString::Printf(TEXT("Engines %%  : 1=%f 2=%f 3=%f 4=%f"), GetEnginePercent(0), GetEnginePercent(1), GetEnginePercent(2), GetEnginePercent(3)), true, DebugFontSizeIn);
		GEngine->AddOnScreenDebugMessage(-1, 0, ColorIn, FString::Printf(TEXT("Engines RPM: 1=%f 2=%f 3=%f 4=%f"), GetEngineRPM(0), GetEngineRPM(1), GetEngineRPM(2), GetEngineRPM(3)), true, DebugFontSizeIn);
		GEngine->AddOnScreenDebugMessage(-1, 0, ColorIn, FString::Printf(TEXT("Thrust / Torque: %s / %s"), *TotalThrust.ToString(), *TotalTorque.ToString()), true, DebugFontSizeIn);
		

	}





};



