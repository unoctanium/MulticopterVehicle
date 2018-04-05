
#pragma once

#include "CoreMinimal.h"

#include "Components/SceneComponent.h"
#include "Components/PrimitiveComponent.h"
#include "PhysicsEngine/BodyInstance.h"


#include "QFMTypes.h"
#include "QFMPIDController.h"
#include "QFMAHRS.h"
#include "QFMEngineController.h"
#include "QFMVehicle.h"

#include "QFMPositionController.generated.h"



/*--- Implementation of the Position-Controller ---*/
USTRUCT(BlueprintType)
struct FPositionController
{
	GENERATED_BODY()

	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "QuadcopterFlightModel", meta = (ToolTip = "Activate Position Z Controller"))
	bool bIsActiveZ = true;

	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "QuadcopterFlightModel", meta = (ToolTip = "Max Speed Up in m/s in AltHold Mode")) 
	float MaxClimbVelocityZ = 20.0f;   
		
	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "QuadcopterFlightModel", meta = (ToolTip = "Max Speed Down in m/s in AltHold Mode")) 
	float MaxDescentVelocityZ = 20.0f;   
	
	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "QuadcopterFlightModel", meta = (ToolTip = "Desired Z Accel in Alt Hold Mode in m/s^2")) 
	float MaxAccelerationZ = 9.8f;

	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "QuadcopterFlightModel", meta = (ToolTip = "Stabilizer-Loop to use for Translations")) 
	EControlLoop TranslationControlLoop = EControlLoop::ControlLoop_P;

	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "QuadcopterFlightModel", meta = (ToolTip = "Yaw Rate PID"))
	FVector RateZPidSettings = FVector(0.1f, 0.0f, 0.001f);

	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "QuadcopterFlightModel", meta = (ToolTip = "FPD Damping. 1=crit damped, <1 = underdamped, >1 = overdamped"))
	float SPDDamping = 6.0f;	
	
	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "QuadcopterFlightModel", meta = (ToolTip = "FPD Frequency. Reach 95% of target in 1/Freq secs"))
	float SPDFrequency = 100.0f;



	/*--- PARAMETERS ---*/
	UPROPERTY() float PosTargetZ = 0.0f;
	UPROPERTY() bool bIsLockedZ = false;
	
	// PIDs
	UPROPERTY() FPIDController RateZPid;


	/*--- INTERFACE DATA ---*/
	// Copy of Parent Data. Put inside here during Init and Tock 
	float DeltaTime;
	FBodyInstance *bodyInstance;
	UPrimitiveComponent *primitiveComponent;
	FAHRS *AHRS;
	FEngineController *EngineController;
	FVehicle *Vehicle;

	void Init(FBodyInstance *bodyInstanceIn, UPrimitiveComponent *primitiveComponentIn, FAHRS *AHRSIn, 	FVehicle *VehicleIn, FEngineController *EngineControllerIn)
	{
		bodyInstance = bodyInstanceIn;
		primitiveComponent = primitiveComponentIn;
		AHRS = AHRSIn;
		EngineController = EngineControllerIn;
		Vehicle = VehicleIn;

		//bIsActiveZ = true;
		bIsLockedZ = false;

		// Init Pids with min,max = -1..1. We normalize Rates in RunZController, so we allways have values from 0..1
		RateZPid.Init(-1, 1, RateZPidSettings.X, RateZPidSettings.Y, RateZPidSettings.Z);

	}


	void Reset()
	{
		//bIsActiveZ = true;
		bIsLockedZ = false;

		// ResetPids
		RateZPid.Reset();
	}

	
	void Tock(float DeltaTimeIn)
	{
		DeltaTime = DeltaTimeIn;
	}




	/* --- Check Active ---*/	

	bool IsActiveZ()
	{
		// pos_control->is_active_z() 
		return bIsActiveZ;
	}



	/* --- Set Alt Targets ---*/

	void SetAltTarget(float AltIn)
	{
		PosTargetZ = AltIn;
		bIsLockedZ = true;
	}


	void SetAltTargetToCurrentAlt()
	{
		//pos_control->set_alt_target_to_current_alt(); 
		PosTargetZ = AHRS->GetWorldAltitude();
		bIsLockedZ = true;
	}
	

	void SetAltTargetFromClimbRate(float TargetClimbRate)
	{
		//set_alt_target_from_climb_rate_ff 
		TargetClimbRate = FMath::Clamp<float>(TargetClimbRate, -MaxDescentVelocityZ, MaxClimbVelocityZ);

		if (FMath::Abs(TargetClimbRate) < 0.001f  ) {
			if(!bIsLockedZ)
			{
				SetAltTargetToCurrentAlt(); // Will lock bIsLockedZ
				return;
			}
			else
			{
				return;
			}
		}
		bIsLockedZ = false;
		PosTargetZ = AHRS->GetWorldAltitude() + TargetClimbRate * DeltaTime;
	}


	/* Set MinMax Values ---*/

	void SetMaxVelocityZ(float SpeedDownIn, float SpeedUpIn)
	{
		//sets maximum climb and descent rates 
		MaxClimbVelocityZ = SpeedUpIn;
		MaxDescentVelocityZ = SpeedDownIn;
	}


	void SetMaxAccelerationZ(float AccelIn)
	{
		//sets maximum climb and descent acceleration 
		MaxAccelerationZ = AccelIn;
	}



	/* --- Update Loop ---*/


	void UpdateZController()
	{

		float PosErrorZ = 0.0f;
		float VelocityTargetZ = 0.0f;
		float AccelerationTargetZ = 0.0f;

		// Get current altitude in m
		float CurrentAlt = AHRS->GetWorldAltitude();

		// get position error in m
    	PosErrorZ = PosTargetZ - CurrentAlt;

		// calculate Velocity Target for actual position based on PosError using Linear Function
    	VelocityTargetZ = PosErrorZ / DeltaTime;

    	// check speed limits
        if (VelocityTargetZ > MaxClimbVelocityZ) 
		{
        	VelocityTargetZ = MaxClimbVelocityZ;
   		}
    	if (VelocityTargetZ < -MaxDescentVelocityZ) 
		{
        	VelocityTargetZ = -MaxDescentVelocityZ;
        }

    	// the following section calculates acceleration required to achieve the velocity target

    	float VelocityCurrentZ = AHRS->GetWorldVelocity().Z;

		AccelerationTargetZ = (VelocityTargetZ - VelocityCurrentZ); // / DeltaTime;
		float AccelerationCurrentZ = AHRS->GetWorldAccelerationXYZ().Z;// + Vehicle->GetGravity();
		

		// First draft: Without any Loop Controllers.

		// ... LoopController would be called here

		//

		//float AccelerationToApplyZ = AccelerationTargetZ;

/*
		// check accel limits
		if (AccelerationToApplyZ > 0)
		{
			if (AccelerationToApplyZ > MaxAccelerationZ)
			{
				AccelerationToApplyZ = MaxAccelerationZ;
			}
		}
		else if (AccelerationToApplyZ < 0) 
		{
			if (AccelerationToApplyZ < -MaxAccelerationZ)
			{
				AccelerationToApplyZ = -MaxAccelerationZ;
			}
		}
*/

		// the following section calculates a desired throttle needed to achieve the acceleration target
		// Normalize Accel Request
				
		//Trying Workaround...
		// Works with Stable PD 100/10
		//AccelerationTargetZ = VelocityTargetZ / MaxClimbVelocityZ * MaxAccelerationZ;
		//AccelerationCurrentZ = VelocityCurrentZ / MaxClimbVelocityZ * MaxAccelerationZ;
		

		float ThrottleOut = 0.0f;

		if(TranslationControlLoop == EControlLoop::ControlLoop_P)
		{
			// P-Controller
			ThrottleOut = (AccelerationTargetZ - AccelerationCurrentZ) / MaxAccelerationZ;;
			//ThrottleOut += EngineController->GetThrottleHover();
		}
		else if (TranslationControlLoop == EControlLoop::ControlLoop_PID)
		{
			// PID Controller
			ThrottleOut = RateZPid.Calculate(AccelerationTargetZ / MaxAccelerationZ, AccelerationCurrentZ / MaxAccelerationZ, DeltaTime);
			ThrottleOut += EngineController->GetThrottleHover();
		}
		else if (TranslationControlLoop == EControlLoop::ControlLoop_SPD)
		{
			// FPD-Controller 
    	    ThrottleOut = StepAccelZSpd(AccelerationTargetZ / MaxAccelerationZ,  AccelerationCurrentZ / MaxAccelerationZ);
			//ThrottleOut += EngineController->GetThrottleHover();
		}


		// ThrottleOut += EngineController->GetThrottleHover();

		// Sanity Check
		ThrottleOut = FMath::Clamp(ThrottleOut, 0.0f, 1.0f);


/*
		UE_LOG(LogTemp,Display,TEXT("ALT: ZA %f\tZT %f\tZE %f\t\t %d"),CurrentAlt, PosTargetZ, PosErrorZ, bIsLockedZ);
		UE_LOG(LogTemp,Display,TEXT("VEL: VA %f\tVT %f"),VelocityCurrentZ, VelocityTargetZ);
		UE_LOG(LogTemp,Display,TEXT("ACC: AA %f\tAT %f"),AccelerationCurrentZ, AccelerationTargetZ);
		UE_LOG(LogTemp,Display,TEXT("THR: TO %f\t"),ThrottleOut);
*/

		EngineController->SetDesiredThrottlePercent(ThrottleOut);
	}


	// Run the TZranslational Z Acceleration FPD controller and return the output detla w -1..1
	float StepAccelZSpd(float Target, float Current)
	{
		float kp = SPDFrequency * SPDFrequency * 9.0f; 
		float kd = 4.5f * SPDFrequency * SPDDamping; 
		float dt = DeltaTime; 
		
		float g = 1.0f / (1.0f + kd * dt + kp * dt * dt); 
		float kpg = kp * g; 
		float kdg = (kd + kp * dt) * g; 

		return (kpg * Target - kdg * Current);
	}




	void Debug(FColor ColorIn, FVector2D DebugFontSizeIn)
	{
		// Down to up on the debug screen
		
	}



};


