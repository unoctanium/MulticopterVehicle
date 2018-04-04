
#pragma once

#include "CoreMinimal.h"

#include "Components/SceneComponent.h"
#include "Components/PrimitiveComponent.h"
#include "PhysicsEngine/BodyInstance.h"

#include "QFMAHRS.h"
#include "QFMEngineController.h"
#include "QFMVehicle.h"

#include "QFMPositionController.generated.h"



/*--- Implementatrion of the Position-Controller ---*/
USTRUCT(BlueprintType)
struct FPositionController
{
	GENERATED_BODY()


	/*--- PARAMETERS ---*/
	UPROPERTY() float MaxClimbVelocityZ = 0.0f;
	UPROPERTY() float MaxDescentVelocityZ = 0.0f;
	UPROPERTY() float MaxAccelerationZ = 0.0f;
	UPROPERTY() float PosTargetZ = 0.0f;
	UPROPERTY() bool bIsActiveZ = true;
	

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

		bIsActiveZ = true;

	}


	void Reset()
	{
		bIsActiveZ = true;
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
	}


	void SetAltTargetToCurrentAlt()
	{
		//pos_control->set_alt_target_to_current_alt(); 
		PosTargetZ = AHRS->GetWorldAltitude();
	}
	

	void SetAltTargetFromClimbRate(float TargetClimbRate)
	{
		//set_alt_target_from_climb_rate_ff 
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
        if (VelocityTargetZ < MaxDescentVelocityZ) 
		{
        	VelocityTargetZ = MaxDescentVelocityZ;
   		}
    	if (VelocityTargetZ > MaxClimbVelocityZ) 
		{
        	VelocityTargetZ = MaxClimbVelocityZ;
        }

    	// the following section calculates acceleration required to achieve the velocity target

    	float VelocityCurrentZ = AHRS->GetWorldVelocity().Z;

		AccelerationTargetZ = (VelocityTargetZ - VelocityCurrentZ) / DeltaTime;
		float AccelerationCurrentZ = AHRS->GetWorldAccelerationXYZ().Z;// + Vehicle->GetGravity();
		

		// First draft: Without any Loop Controllers.

		// ... LoopController would be called here

		//

		float AccelerationToApplyZ = AccelerationTargetZ - AccelerationCurrentZ;

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




		// the following section calculates a desired throttle needed to achieve the acceleration target
		float ThrottleOut = AccelerationToApplyZ / MaxAccelerationZ;
		ThrottleOut += EngineController->GetThrottleHover();
		ThrottleOut = FMath::Clamp(ThrottleOut, 0.0f, 1.0f);



		UE_LOG(LogTemp,Display,TEXT("ALT: ZA %f\tZT %f\tZE %f"),CurrentAlt, PosTargetZ, PosErrorZ);
		UE_LOG(LogTemp,Display,TEXT("VEL: VA %f\tVT %f"),VelocityCurrentZ, VelocityTargetZ);
		UE_LOG(LogTemp,Display,TEXT("ACC: AA %f\tAT %f\tAE %f"),AccelerationCurrentZ, AccelerationTargetZ, AccelerationToApplyZ);
		UE_LOG(LogTemp,Display,TEXT("THR: TO %f\tAH %f"),ThrottleOut, EngineController->GetThrottleHover());





		EngineController->SetDesiredThrottlePercent(ThrottleOut);

	}




	void Debug(FColor ColorIn, FVector2D DebugFontSizeIn)
	{
		// Down to up on the debug screen
		
	}



};


