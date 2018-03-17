
#pragma once

#include "CoreMinimal.h"

#include "Components/SceneComponent.h"
#include "PhysicsEngine/BodyInstance.h"

#include "QFMAHRS.h"

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
	UPROPERTY() float DesiredVelocityZ = 0.0f;

	UPROPERTY() float AltTarget = 0.0f;
	UPROPERTY() bool bIsActiveZ = true;
	

	/*--- INTERFACE DATA ---*/
	// Copy of Parent Data. Put inside here during Init and Tock 
	float DeltaTime;
	FBodyInstance *bodyInstance;
	UPrimitiveComponent *primitiveComponent;
	FAHRS *AHRS;

	void Init(FBodyInstance *bodyInstanceIn, UPrimitiveComponent *primitiveComponentIn, FAHRS *AHRSIn)
	{
		bodyInstance = bodyInstanceIn;
		primitiveComponent = primitiveComponentIn;
		AHRS = AHRSIn;

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


	
	void Debug(FColor ColorIn, FVector2D DebugFontSizeIn)
	{
		// Down to up on the debug screen
		
	}


	/* --- POS CONTROLLER --- */


	void SetAltTarget(float AltIn)
	{
		AltTarget = AltIn;
	}


	void SetMaxVelocityZ(float SpeedDownIn, float SpeedUpIn)
	{
		//sets maximum climb and descent rates 
		//pos_control->set_speed_z 
		MaxClimbVelocityZ = SpeedUpIn;
		MaxDescentVelocityZ = SpeedDownIn;
	}


	void SetMaxAccelerationZ(float AccelIn)
	{
		//sets maximum climb and descent acceleration 
		//pos_control->set_accel_z 
		MaxAccelerationZ = AccelIn;
	}

	void SetDesiredVelocityZ(float VelocityIn)
	{
		//sets Desired climb/descent rate 
		//pos_control->set_desired_velocity_z 
		DesiredVelocityZ = VelocityIn;
	}



	bool IsActiveZ()
	{
		// pos_control->is_active_z() 
		return bIsActiveZ;
	}

	void SetAltTargetToCurrentAlt()
	{
		//pos_control->set_alt_target_to_current_alt(); 
		AltTarget = AHRS->GetWorldAltitude();
	}

	

	void SetAltTargetFromClimbRate(float TargetClimbRate)
	{
		//set_alt_target_from_climb_rate_ff 
		AltTarget = AHRS->GetWorldAltitude() + TargetClimbRate * DeltaTime;
	}


	void UpdateZController()
	{
		//pos_control->update_z_controller(); 

	}


};


