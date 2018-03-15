
#pragma once

#include "CoreMinimal.h"

#include "Components/SceneComponent.h"
#include "PhysicsEngine/BodyInstance.h"



#include "QFMPositionController.generated.h"



/*--- Implementatrion of the Position-Controller ---*/
USTRUCT(BlueprintType)
struct FPositionController
{
	GENERATED_BODY()


	/*--- PARAMETERS ---*/


	/*--- INTERFACE DATA ---*/
	// Copy of Parent Data. Put inside here during Init and Tock 
	float DeltaTime;
	FBodyInstance *bodyInstance;
	UPrimitiveComponent *primitiveComponent;
	

	void Init(FBodyInstance *bodyInstanceIn, UPrimitiveComponent *primitiveComponentIn)
	{
		bodyInstance = bodyInstanceIn;
		primitiveComponent = primitiveComponentIn;

	}


	void Reset()
	{

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
		//pos_control->set_alt_Target(f) 
	}


	void SetVelocityZ(float SpeedDownIn, float SpeedUpIn)
	{
		//sets maximum climb and descent rates 
		//pos_control->set_speed_z 
	}

	void SetAccelZ(float AccelIn)
	{
		//sets maximum climb and descent acceleration 
		//pos_control->set_accel_z 
	}

	bool IsActiveZ()
	{
		// pos_control->is_active_z() 
		return true;
	}

	void SetAltTargetToCurrentAlt()
	{
		//pos_control->set_alt_target_to_current_alt(); 
	}

	void SetDesiredVelocityZ(float VelocityIn)
	{
		//sets Desired climb/descent rate 
		//pos_control->set_desired_velocity_z 
	}

	void SetAltTargetFromClimbRate(float TargetClimbRate)
	{
		//set_alt_target_from_climb_rate_ff 
	}


	void UpdateZController()
	{
		//pos_control->update_z_controller(); 
	}


};


