
#pragma once

#include "CoreMinimal.h"

#include "Components/SceneComponent.h"
#include "Components/PrimitiveComponent.h"
#include "PhysicsEngine/BodyInstance.h"

#include "GameFramework/PhysicsVolume.h" // For GetGravityZ
#include "PhysXIncludes.h"
#include "PhysicsPublic.h"
#include "PhysXPublic.h"
#include "Classes/PhysicsEngine/PhysicsSettings.h"
#include "Runtime/Engine/Private/PhysicsEngine/PhysXSupport.h"

#include "QFMTypes.h"

#include "QFMVehicle.generated.h"

USTRUCT(BlueprintType)
struct FVehicle
{
	GENERATED_BODY()

	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "QuadcopterFlightModel|Vehicle", meta = (ToolTip = "Frame Mode")) 
	EFrameMode FrameMode = EFrameMode::FrameModeCross;
	
	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "QuadcopterFlightModel|Vehicle|CalculateSetting", meta = (ToolTip = "Calculate bodies mass and inertia")) 
	bool CalculateMassProperties = false;
	
	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "QuadcopterFlightModel|Vehicle|CalculateSetting", meta = (ToolTip = "Central mass (kg)")) 
	float CentralMass = 10.0f;
	
	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "QuadcopterFlightModel|Vehicle|CalculateSetting", meta = (ToolTip = "Central mass radius (m)")) 
	float CentralRadius = 0.25f;
	
	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "QuadcopterFlightModel|Vehicle|CalculateSetting", meta = (ToolTip = "NumberOfEngines")) 
	float NumberOfEngines = 4.0;
	
	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "QuadcopterFlightModel|Vehicle|CalculateSetting", meta = (ToolTip = "Motor mass (kg), one of 4 motors")) 
	float MotorMass = 5.0f;
	
	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "QuadcopterFlightModel|Vehicle|CalculateSetting", meta = (ToolTip = "Arm length of each arm (m)")) 
	float ArmLength = 0.5f;
	
	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "QuadcopterFlightModel|Vehicle|ManualSetting", meta = (ToolTip = "Manually enter bodies mass and inertia")) 
	bool EnterMassProperties = true;
	
	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "QuadcopterFlightModel|Vehicle|ManualSetting", meta = (ToolTip = "Mass in KG")) 
	float Mass = 30.0f;
	
	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "QuadcopterFlightModel|Vehicle|ManualSetting", meta = (ToolTip = "Inertia Tensor in kg * cm^2")) 
	FVector InertiaTensor = FVector(200000.0f, 200000.0f, 400000.0f);
	
	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "QuadcopterFlightModel|Vehicle|ManualSetting", meta = (ToolTip = "Center of mass in m")) 
	FVector CenterOfMass = FVector(0.0f, 0.0f, 0.0f);


	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "QuadcopterFlightModel|Vehicle|Gravity", meta = (ToolTip = "Gravity")) 
	float Gravity = 9.81f;
    
	

	/*--- INTERFACE DATA ---*/
	// Copy of Parent Data. Put inside here during Init and Tock 
	UPROPERTY() 
	float DeltaTime;
	FBodyInstance *BodyInstance;
	UPrimitiveComponent *PrimitiveComponent;


	void Init(FBodyInstance *BodyInstanceIn, UPrimitiveComponent *PrimitiveComponentIn)
	{
		BodyInstance = BodyInstanceIn;
		PrimitiveComponent = PrimitiveComponentIn;


		// Get Gravity
		Gravity = PrimitiveComponent->GetPhysicsVolume()->GetGravityZ() / 100.0f;

		// Find My Px Rigid Body
		physx::PxRigidBody* PRigidBody = BodyInstance->GetPxRigidBody_AssumesLocked();

		//	#if WITH_PHYSX
		//	ScreenMsg("Got into PhysX!!!");
		//  #endif
		
		// Mass, COM and and Inertia overrides
		if (CalculateMassProperties)
		{
			// Calculate new Mass and Inertia Tensor based on Vehicle Properties
			float facCentralMass = 2.0f / 5.0f * CentralMass * CentralRadius * CentralRadius;
			InertiaTensor.X = facCentralMass + 2 * ArmLength * ArmLength * MotorMass * 10000.0f;
			InertiaTensor.Y = facCentralMass + 2 * ArmLength * ArmLength * MotorMass * 10000.0f;
			InertiaTensor.Z = facCentralMass + 4 * ArmLength * ArmLength * MotorMass * 10000.0f;
			Mass = CentralMass + 4 * MotorMass;
			CenterOfMass = FVector(0.0f, 0.0f, 0.0f);
		}
		if (EnterMassProperties || CalculateMassProperties)
		{
			// Update UE4 Mass Properies based on vehicle Properties (Mass and Inertia Tensor)
			
			// PhyX Mass Override
			BodyInstance->SetMassOverride(Mass, true);
			BodyInstance->UpdateMassProperties();

			// PhyX Inertia Tensor Override
			PRigidBody->setMassSpaceInertiaTensor(physx::PxVec3(InertiaTensor.X, InertiaTensor.Y, InertiaTensor.Z));

			// Set Center Of Mass
			//PrimitiveComponent->SetCenterOfMass(BodyInstance->GetCOMPosition() - BodyInstance->GetCOMPosition());
			//FTransform transform = PrimitiveComponent->GetComponentTransform();
			//BodyInstance->COMNudge = transform.GetTranslation() - BodyInstance->GetCOMPosition() + CenterOfMass * 100;
			//PRigidBody->setCMassLocalPose(physx::PxTransform(physx::PxVec3(CenterOfMass.X, CenterOfMass.Y, CenterOfMass.Z)));

			//BodyInstance->UpdateMassProperties();
			
		}
		// Store Mass Properties in Vehicle Struct
		{
			Mass = BodyInstance->GetBodyMass();
			InertiaTensor = BodyInstance->GetBodyInertiaTensor();
			FTransform transform = PrimitiveComponent->GetComponentTransform();
			CenterOfMass = (BodyInstance->GetCOMPosition() - transform.GetTranslation()) / 100.0f; // To set it in m
		}

	}

	void Tock(float DeltaTimeIn)
	{
		DeltaTime = DeltaTimeIn;
	}


	void Debug(FColor ColorIn, FVector2D DebugFontSizeIn)
	{
		GEngine->AddOnScreenDebugMessage(-1, 0, ColorIn, FString::Printf(TEXT("Center of Mass (m): X=%f Y=%f Z=%f"), CenterOfMass.X, CenterOfMass.Y, CenterOfMass.Z), true, DebugFontSizeIn);
		GEngine->AddOnScreenDebugMessage(-1, 0, ColorIn, FString::Printf(TEXT("Moment of intertia (kg*m^2): X=%f Y=%f Z=%f"), InertiaTensor.X / 10000.0f, InertiaTensor.Y / 10000.0f, InertiaTensor.Z / 10000.0f), true, DebugFontSizeIn);
		GEngine->AddOnScreenDebugMessage(-1, 0, ColorIn, FString::Printf(TEXT("Mass (kg): %f"), Mass), true, DebugFontSizeIn);
	}
	


};






