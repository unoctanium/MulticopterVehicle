#pragma once

#include "CoreMinimal.h"

#include "Components/SceneComponent.h"
#include "Components/PrimitiveComponent.h"
#include "PhysicsEngine/BodyInstance.h"

#include "QFMAHRS.generated.h"


USTRUCT(BlueprintType)
struct FAHRS
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadOnly, meta = (ToolTip = "Position in m")) FVector Position = FVector(0.0f, 0.0f, 0.0f);
	UPROPERTY(EditAnywhere, BlueprintReadOnly, meta = (ToolTip = "Rotation in deg")) FRotator Rotation = FRotator(0.0f, 0.0f, 0.0f);
	UPROPERTY(EditAnywhere, BlueprintReadOnly, meta = (ToolTip = "LinearVelocity in m/s")) float LinearVelocity = 0.0f;
	UPROPERTY(EditAnywhere, BlueprintReadOnly, meta = (ToolTip = "Velocity Vector in m/s")) FVector VelocityVector = FVector(0.0f, 0.0f, 0.0f);
	UPROPERTY(EditAnywhere, BlueprintReadOnly, meta = (ToolTip = "LinearVelocity over Ground in m/s")) float LinearVelocity2D = 0.0f;
	UPROPERTY(EditAnywhere, BlueprintReadOnly, meta = (ToolTip = "LinearVelocity Forward in m/s")) float LinearVelocityX = 0.0f;
	UPROPERTY(EditAnywhere, BlueprintReadOnly, meta = (ToolTip = "Angular Velocity in deg/s")) FVector AngularVelocity = FVector(0.0f, 0.0f, 0.0f);
	UPROPERTY(EditAnywhere, BlueprintReadOnly, meta = (ToolTip = "Linear Acceleration in m/s^2")) float LinearAcceleration = 0.0f;
	UPROPERTY(EditAnywhere, BlueprintReadOnly, meta = (ToolTip = "Angular Acceleration in deg/s^2")) FVector AngularAcceleration = FVector(0.0f, 0.0f, 0.0f);

	// ODO: Need these for FlightController
	// Overthink old values above...
    
    UPROPERTY() FRotator BodyRotation; // NOT!! World Rotation
	


// NEW

	UPROPERTY() FQuat WorldRotationQuat; 
	UPROPERTY() FVector WorldTranslationVect;
	UPROPERTY() FVector BodyAngularVelocityVect;
    


	FBodyInstance *BodyInstance;
	UPrimitiveComponent *PrimitiveComponent;


	void Init(FBodyInstance *BodyInstanceIn, UPrimitiveComponent *PrimitiveComponentIn)
	{
		// Need these Vectors to read data from those components
		BodyInstance = BodyInstanceIn;
		PrimitiveComponent = PrimitiveComponentIn;
	}


	void Tock(float DeltaTime)
	{
		FTransform bodyTransform = PrimitiveComponent->GetComponentTransform();
		
		Position = bodyTransform.GetTranslation() / 100.0f; // in m
		Rotation = bodyTransform.GetRotation().Rotator(); // in deg
		float OldLinearVelocity = LinearVelocity;
		LinearVelocity = BodyInstance->GetUnrealWorldVelocity().Size() / 100.0f; // TAS in m/s
		VelocityVector = BodyInstance->GetUnrealWorldVelocity() / 100.0f; // in m / s
		LinearVelocity2D = BodyInstance->GetUnrealWorldVelocity().Size2D() / 100.0f; // Speed over ground
		LinearVelocityX = BodyInstance->GetUnrealWorldVelocity().X / 100.0f; // Speed over ground Forward
		FVector OldAngularVelocity = FVector(AngularVelocity);
		AngularVelocity = FMath::RadiansToDegrees(BodyInstance->GetUnrealWorldAngularVelocityInRadians());
		LinearAcceleration = (OldLinearVelocity - LinearVelocity) / DeltaTime;
		AngularAcceleration = (OldAngularVelocity - AngularVelocity) / DeltaTime;

	///NEW
		WorldRotationQuat = bodyTransform.GetRotation(); // in rad
		WorldTranslationVect = bodyTransform.GetTranslation() / 100.0f; // in m	
		BodyAngularVelocityVect = FMath::RadiansToDegrees(BodyInstance->GetUnrealWorldAngularVelocityInRadians()); // in deg/s
	}


	FQuat GetWorldRotationQuat()
	{
		return WorldRotationQuat;
	}

	FVector GetWorldTranslationVect()
	{
		return WorldTranslationVect;
	}

	FVector GetBodyAngularVelocityVect()
	{
		return BodyAngularVelocityVect;
	}

	float GetWorldAltitude()
	{
		return WorldTranslationVect.Z;
	}





/*

	FRotator GetRotationDeg()
	{
		return Rotation;
	}

	FRotator GetRotationRad()
	{
		return FMath::DegreesToRadians<FRotator>(Rotation);
	}

	FVector GetAngularVelocityDeg()
	{
		return AngularVelocity;
	}

	FVector GetAngularVelocityRad()
	{
		//return FVector(FMath::DegreesToRadians(AngularVelocity.X), FMath::DegreesToRadians(AngularVelocity.Y), FMath::DegreesToRadians(AngularVelocity.Z));
		return FMath::DegreesToRadians<FVector>(AngularVelocity);
	}

	FVector GetAngularAccelerationDeg()
	{
		return AngularAcceleration;
	}

	FVector GetAngularAccelerationRad()
	{
		return FMath::DegreesToRadians<FVector>(AngularAcceleration);
	}

*/

	void Debug(FColor Color, FVector2D DebugFontSize)
	{
		
		// Down to up on the debug screen
		GEngine->AddOnScreenDebugMessage(-1, 0, Color, FString::Printf(TEXT("Angular Acceleration (deg/s^2): X=%f Y=%f Z=%f"), AngularAcceleration.X, AngularAcceleration.Y, AngularAcceleration.Z), true, DebugFontSize);
		GEngine->AddOnScreenDebugMessage(-1, 0, Color, FString::Printf(TEXT("Linear Acceleration (m/s^2): %f"), LinearAcceleration), true, DebugFontSize);
		GEngine->AddOnScreenDebugMessage(-1, 0, Color, FString::Printf(TEXT("Angular Velocity (deg/s): X=%f Y=%f Z=%f"), AngularVelocity.X, AngularVelocity.Y, AngularVelocity.Z), true, DebugFontSize);
		GEngine->AddOnScreenDebugMessage(-1, 0, Color, FString::Printf(TEXT("Linear Velocity 2D (km/h): %f"), LinearVelocity2D * 3600.0f / 1000.0f), true, DebugFontSize);
		GEngine->AddOnScreenDebugMessage(-1, 0, Color, FString::Printf(TEXT("Linear Velocity Vector (m/s): X=%f Y=%f Z=%f"), VelocityVector.X, VelocityVector.Y, VelocityVector.Z), true, DebugFontSize);
		GEngine->AddOnScreenDebugMessage(-1, 0, Color, FString::Printf(TEXT("Linear Velocity (m/s): %f"), LinearVelocity), true, DebugFontSize);
		GEngine->AddOnScreenDebugMessage(-1, 0, Color, FString::Printf(TEXT("Rotation (deg): R=%f P=%f Y=%f"), Rotation.Roll, Rotation.Pitch, Rotation.Yaw), true, DebugFontSize);
		GEngine->AddOnScreenDebugMessage(-1, 0, Color, FString::Printf(TEXT("Position (m): X=%f Y=%f Z=%f"), Position.X, Position.Y, Position.Z), true, DebugFontSize);
		
	}


};



