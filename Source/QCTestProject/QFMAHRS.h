#pragma once

#include "CoreMinimal.h"

#include "Components/SceneComponent.h"
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
	UPROPERTY(EditAnywhere, BlueprintReadOnly, meta = (ToolTip = "Angular Velocity in deg/s")) FVector AngularVelocity = FVector(0.0f, 0.0f, 0.0f);
	UPROPERTY(EditAnywhere, BlueprintReadOnly, meta = (ToolTip = "Linear Acceleration in m/s^2")) float LinearAcceleration = 0.0f;
	UPROPERTY(EditAnywhere, BlueprintReadOnly, meta = (ToolTip = "Angular Acceleration in deg/s^2")) FVector AngularAcceleration = FVector(0.0f, 0.0f, 0.0f);


	// ODO: Need these for FlightController
	// Overthink old values above...
    UPROPERTY() FVector WorldPosition;
    UPROPERTY() FRotator BodyRotation; // NOT!! World Rotation
	UPROPERTY() FVector BodyAngularVelocity;
    


	FBodyInstance *bodyInst;
	USceneComponent *rootComponent;


	void Init(FBodyInstance *bodyInstanceIn, UPrimitiveComponent *primitiveComponentIn)
	{
		// Need these Vectors to read data from those components
		bodyInstance = bodyInstanceIn;
		primitiveComponent = primitiveComponentIn;
	}


	void Tock(float DeltaTime)
	{
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



	}


	FQUAT GetAttitudeQuat()
	{
	}


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


	void Debug(FColor Color, FVector2D DebugFontSize)
	{
		// Down to up on the debug screen
		GEngine->AddOnScreenDebugMessage(-1, 0, Color, FString::Printf(TEXT("Angular Acceleration (deg/s^2): X=%f Y=%f Z=%f"), AngularAcceleration.X, AngularAcceleration.Y, AngularAcceleration.Z), true, DebugFontSize);
		GEngine->AddOnScreenDebugMessage(-1, 0, Color, FString::Printf(TEXT("Linear Acceleration (m/s^2): %f"), LinearAcceleration), true, DebugFontSize);
		GEngine->AddOnScreenDebugMessage(-1, 0, Color, FString::Printf(TEXT("Angular Velocity (deg/s): X=%f Y=%f Z=%f"), AngularVelocity.X, AngularVelocity.Y, AngularVelocity.Z), true, DebugFontSize);
		GEngine->AddOnScreenDebugMessage(-1, 0, Color, FString::Printf(TEXT("Linear Velocity 2D (m/s): %f"), LinearVelocity2D), true, DebugFontSize);
		GEngine->AddOnScreenDebugMessage(-1, 0, Color, FString::Printf(TEXT("Linear Velocity Vector (m/s): X=%f Y=%f Z=%f"), VelocityVector.X, VelocityVector.Y, VelocityVector.Z), true, DebugFontSize);
		GEngine->AddOnScreenDebugMessage(-1, 0, Color, FString::Printf(TEXT("Linear Velocity (m/s): %f"), LinearVelocity), true, DebugFontSize);
		GEngine->AddOnScreenDebugMessage(-1, 0, Color, FString::Printf(TEXT("Rotation (deg): R=%f P=%f Y=%f"), Rotation.Roll, Rotation.Pitch, Rotation.Yaw), true, DebugFontSize);
		GEngine->AddOnScreenDebugMessage(-1, 0, Color, FString::Printf(TEXT("Position (m): X=%f Y=%f Z=%f"), Position.X, Position.Y, Position.Z), true, DebugFontSize);
	}


};



