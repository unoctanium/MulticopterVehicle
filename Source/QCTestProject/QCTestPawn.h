// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "UObject/ObjectMacros.h"

#include "UObject/ConstructorHelpers.h"
#include "GameFramework/Actor.h"
#include "GameFramework/Pawn.h"
#include "Components/SceneComponent.h"
#include "Components/StaticMeshComponent.h"
#include "GameFramework/SpringArmComponent.h"
#include "Camera/CameraComponent.h"
#include "Components/InputComponent.h"
#include "Components/WidgetComponent.h"
#include "Classes/InputCoreTypes.h"
#include <EngineGlobals.h>
#include <Runtime/Engine/Classes/Engine/Engine.h>

#include "HeadMountedDisplay.h"
#include "IXRTrackingSystem.h"

#include "QFMComponent.h"

#include "QFMUDPCustomData.h"
#include "RamaUDPSender.h"

#include "QCTestPawn.generated.h"


//UCLASS()
UCLASS(Blueprintable, ClassGroup = (Custom), meta = (BlueprintSpawnableComponent), HideCategories = (Object, LOD, Physics, Lighting, TextureStreaming, Activation, "Components|Activation", Collision))
class QCTESTPROJECT_API AQCPawn : public APawn
{
	GENERATED_BODY()


public:



	// Root Component of our Pawn
    // I changed the root component to the Mesh!! To revert this: uncomment the following two lines and change the initialization in AcPawn::AcPawn()
	//UPROPERTY(Category = "QuadcopterPawn", VisibleDefaultsOnly, BlueprintReadOnly, meta = (AllowPrivateAccess = "true"))
	//	USceneComponent *PawnRoot;

	// StaticMesh component that will be the visuals for our flying pawn
	UPROPERTY(Category = "QuadcopterPawn", VisibleDefaultsOnly, BlueprintReadOnly, meta = (AllowPrivateAccess = "true"))
		class UStaticMeshComponent* PawnMesh;

	// Our Quadcopter Flight Model which will react to our inputs
	UPROPERTY(Category = "QuadcopterPawn", VisibleDefaultsOnly, BlueprintReadOnly, meta = (AllowPrivateAccess = "true"))
		class UQuadcopterFlightModel* QuadcopterFlightModel;

	// Chase camera
	UPROPERTY(Category = "QuadcopterPawn|Camera", VisibleDefaultsOnly, BlueprintReadOnly, meta = (AllowPrivateAccess = "true"))
		class UCameraComponent* ChaseCamera;

	// Spring arm for chase camera
	UPROPERTY(Category = "QuadcopterPawn|Camera", VisibleDefaultsOnly, BlueprintReadOnly, meta = (AllowPrivateAccess = "true"))
		class USpringArmComponent* ChaseCameraSpringArm;

	// Follow camera
	UPROPERTY(Category = "QuadcopterPawn|Camera", VisibleAnywhere, BlueprintReadOnly, meta = (AllowPrivateAccess = "true"))
		class UCameraComponent* FollowCamera;

	// Spring arm for follow camera
	UPROPERTY(Category = "QuadcopterPawn|Camera", VisibleAnywhere, BlueprintReadOnly, meta = (AllowPrivateAccess = "true"))
		class USpringArmComponent* FollowCameraSpringArm;

	// FPV camera
	UPROPERTY(Category = "QuadcopterPawn|Camera", VisibleAnywhere, BlueprintReadOnly, meta = (AllowPrivateAccess = "true"))
		class UCameraComponent* FpvCamera;

	// Spring arm for FPV camera
	UPROPERTY(Category = "QuadcopterPawn|Camera", VisibleAnywhere, BlueprintReadOnly, meta = (AllowPrivateAccess = "true"))
		class USpringArmComponent* FpvCameraSpringArm;

	UPROPERTY(Category = "QuadcopterPawn|Camera", EditAnywhere, BlueprintReadWrite, meta = (AllowPrivateAccess = "true"))
		float CamDistance = 200.0f;

	UPROPERTY(Category = "QuadcopterPawn|Camera", EditAnywhere, BlueprintReadWrite, meta = (AllowPrivateAccess = "true"))
		float CamElevation = 37.0f;

	UPROPERTY(Category = "QuadcopterPawn|Camera", EditAnywhere, BlueprintReadWrite, meta = (AllowPrivateAccess = "true"))
		float VRCamDistance = 30.0f;

	UPROPERTY(Category = "QuadcopterPawn|Camera", EditAnywhere, BlueprintReadWrite, meta = (AllowPrivateAccess = "true"))
		float VRCamElevation = 80.0f;



	// UI: FPV HUD
	UPROPERTY(Category = "QuadcopterPawn|HUD", EditAnywhere, BlueprintReadWrite, meta = (AllowPrivateAccess = "true"))
    class UWidgetComponent * hudWidget;

	UPROPERTY()
	class UClass * hudWidgetClass;
	
	// Delegates from QuadcopterComponent for BP
	UFUNCTION(BlueprintCallable, Category = "QuadcopterFlightModel|HUD")
	float GetSpeedOverGroundKmh();



	// Networking
	UPROPERTY(Category = "QuadcopterPawn", VisibleDefaultsOnly, BlueprintReadOnly, meta = (AllowPrivateAccess = "true"))
	class URamaUDPSender *UDPSender;

	UPROPERTY()
	float RunningTime = 0.0f;

	UPROPERTY(Category = "QuadcopterPawn|Networking", EditAnywhere, BlueprintReadWrite, meta = (AllowPrivateAccess = "true"))
	float UDPTimer = 0.05f;



public:
	// Sets default values for this pawn's properties
	AQCPawn();

protected:
	// Called when the game starts or when spawned
	virtual void BeginPlay() override;

public:
	// Called every frame
	virtual void Tick(float DeltaTime) override;

	// Called to bind functionality to input
	virtual void SetupPlayerInputComponent(class UInputComponent* PlayerInputComponent) override;

	// Called to notify about a collision
	virtual void NotifyHit(class UPrimitiveComponent* MyComp, class AActor* Other, class UPrimitiveComponent* OtherComp, bool bSelfMoved, FVector HitLocation, FVector HitNormal, FVector NormalImpulse, const FHitResult& Hit) override;

	// Sets up VR 
	void SetupVROptions();

	// Resets HMD Origin position and orientation
	void ResetHMDOrigin();


	// Delegate Input to QuadcopterComponent
	void InputRoll(float inValue);
	void InputPitch(float inValue);
	void InputYaw(float inValue);
	void InputThrottle(float inValue);
	void InputKillTrajectory();






private:
	// Creates a camera and associated spring-arm
	void createCameraWithSpringArm(
		const wchar_t * cameraName,
		UCameraComponent **camera,
		const wchar_t * springArmName,
		USpringArmComponent **springArm,
		float distance,
		float elevation,
		float pitch,
		bool usePawnControlRotation);

	// Helps us cycle among cameras
	unsigned char ActiveCameraIndex; // 0 = Follow, 1 = Chase, 2 = FPV
	void CycleCamera(void);
	void SwitchCamera(int index);
	void SwitchCamera1();
	void SwitchCamera2();
	void SwitchCamera3();
	
	// Rotate Cameras with mouse
	FVector2D MouseInput;
	void InputMouseYaw(float InValue);
	void InputMousePitch(float InValue);
	void RotateCameraWithMouse();

	// Zoom Camera with Mouse wheel
	void InputCameraZoomIn();
	void InputCameraZoomOut();
	float ChaseCameraZoomFactor;
	float FollowCameraZoomFactor;

	// Timer for UDP
	float DeltaTimeUDP;


public:

	FORCEINLINE class UStaticMeshComponent* GetVehicleMesh() const { return PawnMesh; }
	FORCEINLINE class USpringArmComponent* GetFollowCameraSpringArm() const { return FollowCameraSpringArm; }
	FORCEINLINE class UCameraComponent* GetFollowCamera() const { return FollowCamera; }
	FORCEINLINE class USpringArmComponent* GetChaseCameraSpringArm() const { return ChaseCameraSpringArm; }
	FORCEINLINE class UCameraComponent* GetChaseCamera() const { return ChaseCamera; }

};


