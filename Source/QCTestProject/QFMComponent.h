#pragma once

#include "CoreMinimal.h"

#include "Components/SceneComponent.h"
#include "Components/PrimitiveComponent.h"
#include "PhysicsEngine/BodyInstance.h"

#include "DrawDebugHelpers.h"

#include "PhysXIncludes.h"
#include "PhysicsPublic.h"
#include "PhysXPublic.h"
#include "Classes/PhysicsEngine/PhysicsSettings.h"
#include "Runtime/Engine/Private/PhysicsEngine/PhysXSupport.h"

#include "QFMTypes.h"
#include "QFMDebug.h"

#include "QFMInputController.h"
#include "QFMAHRS.h"
#include "QFMAttitudeController.h"
#include "QFMPositionController.h"
#include "QFMEngineController.h"

#include "QFMComponent.generated.h"

DECLARE_STATS_GROUP(TEXT("QuadcopterFlightModelComponent"), STATGROUP_QuadcopterFlightModel, STATCAT_Advanced);


/*----------------------------------------------------------------------------------------------*/

//UCLASS(Blueprintable, ClassGroup = (Custom), meta = (BlueprintSpawnableComponent))
UCLASS(Blueprintable, ClassGroup = (Quadcopter), meta = (BlueprintSpawnableComponent), HideCategories = (Object, LOD, Physics, Lighting, TextureStreaming, Activation, "Components|Activation", Collision))
class QCTESTPROJECT_API UQuadcopterFlightModel : public USceneComponent
{
    
    GENERATED_BODY()
    
public:

    // Sets default values for this component's properties
    UQuadcopterFlightModel();

    
    /*--- GENERAL ---*/
    UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "QuadcopterFlightModel", meta =(ToolTip="Disable to turn off simulation")) 
	bool Enabled = true;

	/*--- DEBUG CONTROLLER---*/
	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "QuadcopterFlightModel", meta = (ToolTip = "Debug-Output"))
	FQuadcopterFlightModelDebugStruct Debug;

	/*--- PILOT INPUT CONTROLLER---*/
	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "QuadcopterFlightModel", meta = (ToolTip = "Pilot Input")) 
	FInputController PilotInput;

	/*--- AHRS CONTROLLER---*/
	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "QuadcopterFlightModel", meta = (ToolTip = "AHRS")) 
	FAHRS AHRS;
	
	/*--- FLIGHT CONTROLLER ---*/
	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "QuadcopterFlightModel", meta = (ToolTip = "Flight Controller")) 
	FAttitudeController AttitudeController;

	/*--- POSITION CONTROLLER ---*/
	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "QuadcopterFlightModel", meta = (ToolTip = "Position Controller")) 
	FPositionController PositionController;

	/*--- ENGINE CONTROLLER ---*/
	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "QuadcopterFlightModel", meta = (ToolTip = "Engine Controller")) 
	FEngineController EngineController;
	


    /*--- PHYSICS IMPLEMENTATION ---*/
    
    //UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "QuadcopterFlightModel", meta =(ToolTip="Use transformation of this component instead of parent body to determine forward direction etc" )) bool UseThisComponentTransform = false;
	//UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "QuadcopterFlightModel", meta =(ToolTip="Scale all linear forces with mass of the parent body. When enabled linear acceleration is the same regardless of the mass")) bool ScaleWithMass = false;
    //UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "QuadcopterFlightModel", meta =(ToolTip="Scale all angular forces with inertia tensor of the parent body. When enabled angular acceleration is the same regardless of mass and dimensons")) bool ScaleWithMomentOfInertia = false;
    

	// For UQuadcopterFlightModelVehicle
	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "QuadcopterVehicle", meta = (ToolTip = "Frame Mode")) 
	EFrameMode FrameMode = EFrameMode::FrameModeCross;
	
	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "QuadcopterVehicle|CalculateSetting", meta = (ToolTip = "Calculate bodies mass and inertia")) 
	bool CalculateMassProperties = false;
	
	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "QuadcopterVehicle|CalculateSetting", meta = (ToolTip = "Central mass (kg)")) 
	float CentralMass = 10.0f;
	
	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "QuadcopterVehicle|CalculateSetting", meta = (ToolTip = "Central mass radius (m)")) 
	float CentralRadius = 0.25f;
	
	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "QuadcopterVehicle|CalculateSetting", meta = (ToolTip = "Motor mass (kg), one of 4 motors")) 
	float MotorMass = 5.0f;
	
	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "QuadcopterVehicle|CalculateSetting", meta = (ToolTip = "Arm length of each arm (m)")) 
	float ArmLength = 0.5f;
	
	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "QuadcopterVehicle|ManualSetting", meta = (ToolTip = "Manually enter bodies mass and inertia")) 
	bool EnterMassProperties = true;
	
	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "QuadcopterVehicle|ManualSetting", meta = (ToolTip = "Mass in KG")) 
	float Mass = 30.0f;
	
	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "QuadcopterVehicle|ManualSetting", meta = (ToolTip = "Inertia Tensor")) 
	FVector InertiaTensor = FVector(200000.0f, 200000.0f, 400000.0f);
	
	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "QuadcopterVehicle|ManualSetting", meta = (ToolTip = "Center of mass in m")) 
	FVector CenterOfMass = FVector(0.0f, 0.0f, 0.0f);
	


    
	/*--- PUBLIC BP FUNCTIONS AD DELEGATE FOR OTHER CLASSES ---*/

	// For UQuadcopterFlightModelPilotInput

	UFUNCTION(BlueprintCallable, Category = "QuadcopterFlightModel|PilotInput") 
	void InputRoll(float InValue);
	
	UFUNCTION(BlueprintCallable, Category = "QuadcopterFlightModel|PilotInput") 
	void InputPitch(float InValue);
	
	UFUNCTION(BlueprintCallable, Category = "QuadcopterFlightModel|PilotInput") 
	void InputYaw(float InValue);
	
	UFUNCTION(BlueprintCallable, Category = "QuadcopterFlightModel|PilotInput") 
	void InputThrottle(float InValue);
	
	UFUNCTION(BlueprintCallable, Category = "QuadcopterFlightModel|PilotInput") 
	void InputKillTrajectory();
	

	// For UQuadcopterFlightModelEngine

	UFUNCTION(BlueprintCallable, Category = "QuadcopterFlightModel|Engine") 
	float GetEngineRPM(int engineNumber);
	
	UFUNCTION(BlueprintCallable, Category = "QuadcopterFlightModel|Engine") 
	float GetEnginePercent(int engineNumber);
	
	UFUNCTION(BlueprintCallable, Category = "QuadcopterFlightModel|Engine")	
	FVector GetEngineThrust();
	
	UFUNCTION(BlueprintCallable, Category = "QuadcopterFlightModel|Engine") 
	FVector GetEngineTorque();
		
    
    // Called when the game starts
    virtual void BeginPlay() override;
    
    // Called every frame
    virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;

  
	// We declare custom Physics to be called on Substep
	FCalculateCustomPhysics OnCalculateCustomPhysics;

	// Catch up errors if we are on low framerate
	//void ErrorCatchUp(FBodyInstance* bodyInst, float DeltaTime);

	
	// Functions related to Pilot Input
	//void ResetPilotInput(void);
    
    
private:
    
	// We declare custom Physics to be called on Substep
    void CustomPhysics(float DeltaTime, FBodyInstance* bodyInst);

	// Simulation Step. Is called by Tick and by CustomPhysics
    void Simulate(float DeltaTime, FBodyInstance* bodyInst);
    
	// Our Body Instances Parent (Our Primitive Component)
	UPrimitiveComponent *Parent;
	
	// Our Body Instance
	FBodyInstance *BodyInstance;

	// Our Rigid Body
	physx::PxRigidBody* PRigidBody;
		
	// Will be set True if we substep physics
	bool bSubstep;

	
	
	
	// Functions related to force Add Forces to our Parent. This will move and rotate it
	void AddLocalForceZ(FVector forceToApply);
	void AddLocalTorqueRad(FVector torqueToApply);
		/*
	// Calculate forces to apply to get a desired result
	FVector GetLinearImpulseToApply(FVector vDeltaInUEU);
	FVector GetLinearForceToApply(FVector dVelocityInUEU, float dTimeInSec);
	FVector GetAngularImpuleToApplyRad(FVector dAngularVerlocityInRad);
	FVector GetAngularForceToApplyRad(FVector dAngularVerlocityInRad, float dTimeInSec);
	FVector GetRadAngularImpuleToApplyFromDegVelocityDelta(FVector dAngularVerlocityInDeg);
	FVector GetRadAngularForceToApplyFromDegVelocityDelta(FVector dAngularVerlocityInDeg, float dTimeInSec);
	*/

	
	
	// Debug Output
	FORCEINLINE void ScreenMsg(const FString& Msg)
	{
		GEngine->AddOnScreenDebugMessage(-1, 5.f, FColor::Red, *Msg);
	}

	FORCEINLINE void ScreenMsg(const FString& Msg, const FString& Msg2)
	{
		GEngine->AddOnScreenDebugMessage(-1, 5.f, FColor::Red, FString::Printf(TEXT("%s %s"), *Msg, *Msg2));
	}

	    
};


