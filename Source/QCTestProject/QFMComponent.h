
// ODO: GetMappedAndClampedValue kann ich bestimmt noch optimieren


#pragma once

//#include "QFMPlugin.h"

#include "Engine.h"
#include "Components/SceneComponent.h"

#include "CoreMinimal.h"
#include "UObject/ObjectMacros.h"

#include "PhysicsEngine/BodyInstance.h"
#include "Engine/World.h"

#include "GameFramework/Pawn.h"
#include "HeadMountedDisplay.h"
#include "IXRTrackingSystem.h"

#include "QFMDebug.h"
#include "QFMTrajectory.h"
#include "QFMFlightController.h"

#include "QFMComponent.generated.h"

DECLARE_STATS_GROUP(TEXT("QuadcopterFlightModelComponent"), STATGROUP_QuadcopterFlightModel, STATCAT_Advanced);


// Enumeration of supported Frame Types
UENUM(BlueprintType)
enum class EFrameMode : uint8
{
    FrameModeCross 	UMETA(DisplayName="Cross"),
	FrameModePlus 	UMETA(DisplayName="Plus")
};


USTRUCT()
struct FQuadcopterFlightModelMixerStruct
{
	GENERATED_BODY()

	UPROPERTY() float Throttle;
	UPROPERTY() float Roll;
	UPROPERTY() float Pitch;
	UPROPERTY() float Yaw;

	// constructor that takes parameters
	FQuadcopterFlightModelMixerStruct(float InThrottle, float InRoll, float InPitch, float InYaw)
		: Throttle(InThrottle), Roll(InRoll), Pitch(InPitch), Yaw(InYaw)
	{};

	// constructor that takes no parameters
	FQuadcopterFlightModelMixerStruct()
	{};
	
};




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
    UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "QuadcopterFlightModel", meta =(ToolTip="Disable to turn off simulation")) bool Enabled = true;
    	
	/*--- TRAJECTORY CALCULATOR---*/
	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "QuadcopterFlightModel", meta = (ToolTip = "Quadcopter Trajectory")) FQuadcopterFlightModelTrajectoryStruct Trajectory;

	
	///////////////////////////
	// THIS IS WORK IN PROGRESS
	///////////////////////////

	///////////////////////////

	///////////////////////////

	/*--- FLIGHT CONTROLLER ---*/
	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "QuadcopterFlightModel", meta = (ToolTip = "Quadcopter FlightController")) FFlightController FlightController;


	///////////////////////////

	///////////////////////////






    /*--- PHYSICS IMPLEMENTATION ---*/
    
    //UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "QuadcopterFlightModel", meta =(ToolTip="Use transformation of this component instead of parent body to determine forward direction etc" )) bool UseThisComponentTransform = false;
	//UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "QuadcopterFlightModel", meta =(ToolTip="Scale all linear forces with mass of the parent body. When enabled linear acceleration is the same regardless of the mass")) bool ScaleWithMass = false;
    //UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "QuadcopterFlightModel", meta =(ToolTip="Scale all angular forces with inertia tensor of the parent body. When enabled angular acceleration is the same regardless of mass and dimensons")) bool ScaleWithMomentOfInertia = false;
    
    
    /*--- DEBUG ---*/
	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "QuadcopterFlightModel", meta = (ToolTip = "Debug-Output")) FQuadcopterFlightModelDebugStruct Debug;




	/*--- PUBLIC BP POPERTIES AD DELEGATE FOR OTHER CLASSES ---*/
	
	// For UQuadcopterFlightModelPilotInput

	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "QuadcopterPilotInput|AxisInput", meta = (ToolTip = "Stick: Roll Axis Input")) float RollAxisInput = 0.0f;
	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "QuadcopterPilotInput|AxisInput", meta = (ToolTip = "Stick: Pitch Axis Input")) float PitchAxisInput = 0.0f;
	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "QuadcopterPilotInput|AxisInput", meta = (ToolTip = "Stick: Yaw Axi Input")) float YawAxisInput = 0.0f;
	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "QuadcopterPilotInput|AxisInput", meta = (ToolTip = "Stick: Throttle Axis Input")) float ThrottleAxisInput = 0.0f;
	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "QuadcopterPilotInput|AxisSettings", meta = (ToolTip = "Stick: Roll Min/Max Values")) FVector2D RollAxisInputInterval = FVector2D(-1.0f, 1.0f);
	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "QuadcopterPilotInput|AxisSettings", meta = (ToolTip = "Stick: Pitch Min/Max Values")) FVector2D PitchAxisInputInterval = FVector2D(-1.0f, 1.0f);
	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "QuadcopterPilotInput|AxisSettings", meta = (ToolTip = "Stick: Yaw Min/Max Values")) FVector2D YawAxisInputInterval = FVector2D(-1.0f, 1.0f);
	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "QuadcopterPilotInput|AxisSettings", meta = (ToolTip = "Stick: Throttle Min/Max Values")) FVector2D ThrottleAxisInputInterval = FVector2D(-1.0f, 1.0f);
	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "QuadcopterPilotInput|AxisSettings", meta = (ToolTip = "Stick: Sacle Axis R,P,Y,T. Negative to invert")) FVector4 InputAxisScale = FVector4(1.0f, 1.0f, 1.0f, 1.0f);

	// For UQuadcopterFlightModelVehicle
	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "QuadcopterVehicle", meta = (ToolTip = "Frame Mode")) EFrameMode FrameMode = EFrameMode::FrameModeCross;
	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "QuadcopterVehicle|CalculateSetting", meta = (ToolTip = "Calculate bodies mass and inertia")) bool CalculateMassProperties = false;
	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "QuadcopterVehicle|CalculateSetting", meta = (ToolTip = "Central mass (kg)")) float CentralMass = 10.0f;
	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "QuadcopterVehicle|CalculateSetting", meta = (ToolTip = "Central mass radius (m)")) float CentralRadius = 0.25f;
	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "QuadcopterVehicle|CalculateSetting", meta = (ToolTip = "Motor mass (kg), one of 4 motors")) float MotorMass = 5.0f;
	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "QuadcopterVehicle|CalculateSetting", meta = (ToolTip = "Arm length of each arm (m)")) float ArmLength = 0.5f;
	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "QuadcopterVehicle|ManualSetting", meta = (ToolTip = "Manually enter bodies mass and inertia")) bool EnterMassProperties = true;
	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "QuadcopterVehicle|ManualSetting", meta = (ToolTip = "Mass in KG")) float Mass = 30.0f;
	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "QuadcopterVehicle|ManualSetting", meta = (ToolTip = "Inertia Tensor")) FVector InertiaTensor = FVector(200000.0f, 200000.0f, 400000.0f);
	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "QuadcopterVehicle|ManualSetting", meta = (ToolTip = "Center of mass in m")) FVector CenterOfMass = FVector(0.0f, 0.0f, 0.0f);
	
	// For UQuadcopterFlightModelEngine
	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "QuadcopterEngineSettings", meta = (ToolTip = "Engine Max RPM")) float EngineMaxRPM = 1000.0f;
	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "QuadcopterEngineSettings", meta = (ToolTip = "Engine Thrust Coefficient K")) float Engine_K = 50000.0f;
	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "QuadcopterEngineSettings", meta = (ToolTip = "Engine Thrust Exponent")) float Engine_Q = 2.0f;
	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "QuadcopterEngineSettings", meta = (ToolTip = "Engine Torque Yaw Coefficient")) float Engine_B = 500000.0f;
	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "QuadcopterEngineSettings", meta = (ToolTip = "Engine Torque Exponent")) float Engine_QQ = 2.0f;
	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "QuadcopterEngineSettings", meta = (ToolTip = "Length of Arm in m")) float Engine_L = 2.0f;
	
    
	/*--- PUBLIC BP FUNCTIONS AD DELEGATE FOR OTHER CLASSES ---*/

	// For UQuadcopterFlightModelPilotInput

	UFUNCTION(BlueprintCallable, Category = "QuadcopterFlightModel|PilotInput") void InputRoll(float InValue);
	UFUNCTION(BlueprintCallable, Category = "QuadcopterFlightModel|PilotInput") void InputPitch(float InValue);
	UFUNCTION(BlueprintCallable, Category = "QuadcopterFlightModel|PilotInput") void InputYaw(float InValue);
	UFUNCTION(BlueprintCallable, Category = "QuadcopterFlightModel|PilotInput") void InputThrottle(float InValue);
	UFUNCTION(BlueprintCallable, Category = "QuadcopterFlightModel|PilotInput") void InputKillTrajectory();
	

	// For UQuadcopterFlightModelEngine

	UFUNCTION(BlueprintCallable, Category = "QuadcopterFlightModel|Engine") float GetEngineRPM(int engineNumber);
	UFUNCTION(BlueprintCallable, Category = "QuadcopterFlightModel|Engine") float GetEnginePercent(int engineNumber);
	UFUNCTION(BlueprintCallable, Category = "QuadcopterFlightModel|Engine")	FVector GetEngineThrust();
	UFUNCTION(BlueprintCallable, Category = "QuadcopterFlightModel|Engine") FVector GetEngineTorque();
		
    
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
    void Simulate(FBodyInstance* bodyInst, float DeltaTime);
    
	// Our Body Instances Parent (Our Primitive Component)
	UPrimitiveComponent *Parent;
	
	// Our Body Instance
	FBodyInstance *BodyInstance;

	// Our Rigid Body
	physx::PxRigidBody* PRigidBody;
		
	// Will be set True if we substep physics
	bool bSubstep;

	
	// Pilot Input
	FVector4 DesiredPilotInput = FVector4(0.0f, 0.0f, 0.0f, 0.0f);
	
	// Mixer
	FQuadcopterFlightModelMixerStruct MixerQuadCross[4] = {
		{ +1, -1,  -1, -1 }, // RF-
		{ +1, -1,  +1, +1 }, // RB+
		{ +1, +1,  +1, -1 }, // LB-
		{ +1, +1,  -1, +1 } // LF+
	};
		
	FQuadcopterFlightModelMixerStruct MixerQuadPlus[4] = {
		{ +1, 0,  -1, -1 }, // F-
		{ +1, -1,  +1, +1 }, // R+
		{ +1, 0,  +1, -1 }, // B-
		{ +1, +1,  -1, +1 } // L+
	};
	
	float EngineMixPercent[4] = {0.0f, 0.0f, 0.0f, 0.0f};
	
	
	
	// Engines
	float EngineSpeed[4] = {0.0f, 0.0f, 0.0f, 0.0f}; // 0..1
	FVector TotalThrust = FVector(0.0f, 0.0f, 0.0f);
	FVector TotalTorque = FVector(0.0f, 0.0f, 0.0f);;
	
				
			
	// Functions related to Pilot Input
	float GetMappedAndClampedValueNormal(const FVector2D& InputRange, const float Value);
	float GetMappedAndClampedValue(const FVector2D& InputRange, const FVector2D& OutputRange, const float Value);

	// Functions related to Engines
	void SetEnginePercent(int engineNumber, float inValue);
    void SetEngineRPM(int engineNumber, float inValue);
	void GetEngineForces();
	//FVector GetTorquesCross4(void);
	//FVector GetTorquesPlus4(void);
	
	
	// Functions related to Mixer
	void MixEngines(void);
	void SetEnginesFromMixer(void);
	
	
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


