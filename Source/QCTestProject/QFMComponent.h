#pragma once

#include "CoreMinimal.h"

#include "Components/SceneComponent.h"
#include "Components/PrimitiveComponent.h"
#include "PhysicsEngine/BodyInstance.h"

#include "DrawDebugHelpers.h"

#include "QFMTypes.h"
#include "QFMDebug.h"

#include "QFMInputController.h"
#include "QFMVehicle.h"
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

	/*--- VEHICLE ---*/
	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "QuadcopterFlightModel", meta = (ToolTip = "Vehicle Data"))
	FVehicle Vehicle;

	/*--- PILOT INPUT CONTROLLER---*/
	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "QuadcopterFlightModel", meta = (ToolTip = "Pilot Input")) 
	FInputController PilotInput;

	/*--- AHRS CONTROLLER---*/
	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "QuadcopterFlightModel", meta = (ToolTip = "AHRS")) 
	FAHRS AHRS;
	
	/*--- FLIGHT CONTROLLER ---*/
	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "QuadcopterFlightModel", meta = (ToolTip = "Attitude Controller")) 
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

	UFUNCTION(BlueprintCallable, Category = "QuadcopterFlightModel|HUD") 
	float GetEngineRPM(int engineNumber);
	
	UFUNCTION(BlueprintCallable, Category = "QuadcopterFlightModel|HUD") 
	float GetEnginePercent(int engineNumber);
	
	UFUNCTION(BlueprintCallable, Category = "QuadcopterFlightModel|HUD")	
	FVector GetTotalThrust();
	
	UFUNCTION(BlueprintCallable, Category = "QuadcopterFlightModel|HUD") 
	FVector GetTotalTorque();
		
    UFUNCTION(BlueprintCallable, Category = "QuadcopterFlightModel|HUD") 
	float GetSpeedOverGroundKmh();

    UFUNCTION(BlueprintCallable, Category = "QuadcopterFlightModel|HUD") 
	float GetSpeedOverGroundMph();

	UFUNCTION(BlueprintCallable, Category = "QuadcopterFlightModel|HUD") 
	float GetTrueAirspeedKmh();

    UFUNCTION(BlueprintCallable, Category = "QuadcopterFlightModel|HUD") 
	float GetTrueAirspeedMph();

	UFUNCTION(BlueprintCallable, Category = "QuadcopterFlightModel|HUD") 
	float GetForwardSpeedOverGroundKmh();

    UFUNCTION(BlueprintCallable, Category = "QuadcopterFlightModel|HUD") 
	float GetForwardSpeedOverGroundMph();

    UFUNCTION(BlueprintCallable, Category = "QuadcopterFlightModel|HUD") 
	float GetTrueAltitudeM();

	UFUNCTION(BlueprintCallable, Category = "QuadcopterFlightModel|HUD") 
	float GetTrueAltitudeFt();

	UFUNCTION(BlueprintCallable, Category = "QuadcopterFlightModel|HUD") 
	float GetCompassDirectionNorm();

	UFUNCTION(BlueprintCallable, Category = "QuadcopterFlightModel|HUD") 
	float GetAttitudePitchNorm();

	UFUNCTION(BlueprintCallable, Category = "QuadcopterFlightModel|HUD") 
	float GetAttitudeRollNorm();
	


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
    

	// UDP Debug Output
	UFUNCTION(BlueprintCallable, Category = "QuadcopterFlightModel|Engine") 
	FVector GetUDPDebugOutput();

    
private:
    
	// We declare custom Physics to be called on Substep
    void CustomPhysics(float DeltaTime, FBodyInstance* bodyInst);

	// Simulation Step. Is called by Tick and by CustomPhysics
    void Simulate(float DeltaTime, FBodyInstance* bodyInst);
    
	// Our Body Instances Parent (Our Primitive Component)
	UPrimitiveComponent *Parent;
	
	// Our Body Instance
	FBodyInstance *BodyInstance;

	// Will be set True if we substep physics
	bool bSubstep;

	
	
	
	// Functions related to force Add Forces to our Parent. This will move and rotate it
	void AddLocalForceZ(FVector forceToApply);
	void AddLocalTorque(FVector torqueToApply);
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


