
#include "QFMComponent.h"


UQuadcopterFlightModel::UQuadcopterFlightModel()
{
	// Set this component to be initialized when the game starts, and to be ticked every frame.  You can turn these features
	// off to improve performance if you don't need them.
	PrimaryComponentTick.bCanEverTick = true;
	// ...

	OnCalculateCustomPhysics.BindUObject(this, &UQuadcopterFlightModel::CustomPhysics);
	SetTickGroup(ETickingGroup::TG_PrePhysics);
	
}



// Called when the game starts
void UQuadcopterFlightModel::BeginPlay()
{
	Super::BeginPlay();
		
	// Find Parent and Socket
	Parent = Cast<UPrimitiveComponent>(GetAttachParent());
	//FName ParentSocket = GetAttachSocketName();

	//check attachment
	if (!Parent) {
		GEngine->AddOnScreenDebugMessage(-1, 0, FColor::Red, TEXT("QuadcopterFlightModel ERROR - not attached to UPrimitiveComponent"));
		return;
	}
	if (!Parent->IsSimulatingPhysics()) {
		GEngine->AddOnScreenDebugMessage(-1, 0, FColor::Red, TEXT("QuadcopterFlightModel ERROR - parent object doesn't simulate physics"));
		return;
	}

	// Find My BodyInstance
	BodyInstance = Parent->GetBodyInstance();

	// Init all our Subsystems
	Vehicle.Init(BodyInstance, Parent);
	PilotInput.Init(BodyInstance, Parent);
	AHRS.Init(BodyInstance, Parent);
	AttitudeController.Init(BodyInstance, Parent, &PilotInput, &AHRS, &PositionController, &EngineController);
	PositionController.Init(BodyInstance, Parent, &AHRS);
	EngineController.Init(BodyInstance, Parent, &Vehicle);

	// Prepare Substepping, if requested
	const UPhysicsSettings* Settings = GetDefault<UPhysicsSettings>();
	if (Settings) {
		bSubstep = Settings->bSubstepping;
	}
	else {
		UE_LOG(LogTemp, Error, TEXT("Project settings inaccessible"));
		bSubstep = false;
	}

}

//physics substep
void UQuadcopterFlightModel::CustomPhysics(float DeltaTime, FBodyInstance* bodyInst)
{
	Simulate(DeltaTime, bodyInst);
}


// Called every frame
void UQuadcopterFlightModel::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{

	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);
	if (!Enabled) return;

    //UE_LOG(LogTemp, Error, TEXT("TICK"));
  
	if (bSubstep) {
		BodyInstance->AddCustomPhysics(OnCalculateCustomPhysics);
	}
	else {
		Simulate(DeltaTime, BodyInstance);
	}
}






// Pilot Input Related Stuff

void UQuadcopterFlightModel::InputRoll(float InValue) 
{	
	PilotInput.RollAxisInput = InValue; 
}

void UQuadcopterFlightModel::InputPitch(float InValue) 
{ 
	PilotInput.PitchAxisInput = InValue; 
}

void UQuadcopterFlightModel::InputYaw(float InValue) 
{ 
	PilotInput.YawAxisInput = InValue; 
}

void UQuadcopterFlightModel::InputThrottle(float InValue) 
{ 
	PilotInput.ThrottleAxisInput = InValue; 
}

// Reset all speeds and accelerations
void UQuadcopterFlightModel::InputKillTrajectory()
{
	BodyInstance->SetAngularVelocityInRadians(FVector(0, 0, 0), false);
	BodyInstance->SetLinearVelocity(FVector(0, 0, 0), false);
}


// Engine related stuff. These are callable from Blueprint

float UQuadcopterFlightModel::GetEnginePercent(int engineNumber) 
{ 
	return EngineController.GetEnginePercent(engineNumber); 
}


float UQuadcopterFlightModel::GetEngineRPM(int engineNumber) 
{ 
	return EngineController.GetEngineRPM(engineNumber);
}


FVector UQuadcopterFlightModel::GetTotalThrust() 
{	
	return EngineController.GetTotalThrust(); 
}


FVector UQuadcopterFlightModel::GetTotalTorque() 
{	
	return EngineController.GetTotalTorque(); 
}


