
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
	PositionController.Init(BodyInstance, Parent, &AHRS, &Vehicle, &EngineController);
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


FVector UQuadcopterFlightModel::GetUDPDebugOutput()
{
	return AttitudeController.GetUDPDebugOutput();
}


float UQuadcopterFlightModel::GetSpeedOverGroundKmh()
{
	// 1 m/s = 3.6 km/h
	return AHRS.LinearVelocity2D * 3.6;
}

float UQuadcopterFlightModel::GetSpeedOverGroundMph()
{
	// 1 mph = 1,609344 kmh.
	return AHRS.LinearVelocity2D / 1.609344 * 3.6;
}


float UQuadcopterFlightModel::GetTrueAirspeedKmh()
{
	// 1 m/s = 3.6 km/h
	return AHRS.LinearVelocity * 3.6;
}

float UQuadcopterFlightModel::GetTrueAirspeedMph()
{
	// 1 mph = 1,609344 kmh.
	return AHRS.LinearVelocity / 1.609344 * 3.6;
}

float UQuadcopterFlightModel::GetForwardSpeedOverGroundKmh()
{
	return AHRS.LinearVelocityX * 3.6;
}

float UQuadcopterFlightModel::GetForwardSpeedOverGroundMph()
{
	// 1 mph = 1,609344 kmh.
	return AHRS.LinearVelocityX / 1.609344 * 3.6;
}

float UQuadcopterFlightModel::GetTrueAltitudeM()
{
	// 1 m = 3,28084 ft.
	return AHRS.Position.Z * 3.28084;
}


float UQuadcopterFlightModel::GetTrueAltitudeFt()
{
	// 1 m = 3,28084 ft.
	return AHRS.Position.Z * 3.28084;
}

float UQuadcopterFlightModel::GetCompassDirectionNorm()
{
	// Returns Z World Orientation in 0..1
	//float NormAngle = FMath::ClampAngle(AHRS.Rotation.Yaw, 0, 360) / 360;
	float NormAngle = fmod(AHRS.Rotation.Yaw + 360, 360) / 360 + 0.5;
	//UE_LOG(LogTemp,Display, TEXT("%f"),NormAngle);
	return NormAngle;
}

float UQuadcopterFlightModel::GetAttitudePitchNorm()
{
	float NormAngle = AHRS.Rotation.Pitch / 90 ; // -1: down, 1: up
	//UE_LOG(LogTemp,Display, TEXT("%f"),NormAngle);
	return NormAngle;
}

float UQuadcopterFlightModel::GetAttitudeRollNorm()
{
	float NormAngle = fmod(AHRS.Rotation.Roll + 360, 360) / 360;
	//UE_LOG(LogTemp,Display, TEXT("%f"),NormAngle);
	return NormAngle;
}
