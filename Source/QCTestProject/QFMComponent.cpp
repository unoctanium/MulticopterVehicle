
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

	// Find My Px Rigid Body
	PRigidBody = BodyInstance->GetPxRigidBody_AssumesLocked();
	

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

		// Set Center Of Mass
		Parent->SetCenterOfMass(BodyInstance->GetCOMPosition() - BodyInstance->GetCOMPosition());
		FTransform transform = Parent->GetComponentTransform();
		BodyInstance->COMNudge = transform.GetTranslation() - BodyInstance->GetCOMPosition() + CenterOfMass * 100;

		// PhyX Inertia Tensor Override
		PRigidBody->setMassSpaceInertiaTensor(physx::PxVec3(InertiaTensor.X, InertiaTensor.Y, InertiaTensor.Z));
		
	}
	// Store Mass Properties in Vehicle Struct
	{
		Mass = BodyInstance->GetBodyMass();
		InertiaTensor = BodyInstance->GetBodyInertiaTensor();
		FTransform transform = Parent->GetComponentTransform();
		CenterOfMass = (BodyInstance->GetCOMPosition() - transform.GetTranslation()) / 100.0f; // To set it in m
	}
	


	// Init all our Subsystems
	PilotInput.Init(BodyInstance, Parent);
	AHRS.Init(BodyInstance, Parent);
	AttitudeController.Init(BodyInstance, Parent, &AHRS, &PositionController, &EngineController);
	PositionController.Init(BodyInstance, Parent);
	EngineController.Init(BodyInstance, Parent);



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






