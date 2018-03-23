
#include "QCTestPawn.h"

// Sets default values
AQCPawn::AQCPawn()
{
	// Set this pawn to call Tick() every frame.  You can turn this off to improve performance if you don't need it.

	PrimaryActorTick.bCanEverTick = true;

    // New: Create a Mesh component as the root component of our pawn
	PawnMesh = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("QuadcopterMesh"));
    RootComponent = PawnMesh;
    
    // Old: Create a Scene component as the root component of our pawn
    /*
    // Create a Scene component as the root component of our pawn
    PawnRoot = CreateDefaultSubobject<USceneComponent>(TEXT("QuadcopterRootComponent"));
    RootComponent = PawnRoot;
     
    // Create and position a mesh component so we can see where our Pawn is
    PawnMesh = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("QuadcopterMesh"));
    PawnMesh->SetupAttachment(RootComponent);
    */
    
    
	// And load a mesh into it
	
	static ConstructorHelpers::FObjectFinder<UStaticMesh> MeshVisualAsset(TEXT("/Game/QC/Meshes/3DFly")); 
																										
	if (MeshVisualAsset.Succeeded())
	{
		PawnMesh->SetStaticMesh(MeshVisualAsset.Object);
		PawnMesh->SetRelativeLocation(FVector(0.0f, 0.0f, 0.0f));
		PawnMesh->SetWorldScale3D(FVector(MeshScale));
	}
	//RootComponent = PawnMesh;


	// Setup Physics
	PawnMesh->SetCollisionProfileName(TEXT("BlockAll"));
	PawnMesh->SetNotifyRigidBodyCollision(true);
	PawnMesh->SetSimulatePhysics(true);
	PawnMesh->SetEnableGravity(true);
	PawnMesh->SetLinearDamping(0.15f);
	PawnMesh->SetAngularDamping(0.15f);

    
	// Create a Quadcopter Component
	
	QuadcopterFlightModel = CreateDefaultSubobject<UQuadcopterFlightModel>(TEXT("Quadcopter Flight Model"));
	QuadcopterFlightModel->SetupAttachment(PawnMesh);

    
	// Create the FPV camera
	
	createCameraWithSpringArm(L"FpvCamera", &FpvCamera, L"FpvCameraSpringArm",
		&FpvCameraSpringArm, VRCamDistance,  VRCamElevation, 0, false);
	FpvCameraSpringArm->SetRelativeScale3D(FVector(1 / MeshScale, 1 / MeshScale, 1 / MeshScale));


	// Create the follow camera
	
	createCameraWithSpringArm(L"FollowCamera", &FollowCamera, L"FollowCameraSpringArm",
		&FollowCameraSpringArm, CamDistance, CamElevation, 0, true);
	// We have to put these to false to be able to rotate the view with the mouse
	FollowCameraSpringArm->bInheritRoll = false;
	FollowCameraSpringArm->bInheritPitch = false;
	FollowCameraSpringArm->bInheritYaw = false;
	// Initialize Zoom Factor
	FollowCameraZoomFactor = FollowCameraSpringArm->TargetArmLength;


	// Create the chase camera
	
	createCameraWithSpringArm(L"ChaseCamera", &ChaseCamera, L"ChaseCameraSpringArm",
		&ChaseCameraSpringArm, CamDistance, CamElevation, 0, false);
	// Initialize Zoom Factor
	ChaseCameraZoomFactor = ChaseCameraSpringArm->TargetArmLength;


	// Create UDP Actor
	UDPSender = CreateDefaultSubobject<URamaUDPSender>(TEXT("UDP Sender"));
	UDPSender->AttachToComponent(RootComponent, FAttachmentTransformRules::SnapToTargetIncludingScale);


	// Take control of the default player
	AutoPossessPlayer = EAutoReceiveInput::Player0;

}

// Called when the game starts or when spawned
void AQCPawn::BeginPlay()
{
	Super::BeginPlay();

	// Setup Camera
	// Start with the FPV camera activated
	SwitchCamera(2);

	// Setup VR
	SetupVROptions();

	// Start UDP Sender
	UDPSender->Start("Socket1","127.0.0.1",12345);
}

// Called every frame
void AQCPawn::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);
	
	// soft zoom cameras by mouse wheel
	switch (ActiveCameraIndex)
	{
	case 0: // Follow
		FollowCameraSpringArm->TargetArmLength = FMath::FInterpTo(FollowCameraSpringArm->TargetArmLength, FollowCameraZoomFactor, DeltaTime, 3);
		break;
	case 1: // chase
		ChaseCameraSpringArm->TargetArmLength = FMath::FInterpTo(ChaseCameraSpringArm->TargetArmLength, ChaseCameraZoomFactor, DeltaTime, 3);
		break;
	}

	// Write some UDP Test Data
	DeltaTimeUDP += DeltaTime;
	if (DeltaTimeUDP > UDPTimer)
	{
		FString ToSend = FString::Printf(TEXT("%f,%f"), RunningTime, FMath::Sin(RunningTime));
		UDPSender->SendData(ToSend);
		DeltaTimeUDP = 0.0f;
	}
	RunningTime += DeltaTime;

}




/* Input Related stuff */


// Called to bind functionality to input
void AQCPawn::SetupPlayerInputComponent(UInputComponent* PlayerInputComponent)
{
	APawn::SetupPlayerInputComponent(PlayerInputComponent);

	// Pawn Movement
	PlayerInputComponent->BindAxis("Roll", this, &AQCPawn::InputRoll);
	PlayerInputComponent->BindAxis("Pitch", this, &AQCPawn::InputPitch);
	PlayerInputComponent->BindAxis("Yaw", this, &AQCPawn::InputYaw);
	PlayerInputComponent->BindAxis("Throttle", this, &AQCPawn::InputThrottle);
	PlayerInputComponent->BindAction("EmergencyStop", IE_Pressed, this, &AQCPawn::InputKillTrajectory);

	// Pawn Camera Control
	PlayerInputComponent->BindAction("CycleCamera", IE_Pressed, this, &AQCPawn::CycleCamera);
	PlayerInputComponent->BindAction("Camera1", IE_Pressed, this, &AQCPawn::SwitchCamera1);
	PlayerInputComponent->BindAction("Camera2", IE_Pressed, this, &AQCPawn::SwitchCamera2);
	PlayerInputComponent->BindAction("Camera3", IE_Pressed, this, &AQCPawn::SwitchCamera3);
	// Reset VR Camera
	PlayerInputComponent->BindAction("ResetVR", IE_Pressed, this, &AQCPawn::ResetHMDOrigin);
	// Mouse Look Pawn Camera
	PlayerInputComponent->BindAxis("CameraPitch", this, &AQCPawn::InputMousePitch);
	PlayerInputComponent->BindAxis("CameraYaw", this, &AQCPawn::InputMouseYaw);
	PlayerInputComponent->BindAction("CameraZoomIn", IE_Pressed, this, &AQCPawn::InputCameraZoomIn);
	PlayerInputComponent->BindAction("CameraZoomOut", IE_Pressed, this, &AQCPawn::InputCameraZoomOut);
}




/* Collision related stuff */


void AQCPawn::NotifyHit(class UPrimitiveComponent* MyComp, class AActor* Other, class UPrimitiveComponent* OtherComp, bool bSelfMoved, FVector HitLocation, FVector HitNormal, FVector NormalImpulse, const FHitResult& Hit)
{
	GEngine->AddOnScreenDebugMessage(20, 1, FColor::Red, FString::Printf(TEXT("COLLIDE: %s"), *Other->GetName()), true, FVector2D(1,1));
}



/* Camera related stuff */

// Switch to a camera
void AQCPawn::SwitchCamera(int index)
{
	ActiveCameraIndex = index;

	switch (ActiveCameraIndex) {

	case 1:
		FollowCamera->Deactivate();
		ChaseCamera->Activate();
		FpvCamera->Deactivate();
		break;
	case 2:
		FollowCamera->Deactivate();
		ChaseCamera->Deactivate();
		FpvCamera->Activate();
		break;
	default: // 0 = Cam 1
		FollowCamera->Activate();
		ChaseCamera->Deactivate();
		FpvCamera->Deactivate();
	}
}

void AQCPawn::SwitchCamera1()
{
	SwitchCamera(0);
}

void AQCPawn::SwitchCamera2()
{
	SwitchCamera(1);
}

void AQCPawn::SwitchCamera3()
{
	SwitchCamera(2);
}


// Cycles among our three cameras
void AQCPawn::CycleCamera(void)
{
	ActiveCameraIndex = (ActiveCameraIndex + 1) % 3;
	SwitchCamera(ActiveCameraIndex);
}


void AQCPawn::createCameraWithSpringArm(
	const wchar_t * cameraName,
	UCameraComponent **camera,
	const wchar_t * springArmName,
	USpringArmComponent **springArm,
	float distance,
	float elevation,
	float pitch,
	bool usePawnControlRotation)
{
	*springArm = CreateDefaultSubobject<USpringArmComponent>(springArmName);
	(*springArm)->SetupAttachment(PawnMesh);
	(*springArm)->TargetArmLength = distance;
	(*springArm)->SetRelativeLocation(FVector(0.f, 0.f, elevation));
	(*springArm)->bUsePawnControlRotation = usePawnControlRotation;
	(*springArm)->SetWorldRotation(FRotator(pitch, 0.f, 0.f));

	*camera = CreateDefaultSubobject<UCameraComponent>(cameraName);
	(*camera)->SetupAttachment(*springArm, USpringArmComponent::SocketName);
	(*camera)->bUsePawnControlRotation = false; // Camera does not rotate relative to arm
												//(*camera)->FieldOfView = 90.0f;
}

void AQCPawn::InputMouseYaw(float InValue)
{
	MouseInput.X = InValue;
	RotateCameraWithMouse();
}

void AQCPawn::InputMousePitch(float InValue)
{
	MouseInput.Y = InValue;
	RotateCameraWithMouse();
}

void AQCPawn::InputCameraZoomIn()
{
	switch (ActiveCameraIndex)
	{
	case 0: // Follow
		FollowCameraZoomFactor = FMath::Clamp<float>(FollowCameraZoomFactor - 20, 0.0f, 1000.0f);
		break;
	case 1: // chase
		ChaseCameraZoomFactor = FMath::Clamp<float>(ChaseCameraZoomFactor - 20, 0.0f, 1000.0f);
		break;
	}
	
}

void AQCPawn::InputCameraZoomOut()
{
	switch (ActiveCameraIndex)
	{
	case 0: // Follow
		FollowCameraZoomFactor = FMath::Clamp<float>(FollowCameraZoomFactor + 20, 0.0f, 1000.0f);
		break;
	case 1: // chase
		ChaseCameraZoomFactor = FMath::Clamp<float>(ChaseCameraZoomFactor + 20, 0.0f, 1000.0f);
		break;
	}
}



void AQCPawn::RotateCameraWithMouse()
{

	USceneComponent *RotatingComponent;

	switch (ActiveCameraIndex)
	{
	case 0: 
		RotatingComponent = FollowCameraSpringArm;
		break;
	case 1: 
		RotatingComponent = ChaseCameraSpringArm;
		break;
	case 2: 
		RotatingComponent = FpvCamera;
		break;
	default:
		return;
	}

	// I am sure, this could be done more elegant ;-)
	FRotator CamMouseRotation = FRotator(MouseInput.Y, MouseInput.X, 0.0f);
	FTransform RotatingComponentTransform = RotatingComponent->GetRelativeTransform();
	FRotator RotatingComponentNewRotation = RotatingComponentTransform.Rotator() + CamMouseRotation;
	RotatingComponentNewRotation = RotatingComponentNewRotation.Clamp();
	RotatingComponentNewRotation.Pitch = FMath::ClampAngle(RotatingComponentNewRotation.Pitch, -80.0, 80.0);
	RotatingComponentTransform.SetRotation(FQuat(RotatingComponentNewRotation));
	RotatingComponent->SetRelativeTransform(RotatingComponentTransform);

}




/* VR Related stuff */


void AQCPawn::SetupVROptions()
{
	FpvCamera->SetRelativeLocation(FVector(0, 0, 0));
}


void AQCPawn::ResetHMDOrigin()
{

	GEngine->XRSystem->ResetOrientationAndPosition();
}



/* No Idea why I cant use the delegates directly un BindAction */

void AQCPawn::InputRoll(float inValue)
{
	QuadcopterFlightModel->InputRoll(inValue);
}

void AQCPawn::InputPitch(float inValue)
{
	QuadcopterFlightModel->InputPitch(inValue);
}

void AQCPawn::InputYaw(float inValue)
{
	QuadcopterFlightModel->InputYaw(inValue);
}

void AQCPawn::InputThrottle(float inValue)
{
	QuadcopterFlightModel->InputThrottle(inValue);
}

void AQCPawn::InputKillTrajectory()
{
	QuadcopterFlightModel->InputKillTrajectory();
}

