
#pragma once

#include "CoreMinimal.h"
#include "UObject/ObjectMacros.h"

#include "Components/SceneComponent.h"
#include "Components/PrimitiveComponent.h"
#include "PhysicsEngine/BodyInstance.h"

#include "QFMTypes.h"
#include "QFMPIDController.h"

#include "QFMInputController.h"
#include "QFMAHRS.h"
#include "QFMPositionController.h"
#include "QFMEngineController.h"

#include "QFMAttitudeController.generated.h"


/*--- Implementatrion of the Attitude and Position Flight-Controller ---*/
USTRUCT(BlueprintType)
struct FAttitudeController
{
	GENERATED_BODY()


	/*--- PARAMETERS ---*/

	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "QuadcopterFlightModel", meta = (ToolTip = "FlightMode")) 
	EFlightMode FlightMode = EFlightMode::FM_Direct;

	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "QuadcopterFlightModel", meta = (ToolTip = "Max Lean Angle Deg in Stab Mode")) 
	float AngleMax = 45.0f;  
	
	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "QuadcopterFlightModel", meta = (ToolTip = "Smoothing Value for StabilizerMode. Must be 0..1")) 
	float SmoothingGain = 0.5f;  
	
	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "QuadcopterFlightModel", meta = (ToolTip = "Zero Pos on Throttle for Accro Mode. Must be 0..1")) 
	float AccroThrottleMid = 0.5f;   
	
	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "QuadcopterFlightModel", meta = (ToolTip = "Max Speed Down in m/s in AltHold Mode")) 
	float PilotMaxSpeedDown = 10.0f;   
	
	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "QuadcopterFlightModel", meta = (ToolTip = "Max Speed Up in m/s in AltHold Mode")) 
	float PilotMaxSpeedUp = 7.0f;   
	
	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "QuadcopterFlightModel", meta = (ToolTip = "Max Speed Down in m/s in Stab/Accro Mode")) 
	float PilotSpeedDown = 5.0f;   
	
	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "QuadcopterFlightModel", meta = (ToolTip = "Max Speed Up in m/s in Stab/Accro Mode")) 
	float PilotSpeedUp = 5.0f;   

	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "QuadcopterFlightModel", meta = (ToolTip = "Max Speed Down in m/s in AltHold Mode")) 
	float DesiredVerticalSpeed = 3.0f;   
	
	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "QuadcopterFlightModel", meta = (ToolTip = "Desired Z Accel in Alt Hold Mode in m/s^2")) 
	float PilotZAccel = 1.5f;   
	
	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "QuadcopterFlightModel", meta = (ToolTip = "1..10 , 4.5 = 200 (deg/s) Max rotation rate of yaw axis")) 
	float YawPGain = 400.0f;   
	
	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "QuadcopterFlightModel", meta = (ToolTip = "1..10 , 4.5 = 200 (deg/s) Max rotation rate of roll/pitch axis")) 
	float AccroRollPitchPGain = 400.0f;   
	
	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "QuadcopterFlightModel", meta = (ToolTip = "-0.5..1 Amount of Expo to add to Accro Yaw 0 = disable, 1.0 = very high")) 
	float AccroYawExpo = 0.0f;   
	
	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "QuadcopterFlightModel", meta = (ToolTip = "-0.5..1 Amount of Expo to add to Accro Yaw 0 = disable, 1.0 = very high")) 
	float AccroRollPitchExpo = 0.0f;  
	
	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "QuadcopterFlightModel", meta = (ToolTip = "deadzone in % (0..1) up and % down from center")) 
	float ThrottleDeadzone = 0.1f;   

	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "QuadcopterFlightModel", meta = (ToolTip = "Roll Rate PID P"))
	float RateRollP = 0.002;//4.0f;//0.15f;

	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "QuadcopterFlightModel", meta = (ToolTip = "Roll Rate PID I"))
	float RateRollI = 0.002;//0.2f;//0.1f;
	
	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "QuadcopterFlightModel", meta = (ToolTip = "Roll Rate PID D"))
	float RateRollD = 0.003;//0.4f;//0.004f;
	
	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "QuadcopterFlightModel", meta = (ToolTip = "Pitch Rate PID P"))
	float RatePitchP = 0.002;//4.0f;//0.15f;

	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "QuadcopterFlightModel", meta = (ToolTip = "Pitch Rate PID I"))
	float RatePitchI = 0.002;//0.2f;//0.1f;

	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "QuadcopterFlightModel", meta = (ToolTip = "Pitch Rate PID D"))
	float RatePitchD = 0.003;//0.4f;//0.004f;

	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "QuadcopterFlightModel", meta = (ToolTip = "Yaw Rate PID P"))
	float RateYawP = 0.002;//4.0f;//0.02f;

	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "QuadcopterFlightModel", meta = (ToolTip = "Yaw Rate PID I"))
	float RateYawI = 0.002;//0.2f;//0.02f;

	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "QuadcopterFlightModel", meta = (ToolTip = "Yaw Rate PID D"))
	float RateYawD = 0.003;//0.4f;//0.0f;



	
	/*--- PRIVATE ---*/

	// Targets
	UPROPERTY() FQuat AttitudeTargetQuat;
	UPROPERTY() FVector AttitudeTargetAngVel;

	// PIDs
	UPROPERTY() FPIDController RateRollPid;
	UPROPERTY() FPIDController RatePitchPid;
	UPROPERTY() FPIDController RateYawPid;


	/*--- INTERFACE DATA ---*/
	// Copy of Parent Data. Put inside here during Tock or Init
	UPROPERTY()
	float DeltaTime;
	UPROPERTY()
	FVector4 PilotInput;

	FBodyInstance *BodyInstance;
	UPrimitiveComponent *PrimitiveComponent;
	
	FAHRS *AHRS;
	FPositionController *PositionController;
	FEngineController *EngineController;
	FInputController *InputController;
	
	/*--- UDP PID DEBUG ---*/
	FVector UDPDebugOutput;

	FVector GetUDPDebugOutput()
	{
		return UDPDebugOutput;
	}



	/*--- INIT FLIGHT MODES ---*/


	void Init(FBodyInstance *BodyInstanceIn, UPrimitiveComponent *PrimitiveComponentIn, FInputController *InputControllerIn, FAHRS *AHRSIn, FPositionController *PositionControllerIn, FEngineController *EngineControllerIn)
	{

		// Set up UDP Debug Output
		UDPDebugOutput = FVector::ZeroVector;

		// Set up Interface
	
		BodyInstance = BodyInstanceIn;
		PrimitiveComponent = PrimitiveComponentIn;
		InputController = InputControllerIn;
		AHRS = AHRSIn;
		PositionController = PositionControllerIn;
		EngineController = EngineControllerIn;

		// Init Pids with min,max = -1..1. We normalize velocities in RunQuat(). So we allways have the same PID-Settinghs, regardeless of Max Rates
		RateRollPid.Init(-1, 1, RateRollP, RateRollI, RateRollD);
		RatePitchPid.Init(-1, 1, RatePitchP, RatePitchI, RatePitchD);
		RateYawPid.Init(-1, 1, RateYawP, RateYawI, RateYawD);

		Reset();

		// we start in Stabilize mode
		SelectFlightMode(FlightMode);
	}


	void Reset()
	{
		// Reset Quats
		AttitudeTargetQuat = AHRS->GetWorldRotationQuat(); //		FQuat(FRotator(AHRS->BodyRotation.GetInverse())); // in degs
		AttitudeTargetAngVel = AHRS->GetBodyAngularVelocityVect(); // in deg/s
	
		// ResetPids
		RateRollPid.Reset();
		RatePitchPid.Reset();
		RateYawPid.Reset();
	}


	void SelectFlightMode(EFlightMode FlightModeIn)
	{
		FlightMode = FlightModeIn;

		switch (FlightMode)
		{
		case EFlightMode::FM_Direct:
			InitModeDirect();
				break;
		case EFlightMode::FM_Stabilize:
			InitModeStabilize();
				break;
		case EFlightMode::FM_AltHold:
			InitModeAltHold();
				break;
		case EFlightMode::FM_Accro:
			InitModeAccro();
				break;
		default:
			break;
		}
	}



	void InitModeDirect()
	{
		// Nothing to do here
	}

	void InitModeStabilize()
	{
		PositionController->SetAltTarget(0.0f);
	}


	void InitModeAltHold()
	{
		PositionController->SetMaxVelocityZ(-PilotMaxSpeedDown, PilotMaxSpeedUp);
		PositionController->SetMaxAccelerationZ(PilotZAccel);

		if (!PositionController->IsActiveZ())
		{
			PositionController->SetAltTargetToCurrentAlt();
			PositionController->SetDesiredVelocityZ(DesiredVerticalSpeed);
		}
	}


	void InitModeAccro()
	{
		PositionController->SetAltTarget(0.0f);
	}


	/*--- Tock Method. Call this from parents Tick() ---*/

	void Tock(float DeltaTimeIn)
	{
		DeltaTime = DeltaTimeIn;
		PilotInput = InputController->GetDesiredInput();

		switch (FlightMode)
		{
		case EFlightMode::FM_Direct:
			TockModeDirect();
				break;
		case EFlightMode::FM_Stabilize:
			TockModeStabilize();
				break;
		case EFlightMode::FM_AltHold:
			TockModeAltHold();
				break;
		case EFlightMode::FM_Accro:
			TockModeAccro();
				break;
		default:
			break;
		}
	}


	void TockModeDirect()
	{
		float ThrottleScaled;
		ThrottleScaled = GetPilotDesiredThrottle(PilotInput.W);
		EngineController->SetThrottleOut(ThrottleScaled);
		
		EngineController->SetRollRate(PilotInput.X);
		EngineController->SetPitchRate(PilotInput.Y);
		EngineController->SetYawRate(PilotInput.Z);
	}


	void TockModeStabilize()
	{
		float TargetRoll;
		float TargetPitch;
		float TargetYawRate;
		float ThrottleScaled;

		GetPilotDesiredLeanAngles(PilotInput.X, PilotInput.Y, TargetRoll, TargetPitch);
		TargetYawRate = GetPilotDesiredYawRate(PilotInput.Z);
		ThrottleScaled = GetPilotDesiredThrottle(PilotInput.W);

		InputAngleRollPitchRateYaw(TargetRoll, TargetPitch, TargetYawRate);
		EngineController->SetThrottleOut(ThrottleScaled);
	}


	void TockModeAltHold()
	{

		float TakeoffClimbRate = 0.0f;

		PositionController->SetMaxVelocityZ(-PilotMaxSpeedDown, PilotMaxSpeedUp);
		PositionController->SetMaxAccelerationZ(PilotZAccel);

		float TargetRoll;
		float TargetPitch;
		float TargetYawRate;
		float TargetClimbRate;

		GetPilotDesiredLeanAngles(PilotInput.X, PilotInput.Y, TargetRoll, TargetPitch);
		TargetYawRate = GetPilotDesiredYawRate(PilotInput.Z);
		TargetClimbRate = GetPilotDesiredClimbRate(PilotInput.W);
		TargetClimbRate = FMath::Clamp<float>(TargetClimbRate, -PilotMaxSpeedDown, PilotMaxSpeedUp);

		InputAngleRollPitchRateYaw(TargetRoll, TargetPitch, TargetYawRate);
		PositionController->SetAltTargetFromClimbRate(TargetClimbRate);
		PositionController->UpdateZController();

	}


	void TockModeAccro()
	{
		float TargetRollRate;
		float TargetPitchRate;
		float TargetYawRate;
		float ThrottleScaled;

		GetPilotDesiredAngleRates(PilotInput.X, PilotInput.Y, PilotInput.Z, TargetRollRate, TargetPitchRate, TargetYawRate);
		//TargetYawRate = GetPilotDesiredYawRate(PilotInput.Z);
		ThrottleScaled = GetPilotDesiredThrottle(PilotInput.W);

		InputRateBodyRollPitchYaw(TargetRollRate, TargetPitchRate, TargetYawRate);
		EngineController->SetThrottleOut(ThrottleScaled);
	}


	/*--- CALCULATE PILOTs DESIRE ---*/


	// GetPilotDesiredLeanAngles - transform pilot's roll or pitch input into a desired lean angle 
	// returns desired angles in degrees 
	void GetPilotDesiredLeanAngles(float RollIn, float PitchIn, float &RollOut, float &PitchOut)
	{
		// Limit AngleMax 
		AngleMax = FMath::Clamp<float>(AngleMax, 0.0f, 80.0f);

		// Circular limit Roll and Pitch Inputs 
		FVector2D CircularIn = FVector2D(RollIn, PitchIn);
		float TotalIn = CircularIn.Size();
		if (TotalIn > 1.0f)
		{
			float ratio = 1.0f / TotalIn;
			RollIn *= ratio;
			PitchIn *= ratio;
		}

		// scale RollIn, PitchIn to AngleMax range 
		RollIn *= AngleMax;
		PitchIn *= AngleMax;

		// return 
		RollOut = RollIn;
		PitchOut = PitchIn;
	}


	// GetPilotDesiredAngleRates - transform pilot's roll pitch and yaw input into a desired lean angle rates 
	// returns desired angle rates in degrees-per-second 
	void GetPilotDesiredAngleRates(float RollIn, float PitchIn, float YawIn, float &RollRateOut, float &PitchRateOut, float &YawRateOut)
	{
		//float RateLimit;
		float RollOut;
		float PitchOut;
		float YawOut;

		// Circular limit Roll and Pitch Inputs 
		FVector2D CircularIn = FVector2D(RollIn, PitchIn);
		float TotalIn = CircularIn.Size();
		if (TotalIn > 1.0f)
		{
			float ratio = 1.0f / TotalIn;
			RollIn *= ratio;
			PitchIn *= ratio;
		}

		// calculate roll, pitch rate requests 
		//if (AccroRollPitchExpo <= 0) { 
		//    RollOut = RollIn * AccroRollPitchPGain; 
		//    PitchOut = PitchIn * AccroRollPitchPGain; 
		//} else { 
		// expo variables 
		float RPIn3, RPOut;

		// range check expo 
		AccroRollPitchExpo = FMath::Clamp(AccroRollPitchExpo, -0.5f, 1.0f);

		// roll expo 
		RPIn3 = RollIn * RollIn * RollIn;
		RPOut = (AccroRollPitchExpo * RPIn3) + ((1.0f - AccroRollPitchExpo) * RollIn);
		RollOut = RPOut * AccroRollPitchPGain;

		// pich expo 
		RPIn3 = PitchIn * PitchIn * PitchIn;
		RPOut = (AccroRollPitchExpo * RPIn3) + ((1.0f - AccroRollPitchExpo) * PitchIn);
		PitchOut = RPOut * AccroRollPitchPGain;

		// calculate yaw rate request 
		YawOut = GetPilotDesiredYawRate(YawIn);

		RollRateOut = RollOut;
		PitchRateOut = PitchOut;
		YawRateOut = YawOut;

	}


	// GetPilotDesiredYawRate - transform pilot's yaw input into a desired yaw rate 
	// returns desired yaw rate in degrees per second 
	float GetPilotDesiredYawRate(float YawIn)
	{
		float YawRequest;

		// calculate yaw rate request 
		//if (AccroYawExpo <= 0) { 
		//    YawRequest = YawIn * YawPGain; 
		//} else { 
		// expo variables 
		float YawIn3, YawOut;

		// range check expo 
		AccroYawExpo = FMath::Clamp(AccroYawExpo, -0.5f, 1.0f);

		// yaw expo 
		YawIn3 = YawIn * YawIn * YawIn;
		YawOut = (AccroYawExpo * YawIn3) + ((1.0f - AccroYawExpo) * YawIn);
		YawRequest = YawOut * YawPGain;
		//} 
		// convert pilot input to the desired yaw rate 
		return YawRequest;
	}


	// GetPilotDesiredThrottle transform pilot's manual throttle input to make hover throttle mid stick 
	// used only for manual throttle modes 
	// returns throttle output 0 to 1 
	float GetPilotDesiredThrottle(float ThrottleIn)
	{
		
		float MidStick = InputController->GetThrottleMidStick();
		float ThrottleMidIn = EngineController->GetThrottleHover();

		// ensure reasonable throttle values 
		ThrottleIn = FMath::Clamp<float>(ThrottleIn, 0.0f, 1.0f);

		// calculate normalised throttle input 0..1 and mid = 0.5
		if (ThrottleIn < MidStick) {
			// below the deadband 
			ThrottleIn = ThrottleIn * 0.5f / MidStick;
		}
		else if (ThrottleIn > MidStick) {
			// above the deadband 
			ThrottleIn = 0.5f + (ThrottleIn - MidStick) * 0.5f / (1.0f - MidStick);
		}
		else {
			// must be in the deadband 
			ThrottleIn = 0.5f;
		}

		// Expo 
		float Expo = FMath::Clamp<float>(-(ThrottleMidIn - 0.5) / 0.375, -0.5f, 1.0f); // calculate the output throttle using the given expo function 
		float ThrottleOut = ThrottleIn * (1.0f - Expo) + Expo * ThrottleIn*ThrottleIn*ThrottleIn;

		return ThrottleOut;
	}



	// GetPilotDesiredClimbRate - transform pilot's throttle input to climb rate in m/s 
	// without any deadzone at the bottom 
	float GetPilotDesiredClimbRate(float ThrottleIn)
	{
		float DesiredRate;

		float MidStick = InputController->GetThrottleMidStick();

		// ensure a reasonable deadzone 
		ThrottleDeadzone = FMath::Clamp<float>(ThrottleDeadzone, 0.0f, 0.4f);

		float DeadbandTop = MidStick + ThrottleDeadzone;
		float DeadbandBottom = MidStick - ThrottleDeadzone;

		// ensure a reasonable throttle value 
		ThrottleIn = FMath::Clamp<float>(ThrottleIn, 0.0f, 1.0f);

		// check throttle is above, below or in the deadband 
		if (ThrottleIn < DeadbandBottom) {
			// below the deadband 
			DesiredRate = PilotSpeedDown * (ThrottleIn - DeadbandBottom) / DeadbandBottom;
		}
		else if (ThrottleIn > DeadbandTop) {
			// above the deadband 
			DesiredRate = PilotSpeedUp * (ThrottleIn - DeadbandTop) / (1.0f - DeadbandTop);
		}
		else {
			// must be in the deadband 
			DesiredRate = 0.0f;
		}

		return DesiredRate;
	}





	/*--- INPUT FUNCTIONS: INPUT DATA INTO FLIGHT CONTROLLER ---*/


	// Command an angular roll, pitch and rate yaw with angular velocity feedforward 
	void InputAngleRollPitchRateYaw(float RollIn, float PitchIn, float YawRateIn)
	{

		FRotator AttitudeTargetRotator = AttitudeTargetQuat.Rotator();
		AttitudeTargetRotator.Roll = RollIn;
		AttitudeTargetRotator.Pitch = PitchIn;
		AttitudeTargetRotator.Yaw += YawRateIn * DeltaTime;
		AttitudeTargetRotator = AttitudeTargetRotator.Clamp();

		// Compute quaternion target attitude for roll and pitch
		AttitudeTargetQuat = FQuat(AttitudeTargetRotator);

		// add rate yaw and fall thru
		//InputRateBodyRollPitchYaw(0.0f, 0.0f, YawRateIn);

		// Set rate feedforward requests to zero 
		AttitudeTargetAngVel = FVector(0.0f, 0.0f, 0.0f);

		// Call quaternion attitude controller
		RunQuat();
	}
	

	void InputRateBodyRollPitchYaw(float RollRateIn, float PitchRateIn, float YawRateIn)
	{
		FRotator RateRotator = FRotator(PitchRateIn * DeltaTime, YawRateIn * DeltaTime, RollRateIn*DeltaTime);
		
		RateRotator = RateRotator.Clamp();
		FQuat AttitudeTargetUpdateQuat = FQuat(RateRotator);
		
		AttitudeTargetQuat = AttitudeTargetQuat * AttitudeTargetUpdateQuat;
		AttitudeTargetQuat.Normalize();

		// Set rate feedforward requests to zero 
		AttitudeTargetAngVel = FVector(0.0f, 0.0f, 0.0f);

		// Call quaternion attitude controller
		RunQuat();
	}




	/* --- RUN QUAT --- */

    // Calculates the body frame angular velocities to follow the target attitude
    void RunQuat()
    {
/*
		// Get current Vehicle Attitude in World Space
		FTransform bodyTransform = PrimitiveComponent->GetComponentTransform();
		FQuat AttitudeVehicleQuat = bodyTransform.GetRotation();

		// Calculate Conjugate since we will use it more often
		FQuat AttitudeVehicleQuatConj = AttitudeVehicleQuat.Inverse();

		// Calculate Delta between current attitude and target attitude in Body Space
		FQuat DeltaQuat = AttitudeTargetQuat * AttitudeVehicleQuatConj;

		// for the velocity calculation use the shortest quaternion delta path
		FQuat DeltaQuatShortest = DeltaQuat;
		if ( (AttitudeTargetQuat | AttitudeVehicleQuat) < 0 ) // id dot product negative, take other solution
			DeltaQuatShortest = (AttitudeTargetQuat * -1) * AttitudeVehicleQuatConj;

		// calculate angular velocity between current attitude and target attitude in Body Space
		//FQuat VelocityQuat = ((DeltaQuatShortest) * 2.0f * DeltaTime) ;
		//FQuat VelocityQuat = ((DeltaQuatShortest) * 0.5f / DeltaTime) ;
		FQuat VelocityQuat = ((DeltaQuatShortest * 2) / DeltaTime);

		// ... and make a vector of it in Body Space in rad/s
		FVector VelocityToApply = FVector(VelocityQuat.X, VelocityQuat.Y, VelocityQuat.Z); 
		
		// Transform VelocityToApply into World Space
		VelocityToApply = bodyTransform.InverseTransformVector(VelocityToApply);

		// Get current angular Velocity in World Space in rad/s
		FVector BodyAngularVelocityVect = BodyInstance->GetUnrealWorldAngularVelocityInRadians();

		// Calculate ANgular Velocitry Target in World Space
		FVector BodyAngularVelocityVectTgt = BodyAngularVelocityVect + VelocityToApply;

		// Call the PID-Controllers to find Velocity Targets in rads 
		FVector PIDVelocityToApply = FVector (
			RateTargetToMotorRoll(BodyAngularVelocityVect.X, BodyAngularVelocityVectTgt.X),
			RateTargetToMotorPitch(BodyAngularVelocityVect.Y, BodyAngularVelocityVectTgt.Y),
			RateTargetToMotorYaw(BodyAngularVelocityVect.Z, BodyAngularVelocityVectTgt.Z)
		);
*/



		// Get current Vehicle Attitude in World Space
		//FTransform bodyTransform = PrimitiveComponent->GetComponentTransform();
		//FQuat AttitudeVehicleQuat = bodyTransform.GetRotation();
		FQuat AttitudeVehicleQuat = PrimitiveComponent->GetComponentQuat();

		// Calculate Conjugate since we will use it more often
		FQuat AttitudeVehicleQuatConj = AttitudeVehicleQuat.Inverse();

		// Calculate Delta between current attitude and target attitude (Is Relative, means: local space!)
		FQuat DeltaQuat = AttitudeTargetQuat * AttitudeVehicleQuatConj;

		// for the velocity calculation use the shortest quaternion delta path
		FQuat DeltaQuatShortest = DeltaQuat;
		if ( (AttitudeTargetQuat | AttitudeVehicleQuat) < 0 ) // id dot product negative, take other solution
			DeltaQuatShortest = (AttitudeTargetQuat * -1) * AttitudeVehicleQuatConj;
				
		// Normalize the Deltas
		DeltaQuat.Normalize();
		DeltaQuatShortest.Normalize();

		// calculate angular velocity between current attitude and target attitude in Local Space
		//FQuat VelocityQuat = ((DeltaQuatShortest / DeltaTime) * 2.0f);
		// ... and make a vector of it in Body Space in rad/s
		//FVector VelocityToApply = FVector(VelocityQuat.X, VelocityQuat.Y, VelocityQuat.Z); 
		
		// Alternative Method with better results:
		FVector VelocityToApply;
		FVector DeltaVectorShortest = FVector(DeltaQuatShortest.X, DeltaQuatShortest.Y, DeltaQuatShortest.Z);
		float Len=DeltaQuatShortest.Size();
		if(Len <= SMALL_NUMBER)
		{
			VelocityToApply = DeltaVectorShortest * 2.0f;
		}
		else
		{
			float Angle = 2 * FMath::Atan2(Len, DeltaQuatShortest.W);
			VelocityToApply = DeltaVectorShortest * (Angle / (Len * DeltaTime) );
		}
		
		
		// Transform VelocityToApply into World  Space
		//VelocityToApply = bodyTransform.InverseTransformVector(VelocityToApply);
		VelocityToApply = AttitudeVehicleQuat.UnrotateVector(VelocityToApply);
		
		// Get current angular Velocity in World Space in rad/s
		FVector BodyAngularVelocityVect = BodyInstance->GetUnrealWorldAngularVelocityInRadians();

		// Calculate ANgular Velocitry Target in World Space
		FVector BodyAngularVelocityVectTgt = BodyAngularVelocityVect + VelocityToApply;

		// Call the PID-Controllers to find Velocity Targets in rads 
		FVector PIDVelocityToApply = FVector (
			StepRateRollPid(BodyAngularVelocityVect.X, BodyAngularVelocityVectTgt.X),
			StepRatePitchPid(BodyAngularVelocityVect.Y, BodyAngularVelocityVectTgt.Y),
			StepRateYawPid(BodyAngularVelocityVect.Z, BodyAngularVelocityVectTgt.Z)
		);

		// Rotate Angular Velocity vector into Body Space
		FVector FinalVelocityToApply = AttitudeVehicleQuat.RotateVector(PIDVelocityToApply);
		
		// Send Calculated Roll Rates in rads to the Engine Controller
		//EngineController->SetRollRate(FinalVelocityToApply.X);
		//EngineController->SetPitchRate(FinalVelocityToApply.Y);
		//EngineController->SetYawRate(FinalVelocityToApply.Z);

		

		
		// This will be placed in EngineController and/or Simulate
		//FVector FinalTorque = FinalVelocityToApply; // Because of UE4s units of Torque, we could calibrate here...
		//BodyInstance->AddTorqueInRadians(FinalTorque, true, true);   
		//FHitResult Hit = FHitResult();
		//PrimitiveComponent->MoveComponent(FVector::ZeroVector, DeltaQuat * AttitudeVehicleQuat, false, &Hit, EMoveComponentFlags::MOVECOMP_NoFlags, ETeleportType::TeleportPhysics);
		PrimitiveComponent->SetPhysicsAngularVelocityInRadians(FinalVelocityToApply, true, NAME_None);
		
		//UE_LOG(LogTemp,Display,TEXT("V/PV %f, %f"), VelocityToApply.X, PIDVelocityToApply.X );
		//UDPDebugOutput = FVector(BodyAngularVelocityVect.X, BodyAngularVelocityVectTgt.X, PIDVelocityToApply.X);

    }



/*
How to calculate current angular velocity

FQuat Orientation = GetComponentQuat();
	FQuat DeltaQ = OldOrientation.Inverse() * Orientation;
	FVector Axis;
	float Angle;
	DeltaQ.ToAxisAndAngle(Axis, Angle);
	Angle = FMath::RadiansToDegrees(Angle);
	AngularVelocity = Orientation.RotateVector((Axis * Angle) / DeltaTime);
	OldOrientation = Orientation;

	*/



	// Run the roll angular velocity PID controller and return the output in rads
	float StepRateRollPid(float RateActualRads, float RateTargetRads) 
	{ 
		float RateActualNorm = RateActualRads / FMath::DegreesToRadians(AccroRollPitchPGain);
		float RateTargetNorm = RateTargetRads / FMath::DegreesToRadians(AccroRollPitchPGain);
		return RateRollPid.Calculate(RateTargetNorm, RateActualNorm, DeltaTime) * FMath::DegreesToRadians(AccroRollPitchPGain);
	} 

	// Run the pitch angular velocity PID controller and return the output in rads
	float StepRatePitchPid(float RateActualRads, float RateTargetRads) 
	{ 
		float RateActualNorm = RateActualRads / FMath::DegreesToRadians(AccroRollPitchPGain);
		float RateTargetNorm = RateTargetRads / FMath::DegreesToRadians(AccroRollPitchPGain);
		return RatePitchPid.Calculate(RateTargetNorm, RateActualNorm, DeltaTime) * FMath::DegreesToRadians(AccroRollPitchPGain);
	} 

	// Run the roll angular velocity PID controller and return the output in rads
	float StepRateYawPid(float RateActualRads, float RateTargetRads) 
	{ 
		float RateActualNorm = RateActualRads / FMath::DegreesToRadians(YawPGain);
		float RateTargetNorm = RateTargetRads / FMath::DegreesToRadians(YawPGain);
		return RateYawPid.Calculate(RateTargetNorm, RateActualNorm, DeltaTime) * FMath::DegreesToRadians(YawPGain);
	} 

	void UpdateThrottleRPYMix()
	{

	}


	void Debug(FColor ColorIn, FVector2D DebugFontSizeIn)
	{
	//	GEngine->AddOnScreenDebugMessage(-1, 0, ColorIn, FString::Printf(TEXT("Engines %%  : 1=%f 2=%f 3=%f 4=%f"), GetEnginePercent(0), GetEnginePercent(1), GetEnginePercent(2), GetEnginePercent(3)), true, DebugFontSizeIn);
		
	}


};


