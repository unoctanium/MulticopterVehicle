
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

	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "QuadcopterFlightModel", meta = (ToolTip = "Stabilizer-Loop to use for Rotations")) 
	EControlLoop RotationControlLoop = EControlLoop::ControlLoop_FPD;

	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "QuadcopterFlightModel", meta = (ToolTip = "RPY SPD Damping. 1=crit damped, <1 = underdamped, >1 = overdamped"))
	FVector SPDDamping = FVector(1.0f, 1.0f, 1.0f);
	
	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "QuadcopterFlightModel", meta = (ToolTip = "RPY SPD Frequency. Reach 95% of target in 1/Freq secs"))
	FVector SPDFrequency = FVector(0.6f, 0.6f, 0.6f);


	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "QuadcopterFlightModel", meta = (ToolTip = "FPD Damping. 1=crit damped, <1 = underdamped, >1 = overdamped"))
	float FPDDamping = 5.0f;	
	
	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "QuadcopterFlightModel", meta = (ToolTip = "FPD Frequency. Reach 95% of target in 1/Freq secs"))
	float FPDFrequency = 0.1f;
	

	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "QuadcopterFlightModel", meta = (ToolTip = "RPY Rate PID P"))
	FVector RatePidP = FVector(0.001, 0.001, 0.001);

	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "QuadcopterFlightModel", meta = (ToolTip = "RPY Rate PID I"))
	FVector RatePidI = FVector(0.0005, 0.0005, 0.0005);

	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "QuadcopterFlightModel", meta = (ToolTip = "RPY Rate PID D"))
	FVector RatePidD = FVector(0.1, 0.1, 0.1);


	
	/*--- PRIVATE ---*/

	// Targets
	UPROPERTY() FQuat AttitudeTargetQuat;

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
		RateRollPid.Init(-2, 2, RatePidP.X, RatePidI.X, RatePidD.X);
		RatePitchPid.Init(-2, 2, RatePidP.Y, RatePidI.Y, RatePidD.Y);
		RateYawPid.Init(-2, 2, RatePidP.Z, RatePidI.Z, RatePidD.Z);

		Reset();

		// we start in Stabilize mode
		SelectFlightMode(FlightMode);
	}


	void Reset()
	{
		// Reset Quats
		FTransform bodyTransform =  BodyInstance->GetUnrealWorldTransform();
		AttitudeTargetQuat = bodyTransform.GetRotation();
	
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
		EngineController->SetRotationRates(PilotInput);
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

		// expo variables 
		float YawIn3, YawOut;

		// range check expo 
		AccroYawExpo = FMath::Clamp(AccroYawExpo, -0.5f, 1.0f);

		// yaw expo 
		YawIn3 = YawIn * YawIn * YawIn;
		YawOut = (AccroYawExpo * YawIn3) + ((1.0f - AccroYawExpo) * YawIn);
		YawRequest = YawOut * YawPGain;
	
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
		AttitudeTargetQuat.Normalize();

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

		// Call quaternion attitude controller
		RunQuat();
	}


	/* --- RUN QUAT --- */

	void RunQuat()
	{
		FTransform bodyTransform =  BodyInstance->GetUnrealWorldTransform();
		FQuat AttitudeVehicleQuat = bodyTransform.GetRotation();
		// FQuat AttitudeTargetQuat ist the desired rotation

		//q will rotate from our current rotation to desired rotation 
		float Direction = ((AttitudeTargetQuat | AttitudeVehicleQuat) >= 0) ? 1.0f : -1.0f;
		FQuat DeltaQuat  = (AttitudeTargetQuat * Direction) * AttitudeVehicleQuat.Inverse(); 



		// I must inspect this strange behaviour (slow rotation) if I just use 
		// AngularVelocityTgt from above with PID Loop and Stabilize Mode active
		// Until then, I use this code to correct everything
		// !!!THIS IS DISFUNCTIONAL
		if (RotationControlLoop == EControlLoop::ControlLoop_PID && FlightMode == EFlightMode::FM_Stabilize)
		{
			FVector AttFromThrustVector = AttitudeVehicleQuat.GetUpVector(); 
			FVector AttToThrustVector = AttitudeTargetQuat.GetUpVector(); 
         	FQuat ThrustVectorCorrectionQuat = FQuat::FindBetween(AttFromThrustVector, AttToThrustVector); 
			ThrustVectorCorrectionQuat = ThrustVectorCorrectionQuat * AttitudeVehicleQuat; 
			DeltaQuat = ThrustVectorCorrectionQuat;
		}



		DeltaQuat.Normalize();
		
		//convert to angle axis representation so we can do math with angular velocity 
		FVector Axis = FVector::ZeroVector;
		float Angle = 0.0f;
		DeltaQuat.ToAxisAndAngle(Axis, Angle); 
		Axis.Normalize();
		
		// Get current angular Velocity in World Space in rad/s
        FVector AngularVelocityNow = BodyInstance->GetUnrealWorldAngularVelocityInRadians();

		// AngularVelocityTgt is the w we need to achieve 
		FVector AngularVelocityTgt = Axis * Angle / DeltaTime; 

		// AngularVelocityToApply is the w we need to Apply to physx directly or after torque calculation
		FVector AngularVelocityToApply = FVector::ZeroVector;
		
		// AngularVelocityToApply depends on the choosen ControlLoop
		if(RotationControlLoop == EControlLoop::ControlLoop_None)
		{
			// For Option a) Calculate the (raw) Velocity we have to apply by taking into account, that we have Velocity allready
			AngularVelocityToApply = AngularVelocityTgt - AngularVelocityNow;
		}
		else if (RotationControlLoop == EControlLoop::ControlLoop_PID)
		{
			// For Option b) Run the PID-Controllers to find PID Angular Velocity to Apply in rads 
			AngularVelocityToApply = FVector (
            StepRateRollPid(AngularVelocityNow.X, AngularVelocityTgt.X),
            StepRatePitchPid(AngularVelocityNow.Y, AngularVelocityTgt.Y),
            StepRateYawPid(AngularVelocityNow.Z, AngularVelocityTgt.Z)
        );
		}
		else if (RotationControlLoop == EControlLoop::ControlLoop_FPD)
		{
			// For Option c) Run the FPD-Controllers to find DPD Angular Velocity to Apply in rads 
    	    AngularVelocityToApply = StepRateRollFpd( AngularVelocityTgt, AngularVelocityNow);
		}

		// We clamp Angular Velocity to the requested max
		float MaxRPVelocityRad = FMath::DegreesToRadians(AccroRollPitchPGain);
		float MaxYVelocityRad = FMath::DegreesToRadians(YawPGain);
		AngularVelocityToApply.X = FMath::Clamp(AngularVelocityToApply.X, -MaxRPVelocityRad, MaxRPVelocityRad);
		AngularVelocityToApply.Y = FMath::Clamp(AngularVelocityToApply.Y, -MaxRPVelocityRad, MaxRPVelocityRad);
		AngularVelocityToApply.Z = FMath::Clamp(AngularVelocityToApply.Z, -MaxYVelocityRad, MaxYVelocityRad);

/*
		// OPTION #0: This is going to be the future code here. Options 1..3 will move to EngineController.h or Simulate.h
		// Send Calculated Roll Rates in rads to the Engine Controller
        EngineController->SetRotationRates(AngularVelocityToApply);
*/

/*
		// OPTION #1: Set Velocity in Physx directy (not recommended). Use only withot StabilizerLoop (RotationControlLoop = EControlLoop::ControlLoop_None)
		PrimitiveComponent->SetPhysicsAngularVelocityInRadians(AngularVelocityToApply, true, NAME_None);
*/

///*
		// OPTION #2: Simulate Torque by Velocity-Change in rads, dont care about Inertia, mass etc.
		BodyInstance->AddTorqueInRadians(AngularVelocityToApply, true, true);  
//*/
/*
		// OPTION #3: Apply Torque force from Velocity-Change in rads 
		// to multiply with inertia tensor local then rotationTensor coords 
		FVector AngularVelocityLocal = bodyTransform.InverseTransformVectorNoScale(AngularVelocityToApply);  // a) raw
		FQuat InertiaTensorRotation = BodyInstance->GetMassSpaceToWorldSpace().GetRotation(); 
		FVector AngularVelocityLocalInertia = InertiaTensorRotation * AngularVelocityLocal; 
		AngularVelocityLocalInertia *= BodyInstance->GetBodyInertiaTensor(); 
		FVector TorqueLocal = InertiaTensorRotation.Inverse() * AngularVelocityLocalInertia; 
		FVector TorqueWorld = bodyTransform.TransformVectorNoScale(TorqueLocal); 
		BodyInstance->AddTorqueInRadians(TorqueWorld, true, false);  
*/	

/*
		// I must inspect this strange behaviour (slow rotation) if I just use 
		// AngularVelocityTgt from above with PID Loop and Stabilize Mode active
		// Until then, I use this code to correct everything
		// !!!THIS IS DISFUNCTIONAL
		if (RotationControlLoop == EControlLoop::ControlLoop_PID && FlightMode == EFlightMode::FM_Stabilize)
		{
			FVector FinalLocalTorque = PrimitiveComponent->GetComponentQuat().RotateVector(AngularVelocityToApply);	
			FinalLocalTorque *= 2000000;
			BodyInstance->AddTorqueInRadians(FinalLocalTorque, false, false);   
		}			
*/



	}

	// Run the rotational angular velocity FPD controller and return the output detla w in rads
	FVector StepRateRollFpd(FVector DeltaQ, FVector DeltaQDot)
	{
		float kp = FPDFrequency * FPDFrequency * 9.0f; 
		float kd = 4.5f * FPDFrequency * FPDDamping; 
		float dt = DeltaTime; 
		
		float g = 1.0f / (1.0f + kd * dt + kp * dt * dt); 
		float kpg = kp * g; 
		float kdg = (kd + kp * dt) * g; 

		//return FVector(kp * DeltaQ - kd * DeltaQDot);
		return FVector(kpg * DeltaQ - kdg * DeltaQDot);
	}


	// Run the rotational angular velocity SPD controller and return the output delta w in rads
	FVector StepRateRollSpd(FVector CurrentAttitude, FVector CurrentVelocity, FVector TargetAttitude, FVector InertiaTensor)
	{
		FVector kp = SPDFrequency * SPDFrequency * 9.0f; 
		FVector kd = 4.5f * SPDFrequency * SPDDamping; 
		float dt = DeltaTime; 
		FVector I = InertiaTensor;
		FVector g = FVector (
			1.0f / (I.X + kd.X * dt), 
			1.0f / (I.Y + kd.Y * dt), 
			1.0f / (I.Z + kd.Z * dt) 
		);
		FVector kpg = -g * kp; 
		FVector kdg = -g * kd;

		return FVector(kpg * (CurrentAttitude + CurrentVelocity*dt - TargetAttitude) + kdg * CurrentVelocity) *dt; // dt nötig hier??)
	}



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

	void Debug(FColor ColorIn, FVector2D DebugFontSizeIn)
	{
	//	GEngine->AddOnScreenDebugMessage(-1, 0, ColorIn, FString::Printf(TEXT("Engines %%  : 1=%f 2=%f 3=%f 4=%f"), GetEnginePercent(0), GetEnginePercent(1), GetEnginePercent(2), GetEnginePercent(3)), true, DebugFontSizeIn);
		
	}


};


