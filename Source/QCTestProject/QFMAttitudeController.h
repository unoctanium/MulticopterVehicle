
#pragma once

#include "CoreMinimal.h"
#include "UObject/ObjectMacros.h"

#include "Components/SceneComponent.h"
#include "PhysicsEngine/BodyInstance.h"

#include "QFMTypes.h"
#include "QFMPIDController.h"


#include "QFMPositionController.h"
#include "QFMEngineController.h"
#include "QFMAHRS.h"

#include "QFMAttitudeController.generated.h"



/*--- Implementatrion of the Attitude and Position Flight-Controller ---*/
USTRUCT(BlueprintType)
struct FAttitudeController
{
	GENERATED_BODY()


	/*--- PARAMETERS ---*/

	UPROPERTY() 
	EFlightMode FlightMode;

	UPROPERTY() 
	float AngleMax = 45.0f; // Max Lean Angle Deg in Stab Mode 
	
	UPROPERTY() 
	float SmoothingGain = 0.5f; // Smoothing Value for StabilizerMode. Must be 0..1 
	
	UPROPERTY() 
	float AccroThrottleMid = 0.5f; // Zero Pos on Throttle for Accro Mode. Must be 0..1 
	
	UPROPERTY() 
	float PilotMaxSpeedDown = 10.0f; // Max Speed Down in m/s in AltHold Mode 
	
	UPROPERTY() 
	float PilotMaxSpeedUp = 7.0f; // Max Speed Up in m/s in AltHold Mode 
	
	UPROPERTY() 
	float PilotSpeedDown = 5.0f; // Max Speed Down in m/s in Stab/Accro Mode 
	
	UPROPERTY() 
	float PilotSpeedUp = 5.0f; // Max Speed Up in m/s in Stab/Accro Mode 

	UPROPERTY() 
	float DesiredVerticalSpeed = 3.0f; // Max Speed Down in m/s in AltHold Mode 
	
	UPROPERTY() 
	float PilotZAccel = 1.5f; // Desired Z Accel in Alt Hold Mode in m/s^2 
	
	UPROPERTY() 
	float YawPGain = 200.0f; // 1..10 , 4.5 = 200 (deg/s) Max rotation rate of yaw axis 
	
	UPROPERTY() 
	float AccroRollPitchPGain = 200.0f; // 1..10 , 4.5 = 200 (deg/s) Max rotation rate of roll/pitch axis 
	
	UPROPERTY() 
	float AccroYawExpo = 0.0f; // -0.5..1 Amount of Expo to add to Accro Yaw 0 = disable, 1.0 = very high 
	
	UPROPERTY() 
	float AccroRollPitchExpo = 0.0f; // -0.5..1 Amount of Expo to add to Accro Yaw 0 = disable, 1.0 = very high 
	
	UPROPERTY() 
	float ThrottleDeadzone = 0.1f; // deadzone in % (0..1) up and % down from center 


	
	/*--- PRIVATE ---*/

	// Targets
	UPROPERTY() FQuat AttitudeTargetQuat;
	UPROPERTY() FRotator AttitudeTargetRotator;
	UPROPERTY() FVector AttitudeTargetAngVel;
	//UPROPERTY() FRotator AttitudeTargetAngleRate; // Do I need it?
	UPROPERTY() FVector RateTargetAngVel;

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
	
	/*--- INIT FLIGHT MODES ---*/


	void Init(FBodyInstance *BodyInstanceIn, UPrimitiveComponent *PrimitiveComponentIn, FAHRS *AHRSIn, FPositionController * PositionControllerIn, FEngineController *EngineControllerIn)
	{
		// Set up Interface
	
		BodyInstance = BodyInstanceIn;
		PrimitiveComponent = PrimitiveComponentIn;
		AHRS = AHRSIn;
		PositionController = PositionControllerIn;
		EngineController = EngineControllerIn;


		// set up the PID Controllers?
		// ??????

		// Reset PID parameters etc
		Relax();
		
		// we start in Stabilize mode
		SelectFlightMode(EFlightMode::FM_Stabilize);
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
		// Nothing to do 
	}


	void InitModeStabilize()
	{
		PositionController->SetAltTarget(0.0f);
	}


	void InitModeAltHold()
	{
		PositionController->SetVelocityZ(-PilotMaxSpeedDown, PilotMaxSpeedUp);
		PositionController->SetAccelZ(PilotZAccel);

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



	/*--- Reset ---*/

	// Ensure attitude controller have zero errors to relax rate controller output
	void Relax()
	{

		AttitudeTargetQuat = FQuat(FRotator(AHRS->BodyRotation.GetInverse())); // in degs
		AttitudeTargetAngVel = AHRS->BodyAngularVelocity; // in deg/s
	
		//AttitudeTargetAngleRate = FRotator(); // ODO: HOW TO SET IT???
		// ????????

		AttitudeTargetRotator = AttitudeTargetQuat.Rotator();

		// Set reference angular velocity used in angular velocity controller equal
		// to the input angular velocity and reset the angular velocity integrators.
		// This zeros the output of the angular velocity controller.
		RateTargetAngVel = AHRS->BodyAngularVelocity;

		// ResetRateControllerITerms
		RateRollPid.ResetI();
		RatePitchPid.ResetI();
		RateYawPid.ResetI();

	}



	/*--- Tock Method. Call this from parents Tick() ---*/

	void Tock(float DeltaTimeIn, FVector4 PilotInputIn)
	{

		DeltaTime = DeltaTimeIn;
		PilotInput = PilotInputIn;

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
		InputDirectRollPitchYaw(PilotInput.X, PilotInput.Y, PilotInput.Z);
		float ThrottleScaled = GetPilotDesiredThrottle(PilotInput.W);
		EngineController->SetThrottleOut(ThrottleScaled);
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

		PositionController->SetVelocityZ(-PilotMaxSpeedDown, PilotMaxSpeedUp);
		PositionController->SetAccelZ(PilotZAccel);

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
	void GetPilotDesiredAngleRates(float RollIn, float PitchIn, float YawIn, float &RollRateOut, float &PitchrateOut, float &YawRateOut)
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
	// ThrottleMidIn should be in the range 0 to 1 
	// returns throttle output 0 to 1 
	float GetPilotDesiredThrottle(float ThrottleIn, float ThrottleMidIn = 0.0f)
	{
		if (ThrottleMidIn <= 0.0f) {
			ThrottleMidIn = EngineController->GetThrottleHover();
		}

		float MidStick = GetThrottleMid();
		// protect against unlikely divide by zero 
		if (MidStick <= 0) {
			MidStick = 0.0f;
		}

		// ensure reasonable throttle values 
		ThrottleIn = FMath::Clamp<float>(ThrottleIn, 0.0f, 1.0f);

		// calculate normalised throttle input 
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
		float Expo = FMath::Clamp<float>(-(ThrottleMidIn - 0.5) / 0.375, -0.5f, 1.0f); // ODO: DAS geht besser 
																					   // calculate the output throttle using the given expo function 
		float ThrottleOut = ThrottleIn * (1.0f - Expo) + Expo * ThrottleIn*ThrottleIn*ThrottleIn;
		return ThrottleOut;
	}



	// GetPilotDesiredClimbRate - transform pilot's throttle input to climb rate in m/s 
	// without any deadzone at the bottom 
	float GetPilotDesiredClimbRate(float ThrottleIn)
	{
		float DesiredRate = 0.0f;
		float MidStick = GetThrottleMid();

		float DeadbandTop = MidStick + ThrottleDeadzone;
		float DeadbandBottom = MidStick - ThrottleDeadzone;

		// ensure a reasonable throttle value 
		ThrottleIn = FMath::Clamp<float>(ThrottleIn, 0.0f, 1.0f);

		// ensure a reasonable deadzone 
		ThrottleDeadzone = FMath::Clamp<float>(ThrottleDeadzone, 0.0f, 0.4f);

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


	// Command an angular roll, pitch and rate yaw with angular velocity feedforward and smoothing 
	void InputAngleRollPitchRateYaw(float RollIn, float PitchIn, float YawRateIn)
	{

		AttitudeTargetRotator.Roll = RollIn;
		AttitudeTargetRotator.Pitch = PitchIn;
		AttitudeTargetRotator.Yaw += YawRateIn * DeltaTime;
		AttitudeTargetRotator.Clamp();

		// Compute quaternion target attitude
		AttitudeTargetQuat = FQuat(AttitudeTargetRotator);

		// Set rate feedforward requests to zero 
		AttitudeTargetAngVel = FVector(0.0f, 0.0f, 0.0f);
		//AttitudeTargetAngRates = FRotator(0.0f, 0.0f, 0.0f);

		// Call quaternion attitude controller
		RunQuat();

	}
	

	void InputRateBodyRollPitchYaw(float RollRateIn, float PitchRateIn, float YawRateIn)
	{

		FQuat AttitudeTargetUpdateQuat;

		// calculate the attitude target euler angles 
		AttitudeTargetRotator = AttitudeTargetQuat.Rotator();

		FQuat AttituteTargetUpdateQuat;
		FVector fvect = FMath::DegreesToRadians(FVector(RollRateIn * DeltaTime, PitchRateIn * DeltaTime, YawRateIn * DeltaTime));
		float theta = fvect.Size();
		if (theta == 0.0f) {
			AttitudeTargetUpdateQuat = FQuat(1.0f, 0.0f, 0.0f, 0.0f);
		}
		else
		{
			fvect /= theta;
			AttituteTargetUpdateQuat = FQuat(fvect, theta);
		}

		AttitudeTargetQuat = AttitudeTargetQuat * AttitudeTargetUpdateQuat;
		AttitudeTargetQuat.Normalize();

		// Set rate feedforward requests to zero 
		AttitudeTargetAngVel = FVector(0.0f, 0.0f, 0.0f);
		//AttitudeTargetAngRates = FRotator(0.0f, 0.0f, 0.0f);

		// Call quaternion attitude controller
		RunQuat();

	}


	void InputDirectRollPitchYaw(float RollIn, float PitchIn, float YawIn)
	{
		// ODO: TODO
	}



	/* --- RUN QUAT --- */

    
    
    /*
    void USixDOFMovementComponent::CapsuleRotationUpdate(float DeltaTime, const FVector& TargetUpVector, bool bInstantRot, float RotationSpeed)
    {
        const FVector CapsuleUp = CapsuleComponent->GetUpVector();
        const FQuat DeltaQuat = FQuat::FindBetween(CapsuleUp, TargetUpVector);
        const FQuat TargetQuat = DeltaQuat * CapsuleComponent->GetComponentRotation().Quaternion();
        
        CurrentCapsuleQuat = bInstantRot ? TargetQuat : FQuat::Slerp(CapsuleComponent->GetComponentQuat(), TargetQuat, DeltaTime * RotationSpeed);
        
        FHitResult Hit(1.0f);
        SafeMoveUpdatedComponent(FVector::ZeroVector, CurrentCapsuleQuat, true, Hit);
        //    CapsuleComponent->SetWorldRotation(CurrentCapsuleQuat);
        
    }
    */
    
    // Calculates the body frame angular velocities to follow the target attitude
    void RunQuat()
    {

		FQuat AttitudeVehicleQuat = AHRS->GetAttitudeQuat();
        
        /*
        // Retrieve quaternion vehicle attitude
        // TODO add _ahrs.get_quaternion()
        FQuat AttitudeVehicleQuat;
        attitude_vehicle_quat.from_rotation_matrix(_ahrs.get_rotation_body_to_ned());
        
        // Compute attitude error
        Vector3f attitude_error_vector;
        thrust_heading_rotation_angles(_attitude_target_quat, attitude_vehicle_quat, attitude_error_vector, _thrust_error_angle);
        
        // Compute the angular velocity target from the attitude error
        _rate_target_ang_vel = update_ang_vel_target_from_att_error(attitude_error_vector);
        
        // Add feedforward term that attempts to ensure that roll and pitch errors rotate with the body frame rather than the reference frame.
        _rate_target_ang_vel.x += attitude_error_vector.y * _ahrs.get_gyro().z;
        _rate_target_ang_vel.y += -attitude_error_vector.x * _ahrs.get_gyro().z;
        
        // Add the angular velocity feedforward, rotated into vehicle frame
        Quaternion attitude_target_ang_vel_quat = Quaternion(0.0f, _attitude_target_ang_vel.x, _attitude_target_ang_vel.y, _attitude_target_ang_vel.z);
        Quaternion attitude_error_quat = attitude_vehicle_quat.inverse() * _attitude_target_quat;
        Quaternion target_ang_vel_quat = attitude_error_quat.inverse()*attitude_target_ang_vel_quat*attitude_error_quat;
        
        // Correct the thrust vector and smoothly add feedforward and yaw input
        if(_thrust_error_angle > AC_ATTITUDE_THRUST_ERROR_ANGLE*2.0f){
            _rate_target_ang_vel.z = _ahrs.get_gyro().z;
        }else if(_thrust_error_angle > AC_ATTITUDE_THRUST_ERROR_ANGLE){
            float flip_scalar = (1.0f - (_thrust_error_angle-AC_ATTITUDE_THRUST_ERROR_ANGLE)/AC_ATTITUDE_THRUST_ERROR_ANGLE);
            _rate_target_ang_vel.x += target_ang_vel_quat.q2*flip_scalar;
            _rate_target_ang_vel.y += target_ang_vel_quat.q3*flip_scalar;
            _rate_target_ang_vel.z += target_ang_vel_quat.q4;
            _rate_target_ang_vel.z = _ahrs.get_gyro().z*(1.0-flip_scalar) + _rate_target_ang_vel.z*flip_scalar;
        } else {
            _rate_target_ang_vel.x += target_ang_vel_quat.q2;
            _rate_target_ang_vel.y += target_ang_vel_quat.q3;
            _rate_target_ang_vel.z += target_ang_vel_quat.q4;
        }
        
        if (_rate_bf_ff_enabled & _use_ff_and_input_shaping) {
            // rotate target and normalize
            Quaternion attitude_target_update_quat;
            attitude_target_update_quat.from_axis_angle(Vector3f(_attitude_target_ang_vel.x * _dt, _attitude_target_ang_vel.y * _dt, _attitude_target_ang_vel.z * _dt));
            _attitude_target_quat = _attitude_target_quat * attitude_target_update_quat;
            _attitude_target_quat.normalize();
        }
         */
    }



	// Thrust and Rotation changes are calculated as two different rotations. 
	// First one rotates thrust (body Z) 
	// second one rotates Heading AFTER thrust rotation occured 
	void ThrustHeadingAndRotationAngles(FQuat &AttToQuat, FQuat &AttFromQuat, FVector &AttErrorAngles, float &AttErrorDot) 
	{ 
        // assumption: AttFromQuat and AttToQuat are in World AND Body Space, because they refer to a root object. 
        // This means: We do not need a body to world transform here 
        
        // Get From and To Thrust Vectors 
        FVector AttToThrustVector = AttToQuat.GetUpVector(); 
        FVector AttFromThrustVector = AttFromQuat.GetUpVector(); 
        
        // Find the Axis and Angle between those Thrust Vectors 
        FQuat ThrustVectorCorrectionQuat = FQuat::FindBetween(AttFromThrustVector, AttToThrustVector); 
        float ThrustCorrectionAngle = 0.0f; 
        FVector ThrustCorrectionAxis; 
        ThrustVectorCorrectionQuat.ToAxisAndAngle(ThrustCorrectionAxis, ThrustCorrectionAngle); 
        ThrustCorrectionAngle = FMath::RadiansToDegrees(ThrustCorrectionAngle); 
        
        // Apply Correction based on the initial rotation done by AttFromQuat 
        // ODO: I think we do not need this! 
        ThrustVectorCorrectionQuat = AttFromQuat.Inverse() * ThrustVectorCorrectionQuat * AttFromQuat; 
        
        // Calculate the remaining rotation required to correct the heading after thrust vector is rotated 
        // Means: Rotate around Z in a way that X (FORWARD) points to the requested X (FORWARD) direction 
        FQuat HeadingQuat = ThrustVectorCorrectionQuat * AttFromQuat.Inverse() * AttToQuat; 
        
	}






	/*--- THROTTLE FUNCTIONS ---*/


	// Helper Function
	float GetThrottleMid()
	{
		//get_throttle_mid() 
		// ????????????????????????? 
		// Maybe this is from Engine Control???
		return 0.5;
	}



	void Debug(FColor ColorIn, FVector2D DebugFontSizeIn)
	{
	//	GEngine->AddOnScreenDebugMessage(-1, 0, ColorIn, FString::Printf(TEXT("Engines %%  : 1=%f 2=%f 3=%f 4=%f"), GetEnginePercent(0), GetEnginePercent(1), GetEnginePercent(2), GetEnginePercent(3)), true, DebugFontSizeIn);
		
	}




};


