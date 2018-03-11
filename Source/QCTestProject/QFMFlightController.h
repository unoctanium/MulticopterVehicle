
//////////////////////////////

//////////////////////////////

//////////////////////////////
// WORK IN PROGRESS
//////////////////////////////

//////////////////////////////

//////////////////////////////

#pragma once


//#include "QFMPlugin.h"

#include "CoreMinimal.h"
#include "UObject/ObjectMacros.h"
#include "Engine.h"
#include "Components/SceneComponent.h"
#include "PhysicsEngine/BodyInstance.h"
#include "Math/Vector2D.h"
#include "Math/Vector4.h"

#include "QFMFlightController.generated.h"


/*--- Structure to hold our supported Flight Modes ---*/
UENUM(BlueprintType)                 
enum class EFlightMode : uint8
{
	FM_Direct		UMETA(DisplayName = "Direct Mode"),
	FM_Stabilize	UMETA(DisplayName = "Stabilize Mode"),
	FM_AltHold		UMETA(DisplayName = "Stabilize Mode with Alt Hold"),
	FM_Accro		UMETA(DisplayName = "Accro Mode")
};



/*--- Implementation of the AHRS ---*/
USTRUCT(BlueprintType)
struct FAHRS
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadOnly, meta = (ToolTip = "Position in m")) FVector Position = FVector(0.0f, 0.0f, 0.0f);
	UPROPERTY(EditAnywhere, BlueprintReadOnly, meta = (ToolTip = "Rotation in deg")) FRotator Rotation = FRotator(0.0f, 0.0f, 0.0f);
	UPROPERTY(EditAnywhere, BlueprintReadOnly, meta = (ToolTip = "LinearVelocity in m/s")) float LinearVelocity = 0.0f;
	UPROPERTY(EditAnywhere, BlueprintReadOnly, meta = (ToolTip = "Velocity Vector in m/s")) FVector VelocityVector = FVector(0.0f, 0.0f, 0.0f);
	UPROPERTY(EditAnywhere, BlueprintReadOnly, meta = (ToolTip = "LinearVelocity over Ground in m/s")) float LinearVelocity2D = 0.0f;
	UPROPERTY(EditAnywhere, BlueprintReadOnly, meta = (ToolTip = "Angular Velocity in deg/s")) FVector AngularVelocity = FVector(0.0f, 0.0f, 0.0f);
	UPROPERTY(EditAnywhere, BlueprintReadOnly, meta = (ToolTip = "Linear Acceleration in m/s^2")) float LinearAcceleration = 0.0f;
	UPROPERTY(EditAnywhere, BlueprintReadOnly, meta = (ToolTip = "Angular Acceleration in deg/s^2")) FVector AngularAcceleration = FVector(0.0f, 0.0f, 0.0f);



	// ODO: Need these for FlightController
	// Overthink old values above...
	UPROPERTY() FVector WorldPosition;
	UPROPERTY() FRotator BodyRotation; // World Rotation
	UPROPERTY() FVector BodyAngularVelocity;



	FRotator GetRotationDeg()
	{
		return Rotation;
	}

	FRotator GetRotationRad()
	{
		return FMath::DegreesToRadians<FRotator>(Rotation);
	}

	FVector GetAngularVelocityDeg()
	{
		return AngularVelocity;
	}

	FVector GetAngularVelocityRad()
	{
		//return FVector(FMath::DegreesToRadians(AngularVelocity.X), FMath::DegreesToRadians(AngularVelocity.Y), FMath::DegreesToRadians(AngularVelocity.Z));
		return FMath::DegreesToRadians<FVector>(AngularVelocity);
	}

	FVector GetAngularAccelerationDeg()
	{
		return AngularAcceleration;
	}

	FVector GetAngularAccelerationRad()
	{
		return FMath::DegreesToRadians<FVector>(AngularAcceleration);
	}


	void Debug(FColor Color, FVector2D DebugFontSize)
	{
		// Down to up on the debug screen
		GEngine->AddOnScreenDebugMessage(-1, 0, Color, FString::Printf(TEXT("Angular Acceleration (deg/s^2): X=%f Y=%f Z=%f"), AngularAcceleration.X, AngularAcceleration.Y, AngularAcceleration.Z), true, DebugFontSize);
		GEngine->AddOnScreenDebugMessage(-1, 0, Color, FString::Printf(TEXT("Linear Acceleration (m/s^2): %f"), LinearAcceleration), true, DebugFontSize);
		GEngine->AddOnScreenDebugMessage(-1, 0, Color, FString::Printf(TEXT("Angular Velocity (deg/s): X=%f Y=%f Z=%f"), AngularVelocity.X, AngularVelocity.Y, AngularVelocity.Z), true, DebugFontSize);
		GEngine->AddOnScreenDebugMessage(-1, 0, Color, FString::Printf(TEXT("Linear Velocity 2D (m/s): %f"), LinearVelocity2D), true, DebugFontSize);
		GEngine->AddOnScreenDebugMessage(-1, 0, Color, FString::Printf(TEXT("Linear Velocity Vector (m/s): X=%f Y=%f Z=%f"), VelocityVector.X, VelocityVector.Y, VelocityVector.Z), true, DebugFontSize);
		GEngine->AddOnScreenDebugMessage(-1, 0, Color, FString::Printf(TEXT("Linear Velocity (m/s): %f"), LinearVelocity), true, DebugFontSize);
		GEngine->AddOnScreenDebugMessage(-1, 0, Color, FString::Printf(TEXT("Rotation (deg): R=%f P=%f Y=%f"), Rotation.Roll, Rotation.Pitch, Rotation.Yaw), true, DebugFontSize);
		GEngine->AddOnScreenDebugMessage(-1, 0, Color, FString::Printf(TEXT("Position (m): X=%f Y=%f Z=%f"), Position.X, Position.Y, Position.Z), true, DebugFontSize);
	}


};







/*--- Implementation of the EngineController ---*/
USTRUCT(BlueprintType)
struct FEngineController
{
	GENERATED_BODY()

	UPROPERTY() float delete_me;
		
	// Return Hover Throttle in range 0..1
	// ODO: This comes from calculation ??? or a parameter ???
	float GetThrottleHover()
	{
		return 0.5;
	}

};





/*--- Implementation of the PID Controllers ---*/
USTRUCT(BlueprintType)
struct FPIDController
{
	GENERATED_BODY()

	UPROPERTY() float delete_me;


	void ResetI()
	{
		// TODO
	}

};








/*--- Implementatrion of the Attitude and Position Flight-Controller ---*/
USTRUCT(BlueprintType)
struct FFlightController
{
	GENERATED_BODY()


	/*--- PARAMETERS ---*/

	UPROPERTY() EFlightMode FlightMode;

	UPROPERTY() float AngleMax = 45.0f; // Max Lean Angle Deg in Stab Mode 
	UPROPERTY() float SmoothingGain = 0.5f; // Smoothing Value for StabilizerMode. Must be 0..1 
	UPROPERTY() float AccroThrottleMid = 0.5f; // Zero Pos on Throttle for Accro Mode. Must be 0..1 
	UPROPERTY() float PilotMaxSpeedDown = 10.0f; // Max Speed Down in m/s in AltHold Mode 
	UPROPERTY() float PilotMaxSpeedUp = 7.0f; // Max Speed Up in m/s in AltHold Mode 
	UPROPERTY() float PilotSpeedDown = 5.0f; // Max Speed Down in m/s in Stab/Accro Mode 
	UPROPERTY() float PilotSpeedUp = 5.0f; // Max Speed Up in m/s in Stab/Accro Mode 

	UPROPERTY() float DesiredVerticalSpeed = 3.0f; // Max Speed Down in m/s in AltHold Mode 
	UPROPERTY() float PilotZAccel = 1.5f; // Desired Z Accel in Alt Hold Mode in m/s^2 
	UPROPERTY() float YawPGain = 200.0f; // 1..10 , 4.5 = 200 (deg/s) Max rotation rate of yaw axis 
	UPROPERTY() float AccroRollPitchPGain = 200.0f; // 1..10 , 4.5 = 200 (deg/s) Max rotation rate of roll/pitch axis 
	UPROPERTY() float AccroYawExpo = 0.0f; // -0.5..1 Amount of Expo to add to Accro Yaw 0 = disable, 1.0 = very high 
	UPROPERTY() float AccroRollPitchExpo = 0.0f; // -0.5..1 Amount of Expo to add to Accro Yaw 0 = disable, 1.0 = very high 
	UPROPERTY() float ThrottleDeadzone = 0.1f; // deadzone in % (0..1) up and % down from center 


	
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
	// Copy of Parent Data. Put inside here during Tock 
	float DeltaTime;
	FBodyInstance *bodyInst;
	USceneComponent *rootComponent;
	FVector4 *PilotInput;
	
	
	
	/*--- INTERFACE DATA ---*/
	// Copy of Parent Data. Put inside here during Tock ??
	// Or shall I leave it like it is: as children of the Flight Controller?
	// ODO: TODO
	FEngineController EngineController;
	FAHRS AHRS;





	/*--- INIT FLIGHT MODES ---*/


	void Init(FBodyInstance *bodyInstIn, USceneComponent *rootComponentIn)
	{
		// Need these Vectors to read data from those components
		bodyInst = bodyInstIn;
		rootComponent = rootComponentIn;

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
		PosControlSetAltTarget(0.0f);
	}


	void InitModeAltHold()
	{
		PosControlSetVelocityZ(-PilotMaxSpeedDown, PilotMaxSpeedUp);
		PosControlSetAccelZ(PilotZAccel);

		if (!PosControlIsActiveZ())
		{
			PosControlSetAltTargetToCurrentAlt();
			PosControlSetDesiredVelocityZ(DesiredVerticalSpeed);
		}
	}


	void InitModeAccro()
	{
		PosControlSetAltTarget(0.0f);
	}



	/*--- Reset ---*/

	// Ensure attitude controller have zero errors to relax rate controller output
	void Relax()
	{

		AttitudeTargetQuat = FQuat(FRotator(AHRS.BodyRotation.GetInverse())); // in degs
		AttitudeTargetAngVel = AHRS.BodyAngularVelocity; // in deg/s
	
		//AttitudeTargetAngleRate = FRotator(); // ODO: HOW TO SET IT???
		// ????????

		AttitudeTargetRotator = AttitudeTargetQuat.Rotator();

		// Set reference angular velocity used in angular velocity controller equal
		// to the input angular velocity and reset the angular velocity integrators.
		// This zeros the output of the angular velocity controller.
		RateTargetAngVel = AHRS.BodyAngularVelocity;

		// ResetRateControllerITerms
		RateRollPid.ResetI();
		RatePitchPid.ResetI();
		RateYawPid.ResetI();

	}



	/*--- Tock Method. Call this from parents Tick() ---*/

	void Tock(float DeltaTimeIn, FVector4 *PilotInputIn)
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
		InputDirectRollPitchYaw(PilotInput->X, PilotInput->Y, PilotInput->Z);
		float ThrottleScaled = GetPilotDesiredThrottle(PilotInput->W);
		SetThrottleOut(ThrottleScaled, false);
	}

	void TockModeStabilize()
	{
		float TargetRoll;
		float TargetPitch;
		float TargetYawRate;
		float ThrottleScaled;

		GetPilotDesiredLeanAngles(PilotInput->X, PilotInput->Y, TargetRoll, TargetPitch);
		TargetYawRate = GetPilotDesiredYawRate(PilotInput->Z);
		ThrottleScaled = GetPilotDesiredThrottle(PilotInput->W);

		InputAngleRollPitchRateYaw(TargetRoll, TargetPitch, TargetYawRate);
		SetThrottleOut(ThrottleScaled, false);
	}


	void TockModeAltHold()
	{

		float TakeoffClimbRate = 0.0f;

		PosControlSetVelocityZ(-PilotMaxSpeedDown, PilotMaxSpeedUp);
		PosControlSetAccelZ(PilotZAccel);

		float TargetRoll;
		float TargetPitch;
		float TargetYawRate;
		float TargetClimbRate;

		GetPilotDesiredLeanAngles(PilotInput->X, PilotInput->Y, TargetRoll, TargetPitch);
		TargetYawRate = GetPilotDesiredYawRate(PilotInput->Z);
		TargetClimbRate = GetPilotDesiredClimbRate(PilotInput->W);
		TargetClimbRate = FMath::Clamp<float>(TargetClimbRate, -PilotMaxSpeedDown, PilotMaxSpeedUp);

		InputAngleRollPitchRateYaw(TargetRoll, TargetPitch, TargetYawRate);
		PosControlSetAltTargetFromClimbRate(TargetClimbRate);
		PosControlUpdateZController();

	}


	void TockModeAccro()
	{
		float TargetRollRate;
		float TargetPitchRate;
		float TargetYawRate;
		float ThrottleScaled;

		GetPilotDesiredAngleRates(PilotInput->X, PilotInput->Y, PilotInput->Z, TargetRollRate, TargetPitchRate, TargetYawRate);
		TargetYawRate = GetPilotDesiredYawRate(PilotInput->Z);
		ThrottleScaled = GetPilotDesiredThrottle(PilotInput->W);

		InputRateBodyRollPitchYaw(TargetRollRate, TargetPitchRate, TargetYawRate);
		SetThrottleOut(ThrottleScaled, false);
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
			ThrottleMidIn = EngineController.GetThrottleHover();
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

	void RunQuat()
	{
		// ODO: TODO
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

	float SetThrottleOut(float ThrottleIn, bool ResetAttitudeController = true)
	{
		// ODO:TODO
		return 1.0;
		
	}

	
	float GetThrottleHover()
	{
		
		// motors->get_throttle_hover(); 
		// ????????????????????????? 
		// Maybe this is from Engine Control???
		return 0.5;
	}




	/* --- POS CONTROLLER --- */


	void PosControlSetAltTarget(float AltIn)
	{
		//pos_control->set_alt_Target(f) 
	}


	void PosControlSetVelocityZ(float SpeedDownIn, float SpeedUpIn)
	{
		//sets maximum climb and descent rates 
		//pos_control->set_speed_z 
	}

	void PosControlSetAccelZ(float AccelIn)
	{
		//sets maximum climb and descent acceleration 
		//pos_control->set_accel_z 
	}

	bool PosControlIsActiveZ()
	{
		// pos_control->is_active_z() 
		return true;
	}

	void PosControlSetAltTargetToCurrentAlt()
	{
		//pos_control->set_alt_target_to_current_alt(); 
	}

	void PosControlSetDesiredVelocityZ(float VelocityIn)
	{
		//sets Desired climb/descent rate 
		//pos_control->set_desired_velocity_z 
	}

	void PosControlSetAltTargetFromClimbRate(float TargetClimbRate)
	{
		//set_alt_target_from_climb_rate_ff 
	}


	void PosControlUpdateZController()
	{
		//pos_control->update_z_controller(); 
	}


};


