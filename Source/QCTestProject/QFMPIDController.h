#pragma once

#include "CoreMinimal.h"

#include "QFMPIDController.generated.h"


/*--- Implementation of the PID Controllers ---*/
USTRUCT(BlueprintType)
struct FPIDController
{
	GENERATED_BODY()


	UPROPERTY() 
	float Max;

	UPROPERTY() 
    float Min;

    UPROPERTY() 
	float Kp;

	UPROPERTY() 
	float Ki;

	UPROPERTY() 
	float Kd;

	UPROPERTY() 
	float PreError;

	UPROPERTY() 
	float Integral;


	void Init(float MinIn, float MaxIn, float KpIn, float KiIn, float KdIn)
	{
		Min = MinIn;
		Max = MaxIn;
		Kp = KpIn;
		Ki = KiIn;
		Kd = KdIn;
		Integral = 0.0f;
		PreError = 0.0f;
	}

	
	void Reset()
	{
		Integral = 0.0f;
		PreError = 0.0f;
	}

	void ResetI()
	{
		Integral = 0.0f;
	}


	float Calculate(float Setpoint, float PresentValue, float DeltaTime)
	{
		
		// Calculate error
		float Error = Setpoint - PresentValue;

		// Proportional term
		float POut = Kp * Error;

		// Integral term
		Integral += Error * DeltaTime;
		float IOut = Ki * Integral;

		// Derivative term
		float Derivative = (Error - PreError) / DeltaTime;
		float DOut = Kd * Derivative;

		// Calculate total output
		float Output = POut + IOut + DOut;

		// Restrict to max/min
		if( Output > Max )
			Output = Max;
		else if( Output < Min )
			Output = Min;

		// Save error to previous error
		PreError = Error;

		return Output;


	}


	void Debug(FColor ColorIn, FVector2D DebugFontSizeIn)
	{
		// Down to up on the debug screen
		
	}

};






