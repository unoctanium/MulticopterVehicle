#pragma once

#include "CoreMinimal.h"

#include "Components/SceneComponent.h"
#include "Components/PrimitiveComponent.h"
#include "PhysicsEngine/BodyInstance.h"

#include "QFMInputController.generated.h"

USTRUCT(BlueprintType)
struct FInputController
{
	GENERATED_BODY()

	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "QuadcopterPilotInput|AxisInput", meta = (ToolTip = "Stick: Roll Axis Input")) 
	float RollAxisInput = 0.0f;
	
	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "QuadcopterPilotInput|AxisInput", meta = (ToolTip = "Stick: Pitch Axis Input")) 
	float PitchAxisInput = 0.0f;
	
	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "QuadcopterPilotInput|AxisInput", meta = (ToolTip = "Stick: Yaw Axi Input")) 
	float YawAxisInput = 0.0f;
	
	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "QuadcopterPilotInput|AxisInput", meta = (ToolTip = "Stick: Throttle Axis Input")) 
	float ThrottleAxisInput = 0.0f;
	
	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "QuadcopterPilotInput|AxisSettings", meta = (ToolTip = "Stick: Roll Min/Max Values")) 
	FVector2D RollAxisInputInterval = FVector2D(-1.0f, 1.0f);
	
	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "QuadcopterPilotInput|AxisSettings", meta = (ToolTip = "Stick: Pitch Min/Max Values")) 
	FVector2D PitchAxisInputInterval = FVector2D(-1.0f, 1.0f);
	
	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "QuadcopterPilotInput|AxisSettings", meta = (ToolTip = "Stick: Yaw Min/Max Values")) 
	FVector2D YawAxisInputInterval = FVector2D(-1.0f, 1.0f);
	
	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "QuadcopterPilotInput|AxisSettings", meta = (ToolTip = "Stick: Throttle Min/Max Values")) 
	FVector2D ThrottleAxisInputInterval = FVector2D(-1.0f, 1.0f);
	
	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "QuadcopterPilotInput|AxisSettings", meta = (ToolTip = "Stick: Sacle Axis R,P,Y,T. Negative to invert")) 
	FVector4 InputAxisScale = FVector4(-1.0f, -1.0f, 1.0f, 1.0f);
	// R and P are inverted here, because this is the default of Xbox Controller.
	// So in the Project Settings we can leave all Inpus Axis to default


	// Desired Input
	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "QuadcopterPilotInput|AxisInput", meta = (ToolTip = "Resulting Desired PilotInput")) 
	FVector4 DesiredPilotInput = FVector4(0.0f, 0.0f, 0.0f, 0.0f);
	

	/*--- INTERFACE DATA ---*/
	// Copy of Parent Data. Put inside here during Init and Tock 
	UPROPERTY() 
	float DeltaTime;
	FBodyInstance *BodyInstance;
	UPrimitiveComponent *PrimitiveComponent;



	void Init(FBodyInstance *BodyInstanceIn, UPrimitiveComponent *PrimitiveComponentIn)
	{
		BodyInstance = BodyInstanceIn;
		PrimitiveComponent = PrimitiveComponentIn;
	}

	
	void Reset()
	{
		DesiredPilotInput = FVector4(0.0f, 0.0f, 0.0f, 0.0f);
	}

	
	void Tock(float DeltaTimeIn)
	{
		DeltaTime = DeltaTimeIn;

		// Convert Pilot Input to standard intervals (R,P,Y: -1..1, T 0..1)
		FVector4 NewInputStick = FVector4(
			GetMappedAndClampedValueNormal(RollAxisInputInterval, RollAxisInput * InputAxisScale.X),
			GetMappedAndClampedValueNormal(PitchAxisInputInterval, PitchAxisInput * InputAxisScale.Y),
			GetMappedAndClampedValueNormal(YawAxisInputInterval, YawAxisInput * InputAxisScale.Z),
			GetMappedAndClampedValueNormal(ThrottleAxisInputInterval, ThrottleAxisInput * InputAxisScale.W)
		);
		
		DesiredPilotInput = NewInputStick;

		// Normalize Throttle input to [0..1]
		DesiredPilotInput.W = DesiredPilotInput.W / 2.0f + 0.5f; 
	}



	FVector4 GetDesiredInput()
	{
		return DesiredPilotInput;
	}


	float GetThrottleMidStick()
	{
		return 0.5f; //return (ThrottleAxisInputInterval.X + (ThrottleAxisInputInterval.Y - ThrottleAxisInputInterval.X) / 2.0);
	}

	// Helper Function to Map Pilot Input to [-1..1]
	float GetMappedAndClampedValueNormal(const FVector2D& InputRange, const float Value)
	{
		return ( ( Value - InputRange.X ) / ( InputRange.Y - InputRange.X ) ) * 2.0f -1.0f;
		
	}

	// Helper Function to Map Pilot Input to [OutRange]
	float GetMappedAndClampedValue(const FVector2D& InputRange, const FVector2D& OutputRange, const float Value)
	{
		return ((Value - InputRange.X) / (InputRange.Y - InputRange.X)) * (OutputRange.Y - OutputRange.X) + OutputRange.X;
	}

	
	void Debug(FColor ColorIn, FVector2D DebugFontSizeIn)
	{
		//GEngine->AddOnScreenDebugMessage(-1, 0, ColorIn, TEXT("Pilot Input: (R,P,Y,T) ") + DesiredPilotInput.ToString(), true, DebugFontSizeIn);		
	}



};






