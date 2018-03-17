
#pragma once

#include "CoreMinimal.h"
#include "Math/Color.h"

#include "QFMDebug.generated.h"

USTRUCT(BlueprintType)
struct FQuadcopterFlightModelDebugStruct
{
	GENERATED_BODY()


	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "QuadcopterFlightModel", meta = (ToolTip = "Enable Screen Debug Output")) 
	bool DebugScreen = true;
	
	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "QuadcopterFlightModel", meta = (ToolTip = "Debug: Font Size")) 
	FVector2D FontSize = FVector2D(1.0f, 1.0f);
	
	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "QuadcopterFlightModel", meta = (ToolTip = "Debug: Font Color")) 
	FColor Color = FColor::White;

	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "QuadcopterFlightModel", meta = (ToolTip = "Debug: Display Pilot Input")) 
	bool PrintInput = true;
	
	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "QuadcopterFlightModel", meta = (ToolTip = "Debug: Display Vehicle Data")) 
	bool PrintVehicle = true;
	
	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "QuadcopterFlightModel", meta = (ToolTip = "Debug: Display AHRS")) 
	bool PrintAHRS = true;
	
	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "QuadcopterFlightModel", meta = (ToolTip = "Debug: Display AttitudeControl Data")) 
	bool PrintAttitudeControl = true;
	
	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "QuadcopterFlightModel", meta = (ToolTip = "Debug: Display PositionControl Data")) 
	bool PrintPositionControl = true;

	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "QuadcopterFlightModel", meta = (ToolTip = "Debug: Display EngineControl Data")) 
	bool PrintEngineControl = true;
	
    
	/*
	void Debug(FColor ColorIn, FVector2D DebugFontSizeIn)
	{
		// Down to up on the debug screen
		//GEngine->AddOnScreenDebugMessage(-1, 0, Color, FString::Printf(TEXT("Center of Mass (m): X=%f Y=%f Z=%f"), CenterOfMass.X, CenterOfMass.Y, CenterOfMass.Z), true, DebugFontSize);
	}
	*/


};






