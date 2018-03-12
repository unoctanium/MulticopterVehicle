
#pragma once

#include "CoreMinimal.h"

#include "QFMDebug.generated.h"

USTRUCT(BlueprintType)
struct FQuadcopterFlightModelDebugStruct
{
	GENERATED_BODY()


	UPROPERTY(BlueprintReadWrite, EditAnywhere, SaveGame, Category = "QuadcopterFlightModel", meta = (ToolTip = "Enable Screen Debug Output")) bool DebugScreen = true;
	UPROPERTY(BlueprintReadWrite, EditAnywhere, SaveGame, Category = "QuadcopterFlightModel", meta = (ToolTip = "Debug: Font Size")) FVector2D DebugFontSize = FVector2D(1.0f, 1.0f);
	
	UPROPERTY(BlueprintReadWrite, EditAnywhere, SaveGame, Category = "QuadcopterFlightModel", meta = (ToolTip = "Debug: Display Pilot Input")) bool PrintInput = true;
	UPROPERTY(BlueprintReadWrite, EditAnywhere, SaveGame, Category = "QuadcopterFlightModel", meta = (ToolTip = "Debug: Display Vehicle Data")) bool PrintVehicle = true;
	UPROPERTY(BlueprintReadWrite, EditAnywhere, SaveGame, Category = "QuadcopterFlightModel", meta = (ToolTip = "Debug: Display Trajectory")) bool PrintTrajectory = true;
	UPROPERTY(BlueprintReadWrite, EditAnywhere, SaveGame, Category = "QuadcopterFlightModel", meta = (ToolTip = "Debug: Display Mixer Data")) bool PrintMixer = true;
	UPROPERTY(BlueprintReadWrite, EditAnywhere, SaveGame, Category = "QuadcopterFlightModel", meta = (ToolTip = "Debug: Display EngineControl Data")) bool PrintEngineControl = true;
	
    /*
    //Constructor
    FQuadcopterFlightModelDebugStruct()
    {
		DebugScreen = true;
		DebugFontSize = FVector2D(1.0f, 1.0f);
		PrintInput = true;
		PrintVehicle = true;
        PrintTrajectory = true;
		PrintMixer = true;
		PrintEngineControl = true;
    }
     */

	void Debug(FColor ColorIn, FVector2D DebugFontSizeIn)
	{
		// Down to up on the debug screen
		//GEngine->AddOnScreenDebugMessage(-1, 0, Color, FString::Printf(TEXT("Center of Mass (m): X=%f Y=%f Z=%f"), CenterOfMass.X, CenterOfMass.Y, CenterOfMass.Z), true, DebugFontSize);
	}

};






