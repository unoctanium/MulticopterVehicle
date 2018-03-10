
#pragma once

//#include "QFMPlugin.h"

#include "CoreMinimal.h"
#include "UObject/ObjectMacros.h"
#include "Engine.h"
//#include "Components/SceneComponent.h"
//#include "PhysicsEngine/BodyInstance.h"
#include "Math/Vector2D.h"
#include "Math/Vector4.h"

#include "QFMDebug.generated.h"

USTRUCT(BlueprintType)
struct FQuadcopterFlightModelDebugStruct
{
	GENERATED_BODY()


	UPROPERTY(BlueprintReadWrite, EditAnywhere, SaveGame, Category = "QuadcopterFlightModel", meta = (ToolTip = "Enable Screen Debug Output")) bool DebugScreen;
	UPROPERTY(BlueprintReadWrite, EditAnywhere, SaveGame, Category = "QuadcopterFlightModel", meta = (ToolTip = "Debug: Font Size")) FVector2D DebugFontSize;
	
	UPROPERTY(BlueprintReadWrite, EditAnywhere, SaveGame, Category = "QuadcopterFlightModel", meta = (ToolTip = "Debug: Display Pilot Input")) bool PrintInput;
	UPROPERTY(BlueprintReadWrite, EditAnywhere, SaveGame, Category = "QuadcopterFlightModel", meta = (ToolTip = "Debug: Display Vehicle Data")) bool PrintVehicle;
	UPROPERTY(BlueprintReadWrite, EditAnywhere, SaveGame, Category = "QuadcopterFlightModel", meta = (ToolTip = "Debug: Display Trajectory")) bool PrintTrajectory;
	UPROPERTY(BlueprintReadWrite, EditAnywhere, SaveGame, Category = "QuadcopterFlightModel", meta = (ToolTip = "Debug: Display Mixer Data")) bool PrintMixer;
	UPROPERTY(BlueprintReadWrite, EditAnywhere, SaveGame, Category = "QuadcopterFlightModel", meta = (ToolTip = "Debug: Display EngineControl Data")) bool PrintEngineControl;
	
    //Constructor
    FQuadcopterFlightModelDebugStruct()
    {
		DebugScreen = true;
		DebugFontSize = FVector2D(1.0f, 1.0f);
		PrintInput = true;
		PrintTrajectory = true;
		PrintVehicle = true;
		PrintMixer = true;
		PrintEngineControl = true;

    }

	void Debug(FColor Color, FVector2D DebugFontSize)
	{
		// Down to up on the debug screen
		//GEngine->AddOnScreenDebugMessage(-1, 0, Color, FString::Printf(TEXT("Center of Mass (m): X=%f Y=%f Z=%f"), CenterOfMass.X, CenterOfMass.Y, CenterOfMass.Z), true, DebugFontSize);
	}

};






