#pragma once

#include "CoreMinimal.h"


#include "QFMTypes.generated.h"

typedef unsigned char 		uint8;		// 8-bit  unsigned.

/*--- Structure to hold our supported Flight Modes ---*/
UENUM(BlueprintType)                 
enum class EFlightMode : uint8
{
	FM_Direct		UMETA(DisplayName = "Direct Mode"),
	FM_Stabilize	UMETA(DisplayName = "Stabilize Mode"),
	FM_AltHold		UMETA(DisplayName = "Stabilize Mode with Alt Hold"),
	FM_Accro		UMETA(DisplayName = "Accro Mode")
};


/*--- Structure to hold Rate Stabilizers that are supported by AttitudeController  ---*/
UENUM(BlueprintType)                 
enum class EControlLoop : uint8
{
	ControlLoop_None			UMETA(DisplayName = "Dont use Stabilizer"),
	ControlLoop_PID			UMETA(DisplayName = "PID Controller"),
	ControlLoop_FPD			UMETA(DisplayName = "Forward PD Controller"),
	ControlLoop_SPD			UMETA(DisplayName = "Stable PD Controller")
};



// Enumeration of supported Frame Types
UENUM(BlueprintType)
enum class EFrameMode : uint8
{
    FrameModeCross 	UMETA(DisplayName="Cross"),
	FrameModePlus 	UMETA(DisplayName="Plus")
};



