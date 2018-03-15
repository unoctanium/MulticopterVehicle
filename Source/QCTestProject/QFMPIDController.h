#pragma once

#include "CoreMinimal.h"

#include "QFMPIDController.generated.h"




/*--- Implementation of the PID Controllers ---*/
USTRUCT(BlueprintType)
struct FPIDController
{
	GENERATED_BODY()


	UPROPERTY() 
	float DeltaTime;

	void Init()
	{
	}

	
	void Reset()
	{
	}

	
	void Tock(float DeltaTimeIn)
	{
		DeltaTime = DeltaTimeIn;
	}


	
	void Debug(FColor ColorIn, FVector2D DebugFontSizeIn)
	{
		// Down to up on the debug screen
		
	}



	void ResetI()
	{
		// TODO
	}

};






