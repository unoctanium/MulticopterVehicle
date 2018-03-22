#pragma once

#include "CoreMinimal.h"
#include "Serialization/Archive.h"

#include "QFMUDPCustomData.generated.h"
 
USTRUCT(BlueprintType)
struct FUDPCustomData
{ 
	GENERATED_USTRUCT_BODY()
 
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Joy Color")
	FString Separator = FString(TEXT(","));

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="UDP Custom Data")
	float XValue = 1.f;
 
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="UDP Custom Data")
	float YValue = 1.f;

	FUDPCustomData()
	{}
};
 
FORCEINLINE FArchive& operator<<(FArchive &Ar, FUDPCustomData& TheStruct )
{

	Ar << TheStruct.XValue; 
    Ar << TheStruct.Separator;
	Ar << TheStruct.YValue;
 
	return Ar;
}