/*
	By Rama
	customized by unoctanium
*/
#pragma once
 
#include "CoreMinimal.h"
#include "Networking.h"
#include "Sockets.h"
#include "SocketSubsystem.h"
#include "Containers/UnrealString.h"
#include "GameFramework/Actor.h"
#include "UObject/UObjectGlobals.h"
#include "Serialization/Archive.h"


//Base
#include "RamaUDPSender.generated.h"
 
UCLASS()
class ARamaUDPSender : public AActor
{
	GENERATED_BODY()

public: 
	ARamaUDPSender();
 
	//UFUNCTION(BlueprintCallable, Category=RamaUDPSender)
	bool SendData(FString ToSend);
 
	TSharedPtr<FInternetAddr>	RemoteAddr;
	FSocket* SenderSocket;
 
	bool Start(
		const FString& YourChosenSocketName,
		const FString& TheIP, 
		const int32 ThePort
	);
 
public:
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Rama UDP Sender")
	bool ShowOnScreenDebugMessages;
 
 
	//ScreenMsg
	FORCEINLINE void ScreenMsg(const FString& Msg)
	{
		if(!ShowOnScreenDebugMessages) return;
		GEngine->AddOnScreenDebugMessage(-1, 5.f, FColor::Red, *Msg);
	}
	FORCEINLINE void ScreenMsg(const FString& Msg, const float Value)
	{
		if(!ShowOnScreenDebugMessages) return;
		GEngine->AddOnScreenDebugMessage(-1, 5.f, FColor::Red, FString::Printf(TEXT("%s %f"), *Msg, Value));
	}
	FORCEINLINE void ScreenMsg(const FString& Msg, const FString& Msg2)
	{
		if(!ShowOnScreenDebugMessages) return;
		GEngine->AddOnScreenDebugMessage(-1, 5.f, FColor::Red, FString::Printf(TEXT("%s %s"), *Msg, *Msg2));
	}
 
 
public:
 
	/** Called whenever this actor is being removed from a level */
	virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;
};
