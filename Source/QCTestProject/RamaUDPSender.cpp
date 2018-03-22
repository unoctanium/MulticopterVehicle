/*
 
	RamaUDPSender
 
	by Rama
	customized by unoctanium
*/

#include "RamaUDPSender.h"

#include "QFMUDPCustomData.h"

ARamaUDPSender::ARamaUDPSender()
{	
	SenderSocket = NULL; 
	ShowOnScreenDebugMessages = true;
}
 
void ARamaUDPSender::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
	Super::EndPlay(EndPlayReason);
	//~~~~~~~~~~~~~~~~
 
	if(SenderSocket)
	{
		SenderSocket->Close();
		ISocketSubsystem::Get(PLATFORM_SOCKETSUBSYSTEM)->DestroySocket(SenderSocket);
	}
}
 
bool ARamaUDPSender::Start(
	const FString& YourChosenSocketName,
	const FString& TheIP, 
	const int32 ThePort
){	
	//Create Remote Address.
	RemoteAddr = ISocketSubsystem::Get(PLATFORM_SOCKETSUBSYSTEM)->CreateInternetAddr();
 
	bool bIsValid;
	RemoteAddr->SetIp(*TheIP, bIsValid);
	RemoteAddr->SetPort(ThePort);
 
	if(!bIsValid)
	{
		ScreenMsg("Rama UDP Sender>> IP address was not valid!", TheIP);
		return false;
	}
 
	SenderSocket = FUdpSocketBuilder(*YourChosenSocketName)
		.AsReusable()
		.WithBroadcast()
	;
 
 
	//check(SenderSocket->GetSocketType() == SOCKTYPE_Datagram);
 
	//Set Send Buffer Size
	int32 SendSize = 2*1024*1024;
	SenderSocket->SetSendBufferSize(SendSize,SendSize);
	SenderSocket->SetReceiveBufferSize(SendSize, SendSize);
 
	UE_LOG(LogTemp,Log,TEXT("\n\n\n~~~~~~~~~~~~~~~~~~~~~~~~~~~~~"));
	UE_LOG(LogTemp,Log,TEXT("Rama ****UDP**** Sender Initialized Successfully!!!"));
	UE_LOG(LogTemp,Log,TEXT("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n\n\n"));
 
	return true;
}
 
bool ARamaUDPSender::SendCustomData(void)
{
	if(!SenderSocket) 
	{
		ScreenMsg("No sender socket");
		return false;
	}
	//~~~~~~~~~~~~~~~~
 
	int32 BytesSent = 0;
 
	FUDPCustomData NewData;
	NewData.XValue = FMath::FRandRange(0,1000);
	NewData.YValue = FMath::RandRange(0,100);
 
	FArrayWriter Writer;
 
	Writer << NewData; //Serializing our custom data, thank you UE4!
 
	SenderSocket->SendTo(Writer.GetData(),Writer.Num(),BytesSent,*RemoteAddr);
 
	if(BytesSent <= 0)
	{
		const FString Str = "Socket is valid but the receiver received 0 bytes, make sure it is listening properly!";
		UE_LOG(LogTemp,Error,TEXT("%s"),*Str);
		ScreenMsg(Str);
		return false;
	}
 
	ScreenMsg("UDP~ Send Succcess! Bytes Sent = ",BytesSent );
 
	return true;
}