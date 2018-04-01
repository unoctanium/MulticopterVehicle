// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/HUD.h"

#include "Runtime/UMG/Public/UMG.h"
#include "Runtime/UMG/Public/UMGStyle.h"
#include "Runtime/UMG/Public/Slate/SObjectWidget.h"
#include "Runtime/UMG/Public/IUMGModule.h"
#include "Runtime/UMG/Public/Blueprint/UserWidget.h"
#include "Blueprint/UserWidget.h"

#include "QCHUD.generated.h"

/**
 * 
 */
UCLASS()
class QCTESTPROJECT_API AQCHUD : public AHUD
{
	GENERATED_BODY()

	virtual void BeginPlay () override;

public:

	AQCHUD();
	
protected:
/*
	UPROPERTY()
	class UClass * hudWidgetClass;

	UPROPERTY()
    class UUserWidget * hudWidget;
*/
	
};
