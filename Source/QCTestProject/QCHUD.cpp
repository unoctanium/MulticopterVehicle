// Fill out your copyright notice in the Description page of Project Settings.

#include "QCHUD.h"

AQCHUD::AQCHUD () {

    /*
    // notice that at this point we can't guarantee that the playerController is actually constructed yet, so we can't get a reference to it
    // pay attention to next line! (select right path)
    static ConstructorHelpers::FClassFinder<UUserWidget> hudWidgetObj (TEXT ("/Game/QC/UI/Menu"));
    if (hudWidgetObj.Succeeded ()) {
        hudWidgetClass = hudWidgetObj.Class;
        UE_LOG(LogTemp, Display, TEXT("Bound UI"));
    } else {
        // hudWidgetObj not found
        hudWidgetClass = nullptr;
        UE_LOG(LogTemp, Error, TEXT("No UI"));
    }
    */
}

void AQCHUD::BeginPlay () {
    
    Super::BeginPlay ();

    /*
    if (hudWidgetClass) {
        // the player controller should be constructed by now so we can get a reference to it
        hudWidget = CreateWidget<UUserWidget> (this->GetOwningPlayerController (), this->hudWidgetClass);
        hudWidget->AddToViewport ();
    }
    */
}
     
