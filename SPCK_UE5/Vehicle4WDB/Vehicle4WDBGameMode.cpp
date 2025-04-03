// Copyright Epic Games, Inc. All Rights Reserved.

#include "Vehicle4WDBGameMode.h"
#include "Vehicle4WDBCharacter.h"
#include "UObject/ConstructorHelpers.h"

AVehicle4WDBGameMode::AVehicle4WDBGameMode()
{
	// set default pawn class to our Blueprinted character
	static ConstructorHelpers::FClassFinder<APawn> PlayerPawnBPClass(TEXT("/Game/ThirdPerson/Blueprints/BP_ThirdPersonCharacter"));
	if (PlayerPawnBPClass.Class != NULL)
	{
		DefaultPawnClass = PlayerPawnBPClass.Class;
	}
}
