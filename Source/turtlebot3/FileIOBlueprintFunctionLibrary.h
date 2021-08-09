// Copyright (C) Rapyuta Robotics

#pragma once

#include "CoreMinimal.h"
#include "Kismet/BlueprintFunctionLibrary.h"
#include <Runtime/Core/Public/Misc/FileHelper.h>
#include "FileIOBlueprintFunctionLibrary.generated.h"

/**
 * 
 */
UCLASS()
class TURTLEBOT3_API UFileIOBlueprintFunctionLibrary : public UBlueprintFunctionLibrary
{
	GENERATED_BODY()
	
	UFUNCTION(BlueprintCallable, Category="File I/O")
	static FString LoadFileToString(FString Filename);
	
	// UFUNCTION(BlueprintCallable, Category="File I/O")
	// static TArray<FString> LoadFileToStringArray(FString Filename);
	
	// UFUNCTION(BlueprintCallable, Category = "File I/O")
    // static bool FileSaveString(FString StringToSave, FString Filename);
};
