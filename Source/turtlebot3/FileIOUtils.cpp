// Copyright (C) Rapyuta Robotics
#include "FileIOUtils.h"

// UE
#include <Runtime/Core/Public/HAL/PlatformFilemanager.h>
#include <Runtime/Core/Public/Misc/Paths.h>

// Turtlebot3_UE
#include "Turtlebot3.h"

bool UFileIOUtils::LoadFileToString(const FString& InFileName, FString& OutFileContent)
{
    FString ContentDirectory = FPaths::ProjectContentDir();
    IPlatformFile& File = FPlatformFileManager::Get().GetPlatformFile();

    if (File.CreateDirectory(*ContentDirectory))
    {
        FString TargetFileName = ContentDirectory / InFileName;
        return FFileHelper::LoadFileToString(OutFileContent, *TargetFileName);
    }
    else
    {
        UE_LOG(LogTurtlebot3, Error, TEXT("Dir does not exist or failed being created: [%s]"), *ContentDirectory);
        return false;
    }
}

// TArray<FString> UFileIOUtils::LoadFileToStringArray(const FString& InFileName)
// {
//     FString Directory = FPaths::ProjectContentDir();
//     FString Result;
//     IPlatformFile& File = FPlatformFileManager::Get().GetPlatformFile();

//     if (File.CreateDirectory(*Directory))
//     {
//         FString TargetFile = Directory + "/" + Filename;
//         FFileHelper::LoadFileToStringArray(Result, *TargetFile);
//     }

//     return Result;
// }
