// Copyright (c) 2024 Computer Vision Center (CVC) at the Universitat Autonoma de Barcelona (UAB). This work is licensed under the terms of the MIT license. For a copy, see <https://opensource.org/licenses/MIT>.


#include "NavMeshScannerGameMode.h"

#include "NavMeshMapScanner.h"

void ANavMeshScannerGameMode::BeginPlay()
{
	Super::BeginPlay();

	UNavMeshMapScanner* Scanner = NewObject<UNavMeshMapScanner>();
	Scanner->StartScan(true);
}
