#include "CarlaUnrealEditor.h"
#include "ToolMenus.h"
#include "NavMeshMapScanner.h"

#define LOCTEXT_NAMESPACE "FCarlaUnrealEditorModule"

void FCarlaUnrealEditorModule::StartupModule()
{
	UToolMenus::RegisterStartupCallback(
		FSimpleMulticastDelegate::FDelegate::CreateRaw(
			this, &FCarlaUnrealEditorModule::RegisterMenus));
}

void FCarlaUnrealEditorModule::ShutdownModule()
{
	UToolMenus::UnRegisterStartupCallback(this);
	UToolMenus::UnregisterOwner(this);
}

void FCarlaUnrealEditorModule::RegisterMenus()
{
	FToolMenuOwnerScoped OwnerScoped(this);

	UToolMenu* PlayToolbar =
		UToolMenus::Get()->ExtendMenu("LevelEditor.LevelEditorToolBar.PlayToolBar");

	if (!PlayToolbar)
	{
		UE_LOG(LogTemp, Error, TEXT("PlayToolBar not found!"));
		return;
	}

	FToolMenuSection& Section =
		PlayToolbar->FindOrAddSection("Play");

	FToolMenuEntry Entry = FToolMenuEntry::InitToolBarButton(
		"ScanNavMeshes",
		FUIAction(FExecuteAction::CreateRaw(
			this, &FCarlaUnrealEditorModule::OnButtonClicked)),
		LOCTEXT("ScanNavMeshes_Label", "ScanNav"),
		LOCTEXT("ScanNavMeshes_Tooltip", "Scan all maps for NavMesh"),
		FSlateIcon(FAppStyle::GetAppStyleSetName(), "Icons.Search")
	);

	Entry.SetCommandList(nullptr);

	Section.AddEntry(Entry);
}

void FCarlaUnrealEditorModule::OnButtonClicked()
{
	UNavMeshMapScanner* Scanner = NewObject<UNavMeshMapScanner>();
	Scanner->AddToRoot();
	Scanner->StartScan(false);
}

#undef LOCTEXT_NAMESPACE
    
IMPLEMENT_MODULE(FCarlaUnrealEditorModule, CarlaUnrealEditor)