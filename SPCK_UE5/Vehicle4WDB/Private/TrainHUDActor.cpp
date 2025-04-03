// File: TrainHUDActor.cpp

#include "TrainHUDActor.h"
#include "TrainDataWidget.h"
#include "UDPReceiver.h"        // �����е�AUDPReceiver
#include "Kismet/GameplayStatics.h"
#include "Engine/World.h"

ATrainHUDActor::ATrainHUDActor()
{
    PrimaryActorTick.bCanEverTick = true; // ����Ҫ��Tick�����UI
    bAutoFindUDPReceiver = false; // �Ƿ��Զ�Ѱ�� BP_UDP 
    ManualUDPReceiverRef = nullptr;

    TrainDataWidgetInstance = nullptr;
    UDPReceiverPtr = nullptr;
}

void ATrainHUDActor::BeginPlay()
{
    Super::BeginPlay();

    // 1) �ҵ������� UDPReceiver
    if (bAutoFindUDPReceiver)
    {
        // ���ҹؿ�������AUDPReceiver
        TArray<AActor*> FoundReceivers;
        UGameplayStatics::GetAllActorsOfClass(GetWorld(), AUDPReceiver::StaticClass(), FoundReceivers);
        if (FoundReceivers.Num() > 0)
        {
            // ֻ�õ�һ����Ҳ���������ӵ�ɸѡ
            UDPReceiverPtr = Cast<AUDPReceiver>(FoundReceivers[0]);
        }
    }
    else
    {
        UDPReceiverPtr = ManualUDPReceiverRef;
    }

    // 2) ����HUD Widget
    if (TrainDataWidgetClass)
    {
        TrainDataWidgetInstance = CreateWidget<UTrainDataWidget>(GetWorld(), TrainDataWidgetClass);
        if (TrainDataWidgetInstance)
        {
            TrainDataWidgetInstance->AddToViewport();
        }
    }
}

void ATrainHUDActor::Tick(float DeltaTime)
{
    Super::Tick(DeltaTime);

    // 3) ÿ֡�����Receiver��Widget���ͻ�ȡ�ٶȸ���
    if (UDPReceiverPtr && TrainDataWidgetInstance)
    {
        FTrainData LatestData;
        if (UDPReceiverPtr->GetLatestTrainData(LatestData))
        {
            float Speed_mps = LatestData.CarBodyVx;
            float LongitudinalThroughTrack_m = LatestData.CarBodyLocation.X / 100;

            // 1) �����ٶ�
            TrainDataWidgetInstance->UpdateSpeed(Speed_mps);

            // 2) �������
            TrainDataWidgetInstance->UpdateTrackS(LongitudinalThroughTrack_m);

        }
    }
}
