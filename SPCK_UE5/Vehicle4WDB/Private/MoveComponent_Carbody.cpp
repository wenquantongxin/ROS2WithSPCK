// @encoding=GB2312
// MoveComponent_Carbody.cpp

#include "MoveComponent_Carbody.h"
#include "GameFramework/Actor.h"
#include "Kismet/KismetMathLibrary.h"
#include "UDPReceiver.h"  // ����GetLatestTrainData
#include "TrainData.h"    // FTrainData

static const float cb_hc = 1.2f; // 1.2f;
static const float DistanceScale = 100.f;

UMoveComponent_Carbody::UMoveComponent_Carbody()
{
    PrimaryComponentTick.bCanEverTick = true;

    // Ĭ��ֱ��ʹ����������
    bUseWorldTransform = true;

    // ȱʡ��ֵ�ٶ�
    InterpSpeed = 5.0f;

    bInitialized = false;

    // �Ƚ���ֵλ�ó�ʼ��Ϊ����Ĭ��λ��
    SetupDefaultTransform();
    CurrentLocation = DefaultLocation;
    CurrentRotation = DefaultRotation;
}

void UMoveComponent_Carbody::SetupDefaultTransform()
{
    // ���ó���Ĭ��λ��Ϊ(0, 0, cb_hc)��ע�ⵥλת��
    DefaultLocation = FVector(0.0f, 0.0f, cb_hc * DistanceScale);

    // Ĭ����תΪ��
    DefaultRotation = FRotator::ZeroRotator;
}

void UMoveComponent_Carbody::BeginPlay()
{
    Super::BeginPlay();

    // ��������Ĭ�ϱ任
    SetupDefaultTransform();

    // ����ǰλ������ΪĬ��λ�ã���ȷ����ʼ��ʾ��ȷ
    CurrentLocation = DefaultLocation;
    CurrentRotation = DefaultRotation;

    // �ȴ���һ�ν��յ����ݺ��ٽ� bInitialized ��Ϊtrue
    bInitialized = false;
}

void UMoveComponent_Carbody::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
    Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

    // 1) ȷ����UDPReceiver����
    if (!UDPReceiverRef)
    {
        // ���û��UDPReceiver��ֱ��ʹ��Ĭ��ֵ
        if (!bInitialized)
        {
            // δ��ʼ��ǰֱ������Ĭ��
            CurrentLocation = DefaultLocation;
            CurrentRotation = DefaultRotation;
            bInitialized = true;
        }
        else
        {
            // �ѳ�ʼ������в�ֵ
            CurrentLocation = FMath::VInterpTo(CurrentLocation, DefaultLocation, DeltaTime, InterpSpeed);
            CurrentRotation = FMath::RInterpTo(CurrentRotation, DefaultRotation, DeltaTime, InterpSpeed);
        }

        // Ӧ��λ�ú���ת
        ApplyTransform();
        return;
    }

    // 2) ��ȡ����TrainData
    FTrainData TrainData;
    bool bHasData = UDPReceiverRef->GetLatestTrainData(TrainData);

    // ����Ŀ��λ������ת��Ĭ����ʹ��Defaultֵ
    FVector TargetLocation = DefaultLocation;
    FRotator TargetRotation = DefaultRotation;

    // 3) ��������ݣ�����Ŀ��λ������ת
    if (bHasData)
    {
        TargetLocation = TrainData.CarBodyLocation;
        TargetRotation = TrainData.CarBodyRotation;
    }

    // 4) ��ƽ����ֵ����������
    if (!bInitialized)
    {
        // ��һ�Σ�ֱ������Ŀ��λ�ã������ʼ˲��
        CurrentLocation = TargetLocation;
        CurrentRotation = TargetRotation;
        bInitialized = true;
    }
    else
    {
        // λ�ò�ֵ
        CurrentLocation = FMath::VInterpTo(
            CurrentLocation,   // ��һ֡��ֵλ��
            TargetLocation,    // ��֡��Ҫ�����λ��
            DeltaTime,         // ֡DeltaTime
            InterpSpeed        // ��ֵ�ٶ�
        );

        // ��ת��ֵ
        CurrentRotation = FMath::RInterpTo(
            CurrentRotation,
            TargetRotation,
            DeltaTime,
            InterpSpeed
        );
    }

    // 5) ����ֵ���Ӧ��
    ApplyTransform();
}

void UMoveComponent_Carbody::ApplyTransform()
{
    AActor* OwnerActor = GetOwner();
    if (!OwnerActor) return;

    if (bUseWorldTransform)
    {
        // ֱ��Set��������
        OwnerActor->SetActorLocationAndRotation(CurrentLocation, CurrentRotation);
    }
    else
    {
        // ʹ���������
        SetRelativeLocationAndRotation(CurrentLocation, CurrentRotation);
    }
}