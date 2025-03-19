#include "MoveComponent_Carbody.h"
#include "GameFramework/Actor.h"
#include "Kismet/KismetMathLibrary.h"
#include "UDPReceiver.h"  // ����GetLatestTrainData
#include "TrainData.h"    // FTrainData

UMoveComponent_Carbody::UMoveComponent_Carbody()
{
    PrimaryComponentTick.bCanEverTick = true;

    // Ĭ��ֱ��ʹ����������
    bUseWorldTransform = true;

    // ȱʡ��ֵ�ٶ�
    InterpSpeed = 5.0f;

    bInitialized = false;
    CurrentLocation = FVector::ZeroVector;
    CurrentRotation = FRotator::ZeroRotator;
}

void UMoveComponent_Carbody::BeginPlay()
{
    Super::BeginPlay();

    // ������ڳ�ʼ���� CurrentLocation = ��Actor ���������꣬���������ȡ
    AActor* OwnerActor = GetOwner();
    if (OwnerActor)
    {
        CurrentLocation = OwnerActor->GetActorLocation();
        CurrentRotation = OwnerActor->GetActorRotation();
    }

    // �ȴ���һ�ν��յ����ݺ��ٽ� bInitialized ��Ϊtrue��
    bInitialized = false;
}

void UMoveComponent_Carbody::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
    Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

    // 1) ȷ����UDPReceiver����
    if (!UDPReceiverRef)
    {
        return;
    }

    // 2) ��ȡ����TrainData
    FTrainData TrainData;
    if (!UDPReceiverRef->GetLatestTrainData(TrainData))
    {
        return; // ��û����Ч����
    }

    // 3) ��ȡĿ��λ������ת(����ռ�)
    FVector  TargetLocation = TrainData.CarBodyLocation;
    FRotator TargetRotation = TrainData.CarBodyRotation; // (Pitch,Yaw,Roll)

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
        // ע�⣺��ʾ��ֱ��Set����������������꣬���и��Ӽ�Actor���������Ҫ�����⻻��
        SetRelativeLocationAndRotation(CurrentLocation, CurrentRotation);
    }
}
