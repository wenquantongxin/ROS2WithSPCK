#include "MoveComponent_Wheelset.h"
#include "GameFramework/Actor.h"
#include "Kismet/KismetMathLibrary.h"

// ���UDP������ & ����
#include "UDPReceiver.h"
#include "TrainData.h"
#include "Components/StaticMeshComponent.h"

static const float DistanceScale = 100.f;

UMoveComponent_Wheelset::UMoveComponent_Wheelset()
{
    PrimaryComponentTick.bCanEverTick = true;

    WheelsetIndex = 0;
    bUseWorldTransform = true;
    InterpSpeed = 5.0f;

    WheelRotationSmoothFactor = 20.0f;
    DataTimeoutThreshold = 0.5f; // Ĭ��0.5����������Ϊֹͣ

    bInitialized = false;
    UDPReceiverRef = nullptr;

    LeftWheelIndex = 0;
    RightWheelIndex = 1;
    LeftBarIndex = 0;
    RightBarIndex = 1;

    CurrentLocation = FVector::ZeroVector;
    CurrentRotation = FRotator::ZeroRotator;

    // ��ʼ���ǶȺ��ٶȱ���
    AccumLeftWheelRotation = 0.f;
    AccumRightWheelRotation = 0.f;
    LeftWheelRotSpeed = 0.f;
    RightWheelRotSpeed = 0.f;
    TargetLeftWheelRotSpeed = 0.f;
    TargetRightWheelRotSpeed = 0.f;

    // ��ʼ���ܸ���ر��� - ʵ��ֵ����BeginPlay�и��ݸܸ���������
    CurrentLeftBarPitch = 0.f;
    CurrentRightBarPitch = 0.f;

    // ��ʼ�����ݳ�ʱ��ر���
    TimeSinceLastValidData = 0.f;
    bDataTimeout = false;

    LastSimTime = -1.0;
    bFirstDataReceived = false;

    SetupDefaultTransform();

    LeftWheelMesh = nullptr;
    RightWheelMesh = nullptr;
    LeftBarMesh = nullptr;
    RightBarMesh = nullptr;
}

void UMoveComponent_Wheelset::BeginPlay()
{
    Super::BeginPlay();

    SetupDefaultTransform();

    CurrentLocation = DefaultLocation;
    CurrentRotation = DefaultRotation;

    LeftWheelIndex = WheelsetIndex * 2;
    RightWheelIndex = WheelsetIndex * 2 + 1;
    LeftBarIndex = WheelsetIndex * 2;
    RightBarIndex = WheelsetIndex * 2 + 1;

    // �Զ����Ҳ����ó��ֺ͸ܸ�����
    TArray<USceneComponent*> ChildComponents;
    GetChildrenComponents(false, ChildComponents);

    for (USceneComponent* Child : ChildComponents)
    {
        UStaticMeshComponent* MeshComp = Cast<UStaticMeshComponent>(Child);
        if (MeshComp)
        {
            FString CompName = Child->GetName();
            if (CompName.Contains(TEXT("LeftWheel"), ESearchCase::IgnoreCase))
            {
                LeftWheelMesh = MeshComp;
                //UE_LOG(LogTemp, Warning, TEXT("�Զ����� LeftWheelMesh = %s"), *CompName);

                // ȷ�����ֳ�ʼ��ת��ȷ����Ҫ��Z����ת90�ȣ�
                FRotator InitialRotation = MeshComp->GetRelativeRotation();
                InitialRotation.Roll += 90.0f; // ��Z����ת90��
                MeshComp->SetRelativeRotation(InitialRotation);
            }
            else if (CompName.Contains(TEXT("RightWheel"), ESearchCase::IgnoreCase))
            {
                RightWheelMesh = MeshComp;
                //UE_LOG(LogTemp, Warning, TEXT("�Զ����� RightWheelMesh = %s"), *CompName);
            }
            else if (CompName.Contains(TEXT("LeftBar"), ESearchCase::IgnoreCase))
            {
                LeftBarMesh = MeshComp;
                //UE_LOG(LogTemp, Warning, TEXT("�Զ����� LeftBarMesh = %s"), *CompName);
            }
            else if (CompName.Contains(TEXT("RightBar"), ESearchCase::IgnoreCase))
            {
                RightBarMesh = MeshComp;
                //UE_LOG(LogTemp, Warning, TEXT("�Զ����� RightBarMesh = %s"), *CompName);
            }
        }
    }

    // ��¼��ͼ���趨�������ת�����ֳ�ʼ���룩
    if (LeftWheelMesh)
    {
        InitialLeftWheelRot = LeftWheelMesh->GetRelativeRotation();
        //UE_LOG(LogTemp, Warning, TEXT("���ֳ�ʼ��ת��(P=%.3f, Y=%.3f, R=%.3f)"),
            //InitialLeftWheelRot.Pitch, InitialLeftWheelRot.Yaw, InitialLeftWheelRot.Roll);
    }
    if (RightWheelMesh)
    {
        InitialRightWheelRot = RightWheelMesh->GetRelativeRotation();
        //UE_LOG(LogTemp, Warning, TEXT("���ֳ�ʼ��ת��(P=%.3f, Y=%.3f, R=%.3f)"),
            //InitialRightWheelRot.Pitch, InitialRightWheelRot.Yaw, InitialRightWheelRot.Roll);
    }

    // ��¼�ܸ˵ĳ�ʼ��ת
    if (LeftBarMesh)
    {
        InitialLeftBarRot = LeftBarMesh->GetRelativeRotation();
        //UE_LOG(LogTemp, Warning, TEXT("��ܸ˳�ʼ��ת��(P=%.3f, Y=%.3f, R=%.3f)"),
            //InitialLeftBarRot.Pitch, InitialLeftBarRot.Yaw, InitialLeftBarRot.Roll);
    }
    if (RightBarMesh)
    {
        InitialRightBarRot = RightBarMesh->GetRelativeRotation();
        //UE_LOG(LogTemp, Warning, TEXT("�Ҹܸ˳�ʼ��ת��(P=%.3f, Y=%.3f, R=%.3f)"),
            //InitialRightBarRot.Pitch, InitialRightBarRot.Yaw, InitialRightBarRot.Roll);
    }

    // ���ݸܸ��������ó�ʼ�Ƕ� - ʹ�ܸ���X-Yƽ������X��������
    if (WheelsetIndex == 0 || WheelsetIndex == 2)
    {
        // 0��4�Ÿ˺�1��5�Ÿ˳�ʼת��
        // SPCK Joint ��ǰ��ת�� 4.52 rad = 259.09��; ��ǰ��ת�� 4.903 rad = 280.92��
        // UE5֮�У���Ҫ SPCK Joint ת�� - 270��; Deg_SPCK2UE = 270.0f;
        CurrentLeftBarPitch = 4.520 * RadToDeg - Deg_SPCK2UE;     // ��ǰ�� ��ʼת��
        CurrentRightBarPitch = 4.903 * RadToDeg - Deg_SPCK2UE;     // ��ǰ�� ��ʼת��
    }
    else if (WheelsetIndex == 1 || WheelsetIndex == 3)
    {
        // 2��6�Ÿ˺�3��7�Ÿ˳�ʼת��
        // SPCK Joint ����ת�� 1.761 rad = 100.898��; �Һ��ת�� 1.38 rad = 79.068��
        // UE5֮�У���Ҫ SPCK Joint ת�� - 270��
        CurrentLeftBarPitch = 1.761 * RadToDeg - Deg_SPCK2UE;      // ���� ��ʼת��
        CurrentRightBarPitch = 1.380 * RadToDeg - Deg_SPCK2UE;      // �Һ�� ��ʼת��
    }

    //UE_LOG(LogTemp, Warning, TEXT("�ܸ˳�ʼ�Ƕ�����: ��=%.2f��, ��=%.2f��"), 
    //    CurrentLeftBarPitch, CurrentRightBarPitch);

    bInitialized = false;
    bDataTimeout = true; // ��ʼ״̬Ϊ��ʱ��ֱ���յ���һ����Ч����

    //UE_LOG(LogTemp, Warning, TEXT("[BeginPlay] Wheelset=%d -> DefaultLocation=(%.3f, %.3f, %.3f), DefaultRotation=(P=%.3f, Y=%.3f, R=%.3f)"),
        //WheelsetIndex,
        //DefaultLocation.X, DefaultLocation.Y, DefaultLocation.Z,
        //DefaultRotation.Pitch, DefaultRotation.Yaw, DefaultRotation.Roll);
}

void UMoveComponent_Wheelset::SetupDefaultTransform()
{
    float xPos = 0.0f;
    switch (WheelsetIndex)
    {
    case 0:
        xPos = 10.0f;  // +10 ��
        break;
    case 1:
        xPos = 7.5f;   // +7.5 ��
        break;
    case 2:
        xPos = -7.5f;  // -7.5 ��
        break;
    case 3:
        xPos = -10.0f; // -10 ��
        break;
    default:
        xPos = 0.0f;
        //UE_LOG(LogTemp, Warning, TEXT("��Ч��WheelsetIndex: %d"), WheelsetIndex);
        break;
    }

    DefaultLocation = FVector(DistanceScale * xPos, 0.0f, DistanceScale * (0.43f));
    DefaultRotation = FRotator::ZeroRotator;
}

void UMoveComponent_Wheelset::SetWheelsetIndex(int32 NewIndex)
{
    if (NewIndex >= 0 && NewIndex <= 3 && WheelsetIndex != NewIndex)
    {
        WheelsetIndex = NewIndex;
        SetupDefaultTransform();

        LeftWheelIndex = WheelsetIndex * 2;
        RightWheelIndex = WheelsetIndex * 2 + 1;
        LeftBarIndex = WheelsetIndex * 2;
        RightBarIndex = WheelsetIndex * 2 + 1;

        // ���¸ܸ˳�ʼ�Ƕ� - ʹ����BeginPlay��ͬ�ļ����߼�
        if (WheelsetIndex == 0 || WheelsetIndex == 2)
        {
            // 0��4�Ÿ˺�1��5�Ÿ˳�ʼת��
            // SPCK Joint ��ǰ��ת�� 4.52 rad = 259.09��; ��ǰ��ת�� 4.903 rad = 280.92��
            CurrentLeftBarPitch = 4.520 * RadToDeg - Deg_SPCK2UE;     // ��ǰ�� ��ʼת��
            CurrentRightBarPitch = 4.903 * RadToDeg - Deg_SPCK2UE;     // ��ǰ�� ��ʼת��
        }
        else if (WheelsetIndex == 1 || WheelsetIndex == 3)
        {
            // 2��6�Ÿ˺�3��7�Ÿ˳�ʼת��
            // SPCK Joint ����ת�� 1.761 rad = 100.898��; �Һ��ת�� 1.38 rad = 79.068��
            CurrentLeftBarPitch = 1.761 * RadToDeg - Deg_SPCK2UE;      // ���� ��ʼת��
            CurrentRightBarPitch = 1.380 * RadToDeg - Deg_SPCK2UE;      // �Һ�� ��ʼת��
        }

        if (bInitialized)
        {
            CurrentLocation = DefaultLocation;
            CurrentRotation = DefaultRotation;
        }

        //UE_LOG(LogTemp, Log, TEXT("WheelsetIndex ���� -> %d"), WheelsetIndex);
    }
}

void UMoveComponent_Wheelset::TickComponent(float DeltaTime, ELevelTick TickType,
    FActorComponentTickFunction* ThisTickFunction)
{
    Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

    // 1) �������
    if (WheelsetIndex < 0 || WheelsetIndex > 3)
    {
        return;
    }

    // Ĭ���������ת�ٶ�������Ч
    bool bWheelSpeedDataValid = false;

    // 2) ���û����UDPReceiver����Ĭ��λ��
    if (!UDPReceiverRef)
    {
        if (!bInitialized)
        {
            CurrentLocation = DefaultLocation;
            CurrentRotation = DefaultRotation;
            bInitialized = true;
        }
        else
        {
            CurrentLocation = FMath::VInterpTo(CurrentLocation, DefaultLocation, DeltaTime, InterpSpeed);
            CurrentRotation = FMath::RInterpTo(CurrentRotation, DefaultRotation, DeltaTime, InterpSpeed);
        }

        // û��UDP����ʱ����ת�ٶ���Ϊ0
        TargetLeftWheelRotSpeed = 0.0f;
        TargetRightWheelRotSpeed = 0.0f;
    }
    else
    {
        // 3) ��ȡ���� FTrainData
        FTrainData TrainData;
        bool bHasData = UDPReceiverRef->GetLatestTrainData(TrainData);

        FVector TargetLocation = DefaultLocation;
        FRotator TargetRotation = DefaultRotation;

        // 4) ��������ݣ������λ�ú���ת(������SimTime�Ƿ�仯)
        if (bHasData &&
            WheelsetIndex < TrainData.WheelsetLocations.Num() &&
            WheelsetIndex < TrainData.WheelsetRotations.Num())
        {
            TargetLocation = TrainData.WheelsetLocations[WheelsetIndex];
            TargetRotation = TrainData.WheelsetRotations[WheelsetIndex];

            // 5) ���Գ���ת��Ӧ�ø��¼���߼�
            bool bDataActuallyUpdated = false;
            if (!bFirstDataReceived)
            {
                // �״��յ�����
                bDataActuallyUpdated = true;
                bFirstDataReceived = true;
                LastSimTime = TrainData.SimTime;
            }
            else if (FMath::Abs(TrainData.SimTime - LastSimTime) > 1e-6)
            {
                // SimTime�б仯��˵����������
                bDataActuallyUpdated = true;
                LastSimTime = TrainData.SimTime;
            }

            // ֻ��������������ʱ�Ÿ��³�����ת�ٶ�
            if (bDataActuallyUpdated &&
                LeftWheelIndex >= 0 && LeftWheelIndex < TrainData.WheelsRotSpeed.Num() &&
                RightWheelIndex >= 0 && RightWheelIndex < TrainData.WheelsRotSpeed.Num())
            {
                // ת��Ϊ�Ƕ�/��
                TargetLeftWheelRotSpeed = TrainData.WheelsRotSpeed[LeftWheelIndex] * RadToDeg;
                TargetRightWheelRotSpeed = TrainData.WheelsRotSpeed[RightWheelIndex] * RadToDeg;

                // ���ת��������Ч
                bWheelSpeedDataValid = true;

                // ���ó�ʱ��ʱ��
                TimeSinceLastValidData = 0.0f;
                bDataTimeout = false;
            }
            // ������δ����ʱ�����ֵ�ǰĿ��ת�ٲ��䣬��Ҫ��Ϊ0

            // ����ܸ���ת - �޸Ĳ���
            if (bDataActuallyUpdated &&
                LeftBarIndex >= 0 && LeftBarIndex < TrainData.BarsPitch.Num() &&
                RightBarIndex >= 0 && RightBarIndex < TrainData.BarsPitch.Num())
            {
                // ��ȡԭʼ�ܸ�pitch�Ƕȣ�����ֵ����UDP���ն�תΪ�Ƕȣ�
                float rawLeftPitch = TrainData.BarsPitch[LeftBarIndex];
                float rawRightPitch = TrainData.BarsPitch[RightBarIndex];

                // Ӧ�û����任����ȥ270��ƫ��
                float leftPitch = rawLeftPitch - Deg_SPCK2UE;
                float rightPitch = rawRightPitch - Deg_SPCK2UE;

                // �����ֶ�λ��Ӧ�þ����ϵ
                if (WheelsetIndex == 1 || WheelsetIndex == 3)  // ���ֶ�
                {
                    //leftPitch = leftPitch - 180.0f;
                    //rightPitch = rightPitch ;
                    leftPitch = leftPitch ;     // ��ʵ�ϲ���Ҫ�任
                    rightPitch = rightPitch;

                }

                // ƽ����ֵ�ܸ�pitch�Ƕ�
                CurrentLeftBarPitch = FMath::FInterpTo(CurrentLeftBarPitch, leftPitch,
                    DeltaTime, WheelRotationSmoothFactor);
                CurrentRightBarPitch = FMath::FInterpTo(CurrentRightBarPitch, rightPitch,
                    DeltaTime, WheelRotationSmoothFactor);

            }
        }
        else
        {
            // û����Ч����ʱ����ת�ٶ�Ŀ����Ϊ0
            TargetLeftWheelRotSpeed = 0.0f;
            TargetRightWheelRotSpeed = 0.0f;
        }

        // 6) ƽ����ֵλ�ú���ת - �������ݸ��¼��Ӱ��
        if (!bInitialized)
        {
            CurrentLocation = TargetLocation;
            CurrentRotation = TargetRotation;
            bInitialized = true;
        }
        else
        {
            CurrentLocation = FMath::VInterpTo(CurrentLocation, TargetLocation, DeltaTime, InterpSpeed);
            CurrentRotation = FMath::RInterpTo(CurrentRotation, TargetRotation, DeltaTime, InterpSpeed);
        }
    }

    // 7) ������ת�����ݳ�ʱ�߼� - ��Ӱ�쳵��ת��
    if (!bWheelSpeedDataValid)
    {
        // ����Чת������ʱ�ۼӳ�ʱ��ʱ��
        TimeSinceLastValidData += DeltaTime;

        // ����Ƿ񳬹���ֵ
        if (TimeSinceLastValidData > DataTimeoutThreshold && !bDataTimeout)
        {
            bDataTimeout = true;

        }
    }

    // 8) Ӧ�ñ任��Actor - �������ݸ��¼��Ӱ��
    AActor* OwnerActor = GetOwner();
    if (!OwnerActor) return;

    if (bUseWorldTransform)
    {
        OwnerActor->SetActorLocationAndRotation(CurrentLocation, CurrentRotation);
    }
    else
    {
        SetRelativeLocationAndRotation(CurrentLocation, CurrentRotation);
    }

    // 9) �������ݳ�ʱ״̬��������ת
    if (bDataTimeout)
    {
        // ���ݳ�ʱʱ��ֱ�ӽ���ǰ��ת�ٶ���Ϊ0����������ƽ������
        LeftWheelRotSpeed = 0.0f;
        RightWheelRotSpeed = 0.0f;

        // ���ݳ�ʱ��Ӱ�쳵����ת�ٶȣ���Ӱ��ܸ˽Ƕ�
    }
    else
    {
        // ����״̬��ʹ��ƽ��������ת�ٶ�
        LeftWheelRotSpeed = FMath::FInterpTo(LeftWheelRotSpeed, TargetLeftWheelRotSpeed,
            DeltaTime, WheelRotationSmoothFactor);
        RightWheelRotSpeed = FMath::FInterpTo(RightWheelRotSpeed, TargetRightWheelRotSpeed,
            DeltaTime, WheelRotationSmoothFactor);
    }

    // 10) ������ת�ٶȸ����ۻ��Ƕ�
    AccumLeftWheelRotation += LeftWheelRotSpeed * DeltaTime;
    AccumRightWheelRotation += RightWheelRotSpeed * DeltaTime;

    // �淶����[0,360]���䣬��ֹ����������
    AccumLeftWheelRotation = FMath::Fmod(AccumLeftWheelRotation, 360.0f);
    if (AccumLeftWheelRotation < 0.0f) AccumLeftWheelRotation += 360.0f;

    AccumRightWheelRotation = FMath::Fmod(AccumRightWheelRotation, 360.0f);
    if (AccumRightWheelRotation < 0.0f) AccumRightWheelRotation += 360.0f;

    // 11) Ӧ���ۻ��Ƕȵ�����������
    if (LeftWheelMesh)
    {
        FRotator NewLeftRot = InitialLeftWheelRot + FRotator(0.f, 0.f, AccumLeftWheelRotation);
        LeftWheelMesh->SetRelativeRotation(NewLeftRot);
    }

    if (RightWheelMesh)
    {
        // ע��������ת������Ҫȡ����
        FRotator NewRightRot = InitialRightWheelRot + FRotator(0.f, 0.f, -AccumRightWheelRotation);
        RightWheelMesh->SetRelativeRotation(NewRightRot);
    }

    // 12) Ӧ��pitch�Ƕȵ��ܸ������� - ���Բ�ͬ����ת��
    // ԭʼ����ʹ��Yaw(Y)��������������Ҫ��ΪPitch(P)��Roll(R)
    if (LeftBarMesh)
    {
        FRotator NewLeftBarRot = InitialLeftBarRot + FRotator(0.f, CurrentLeftBarPitch, 0.f);
        LeftBarMesh->SetRelativeRotation(NewLeftBarRot);
    }

    if (RightBarMesh)
    {
        FRotator NewRightBarRot = InitialRightBarRot + FRotator(0.f, CurrentRightBarPitch, 0.f);
        RightBarMesh->SetRelativeRotation(NewRightBarRot);
    }
}