// UDPReceiver.cpp

#include "UDPReceiver.h"
#include "Engine/Engine.h"    // UE_LOG ��
#include "Sockets.h"
#include "SocketSubsystem.h"
#include "IPAddress.h"
#include "Common/UdpSocketBuilder.h"
#include "Common/UdpSocketReceiver.h"
#include "Misc/ScopeLock.h"
#include "HAL/UnrealMemory.h"
#include "Math/UnrealMathUtility.h"
#include "TrainData.h"        // ��Ҫ������� FTrainData �ṹ

/*

 S1 UDPReceiver �г��� 100���ס����ף�
 S2 ����̬�ǳ��� -AngleScale�����Ȼ���ת�ȣ���ȡ���ţ���
 S3 ӳ�䵽��Z ���� + ����ϵ + ��λ cm + (Yaw��Pitch��Roll) - UE �Ĺ���

*/

// ---------------------- ת���������� ----------------------
// ����ⲿλ������ "��" Ϊ��λ -> UE ��Ĭ��Ϊ "����"��     1m = 100cm
static const float DistanceScale = 100.f;

// �ⲿ�� "����" => ��Ҫ (180.f / PI)
static const float AngleScale = 180.f / PI;

// ����Ĭ�ϸ߶�
static const float cb_hc = 1.2f; // 1.2f;

// ת�����߸߶�
static const float bg_hc = 0.0; 

// �ֶ���߸߶�
static const float ws_hc = 0.0; 

//----------------------------------------------------------

AUDPReceiver::AUDPReceiver()
{
    PrimaryActorTick.bCanEverTick = true;

    ListenSocket = nullptr;
    UDPReceiver = nullptr;

    // 96 (ԭ77) �� double��ÿ�� double 8 �ֽ� -> 616 �ֽ�
    ExpectedDataSize = 96 * sizeof(double);

    // �տ�ʼ��δ�յ��κ���Ч����
    bHasValidData = false;
}

void AUDPReceiver::BeginPlay()
{
    Super::BeginPlay();

    // ��ʼ�� Socket �� UDPReceiver
    InitializeUDPReceiver();
}

bool AUDPReceiver::InitializeUDPReceiver()
{
    CloseSocket(); // ȷ��֮ǰ����Դ����

    ISocketSubsystem* SocketSubsystem = ISocketSubsystem::Get(PLATFORM_SOCKETSUBSYSTEM);
    if (!SocketSubsystem)
    {
        UE_LOG(LogTemp, Warning, TEXT("UDPReceiver: Socket subsystem unavailable"));
        return false;
    }

    // �󶨵�ַ(0.0.0.0:10099 ֮��)������������޸Ķ˿�
    FIPv4Address Addr = FIPv4Address::Any;
    const uint16 Port = 10099; // �� ROS2 �ڵ㷢�Ͷ˿�ƥ��
    FIPv4Endpoint Endpoint(Addr, Port);

    // ���� UDP Socket
    ListenSocket = SocketSubsystem->CreateSocket(NAME_DGram, TEXT("UDPReceiverSocket"), false);
    if (!ListenSocket)
    {
        UE_LOG(LogTemp, Warning, TEXT("UDPReceiver: Failed to create socket"));
        return false;
    }

    // ��Ϊ������
    ListenSocket->SetNonBlocking(true);

    // �󶨱��ض˿�
    bool bBindOk = ListenSocket->SetReuseAddr(true) &&
        ListenSocket->SetRecvErr() &&
        ListenSocket->Bind(*Endpoint.ToInternetAddr());
    if (!bBindOk)
    {
        UE_LOG(LogTemp, Warning, TEXT("UDPReceiver: Failed to bind port %d"), Port);
        CloseSocket();
        return false;
    }

    // �����첽������
    UDPReceiver = MakeShared<FUdpSocketReceiver>(
        ListenSocket,
        FTimespan::FromMilliseconds(10),
        TEXT("UDPReceiverThread")
    );

    if (UDPReceiver.IsValid())
    {
        // �󶨻ص�
        UDPReceiver->OnDataReceived().BindUObject(this, &AUDPReceiver::Recv);
        UDPReceiver->Start();

        UE_LOG(LogTemp, Log, TEXT("UDPReceiver: Listening on port %d"), Port);
        return true;
    }

    UE_LOG(LogTemp, Warning, TEXT("UDPReceiver: Failed to create UDPSocketReceiver"));
    CloseSocket();
    return false;
}

void AUDPReceiver::Tick(float DeltaSeconds)
{
    Super::Tick(DeltaSeconds);
    // �����Ҫ��� Socket ״̬���ڴ˴���
}

void AUDPReceiver::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
    CloseSocket();
    Super::EndPlay(EndPlayReason);
}

void AUDPReceiver::CloseSocket()
{
    if (UDPReceiver.IsValid())
    {
        UDPReceiver->Stop();
        UDPReceiver.Reset();
    }
    if (ListenSocket)
    {
        ISocketSubsystem* SocketSubsystem = ISocketSubsystem::Get(PLATFORM_SOCKETSUBSYSTEM);
        if (SocketSubsystem)
        {
            ListenSocket->Close();
            SocketSubsystem->DestroySocket(ListenSocket);
        }
        ListenSocket = nullptr;
    }
}

// ���ջص�
void AUDPReceiver::Recv(const FArrayReaderPtr& ArrayReaderPtr, const FIPv4Endpoint& EndPt)
{
    if (!IsValid(this) || !ListenSocket || !UDPReceiver.IsValid())
    {
        return; // Actor�����ٻ�Socket��Ч
    }

    // 1) ����ָ��ͳ��ȼ��
    if (!ArrayReaderPtr.IsValid())
        return;

    int32 ReceivedSize = ArrayReaderPtr->Num();
    if (ReceivedSize != ExpectedDataSize)
    {
        UE_LOG(LogTemp, Warning, TEXT("UDPReceiver: Data size mismatch. Expect %d, got %d"),
            ExpectedDataSize, ReceivedSize);
        return;
    }

    // 2) ���� 91(ԭ77) �� double
    const double* rawPtr = reinterpret_cast<const double*>(ArrayReaderPtr->GetData());
    FTrainData newData; // ��ʱ�洢

    // (1) ROS 2 ���ڲ�ʱ���, (2) y_spcktime, (3) y_cb_vx
    int idx = 0;
    newData.SimTime = rawPtr[idx++];
    newData.SPCKTime =  rawPtr[idx++];
    newData.CarBodyVx = rawPtr[idx++];

    // (4~9) ����(X, Y, Z, roll, yaw, pitch)
    double cbX = rawPtr[idx++];
    double cbY = rawPtr[idx++];
    double cbZ = rawPtr[idx++] + cb_hc;
    double cbRoll = rawPtr[idx++];
    double cbYaw = rawPtr[idx++];
    double cbPitch = rawPtr[idx++];

    //UE_LOG(LogTemp, Log, TEXT("Recv: CarBody (X=%.3f, Y=%.3f, Z=%.3f) [m]"), cbX, cbY, cbZ);

    cbX *= DistanceScale;
    cbY *= DistanceScale;
    cbZ *= DistanceScale;

    cbRoll *= - AngleScale;
    cbYaw *=  - AngleScale;
    cbPitch *= - AngleScale;

    newData.CarBodyLocation = FVector(static_cast<float>(cbX),
        static_cast<float>(cbY),
        static_cast<float>(cbZ));
    newData.CarBodyRotation = FRotator(cbPitch, cbYaw, cbRoll);

    // (10~17) 8���ֵ���ת�ٶ� rotw
    for (int i = 0; i < 8; i++)
    {
        newData.WheelsRotSpeed[i] = rawPtr[idx++];
    }

    // (18~23) ת��� #1 (X,Y,Z, roll,yaw,pitch)
    double b1X = rawPtr[idx++];
    double b1Y = rawPtr[idx++];
    double b1Z = rawPtr[idx++] + bg_hc;
    double b1Roll = rawPtr[idx++];
    double b1Yaw = rawPtr[idx++];
    double b1Pitch = rawPtr[idx++];

    b1X *= DistanceScale;
    b1Y *= DistanceScale;
    b1Z *= DistanceScale;
    b1Roll *= - AngleScale;
    b1Yaw *= - AngleScale;
    b1Pitch *= - AngleScale;

    newData.Bogie01Location = FVector(static_cast<float>(b1X),
        static_cast<float>(b1Y),
        static_cast<float>(b1Z));
    newData.Bogie01Rotation = FRotator(b1Pitch, b1Yaw, b1Roll);

    // (24~29) ת��� #2
    double b2X = rawPtr[idx++];
    double b2Y = rawPtr[idx++];
    double b2Z = rawPtr[idx++] + bg_hc;
    double b2Roll = rawPtr[idx++];
    double b2Yaw = rawPtr[idx++];
    double b2Pitch = rawPtr[idx++];

    b2X *= DistanceScale;
    b2Y *= DistanceScale;
    b2Z *= DistanceScale;
    b2Roll *=  - AngleScale;
    b2Yaw *=  - AngleScale;
    b2Pitch *= - AngleScale;

    newData.Bogie02Location = FVector(static_cast<float>(b2X),
        static_cast<float>(b2Y),
        static_cast<float>(b2Z));
    newData.Bogie02Rotation = FRotator(b2Pitch, b2Yaw, b2Roll);

    // (30~53) 4���ֶ� (X, Y, Z, roll, yaw, pitch)
    for (int i = 0; i < 4; i++)
    {
        double wsX = rawPtr[idx++];
        double wsY = rawPtr[idx++];
        double wsZ = rawPtr[idx++] + ws_hc;
        double wsRoll = rawPtr[idx++];
        double wsYaw = rawPtr[idx++];
        double wsPitch = rawPtr[idx++];

        wsX *= DistanceScale;
        wsY *= DistanceScale;
        wsZ *= DistanceScale;
        wsRoll *=  - AngleScale;
        wsYaw *=  - AngleScale;
        wsPitch *=  - AngleScale;

        newData.WheelsetLocations[i] = FVector(static_cast<float>(wsX),
            static_cast<float>(wsY),
            static_cast<float>(wsZ));
        newData.WheelsetRotations[i] = FRotator(wsPitch, wsYaw, wsRoll);
    }

    // (54~61) 8������ת�� rota
    for (int i = 0; i < 8; i++)
    {
        double wrota = rawPtr[idx++];
        wrota *= AngleScale;
        newData.WheelsRotation[i] = wrota;
    }

    // (62~69) 8���ܸ� pitch
    for (int i = 0; i < 8; i++)
    {
        double barPitch = rawPtr[idx++];
        barPitch *= AngleScale;
        newData.BarsPitch[i] = barPitch;
    }

    // (70~73) 4���ֶ� vy
    for (int i = 0; i < 4; i++)
    {
        newData.WheelsetVY[i] = rawPtr[idx++];
    }

    // (74~77) 4���ֶ� vyaw
    for (int i = 0; i < 4; i++)
    {
        newData.WheelsetVYaw[i] = rawPtr[idx++];
    }

    // (78~79) ƽ����ָ��, (80~83) �ֹ���, (84~91) ��������-����ת��
    for (int i = 0; i < 2; i++)
    {
        newData.CarbodyComfort[i] = rawPtr[idx++];
    }
    for (int i = 0; i < 4; i++)
    {
        newData.WheelRailContactForce[i] = rawPtr[idx++];
    }    
    for (int i = 0; i < 8; i++)
    {
        newData.InputTorque[i] = rawPtr[idx++];
    }

    // (92) �������
    newData.TrackS = rawPtr[idx++];

    // (93~94) Sperlingָ��, (95~96) �ѹ�ϵ��
    for (int i = 0; i < 2; i++)
    {
        newData.SperlingYZ[i] = rawPtr[idx++];
    }
    for (int i = 0; i < 2; i++)
    {
        newData.DerailmentIndex[i] = rawPtr[idx++];
    }

    // ���Ҫ��� NaN / �����ȣ���ʹ�� IsValidData
    if (!IsValidData(cbX, cbY, cbZ, cbRoll, cbYaw, cbPitch))
    {
        UE_LOG(LogTemp, Verbose, TEXT("Invalid carbody transform data. ignoring."));
        return;
    }

    // 3) д���Ա���� (�̰߳�ȫ����)
    {
        FScopeLock Lock(&DataMutex);
        LatestTrainData = newData;

        // ********* �ؼ��߼����ɹ�������һ����Ч���ݺ���Ϊtrue *********
        bHasValidData = true;
    }
}

bool AUDPReceiver::IsValidData(double x, double y, double z, double roll, double yaw, double pitch)
{
    // ��ֹ NaN �� �����
    if (!FMath::IsFinite((float)x) ||
        !FMath::IsFinite((float)roll))
    {
        return false;
    }
    // λ�ó�������Ҳ�������쳣
    if (FMath::Abs((float)x) > 1e8f || FMath::Abs((float)y) > 1e8f)
    {
        return false;
    }
    return true;
}

bool AUDPReceiver::GetLatestTrainData(FTrainData& OutData) const
{
    FScopeLock Lock(&DataMutex);
    OutData = LatestTrainData;

    // �� bHasValidData == false ʱ����ζ����δ�յ����κ���Ч����
    // ��ʱ���� false�����ϲ��������ʹ�á�Ĭ��λ�á���
    return bHasValidData;
}
