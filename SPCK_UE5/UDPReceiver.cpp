#include "UDPReceiver.h"

AUDPReceiver::AUDPReceiver()
{
    // 不需要每帧Tick这个Actor，所以置为false
    PrimaryActorTick.bCanEverTick = false;

    ListenSocket = nullptr;
    UDPReceiver = nullptr;

    LatestLocation = FVector::ZeroVector;
    LatestRotation = FRotator::ZeroRotator;
}

void AUDPReceiver::BeginPlay()
{
    Super::BeginPlay();

    // 1. 创建一个FIPv4Endpoint：使用ANY地址 + 8888端口
    FIPv4Address Address = FIPv4Address::Any;
    const uint16 Port = 8888;  // 与Python脚本对应
    FIPv4Endpoint Endpoint(Address, Port);

    // 2. 用FUdpSocketBuilder创建Socket
    ListenSocket = FUdpSocketBuilder(TEXT("MyUDPSocket"))
        .AsNonBlocking()
        .AsReusable()
        .BoundToEndpoint(Endpoint)
        .WithReceiveBufferSize(2 * 1024 * 1024);

    if (ListenSocket)
    {
        // 3. 创建异步接收器
        UDPReceiver = MakeShared<FUdpSocketReceiver>(
            ListenSocket,
            FTimespan::FromMilliseconds(100),
            TEXT("UDPReceiverThread")
        );

        // 4. 绑定接收数据时的回调
        if (UDPReceiver.IsValid())
        {
            UDPReceiver->OnDataReceived().BindUObject(this, &AUDPReceiver::Recv);
            // 5. 启动接收器
            UDPReceiver->Start();
            UE_LOG(LogTemp, Log, TEXT("UDPReceiver listening on port %d"), Port);
        }
    }
    else
    {
        UE_LOG(LogTemp, Warning, TEXT("Failed to create UDP ListenSocket on port %d"), Port);
    }
}

void AUDPReceiver::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
    // 停止接收并关闭Socket
    if (UDPReceiver.IsValid())
    {
        UDPReceiver->Stop();
    }

    if (ListenSocket)
    {
        ListenSocket->Close();
        ISocketSubsystem::Get(PLATFORM_SOCKETSUBSYSTEM)->DestroySocket(ListenSocket);
        ListenSocket = nullptr;
    }

    Super::EndPlay(EndPlayReason);
}

void AUDPReceiver::Recv(const FArrayReaderPtr& ArrayReaderPtr, const FIPv4Endpoint& EndPt)
{
    // Python发送顺序： x, y, z, pitch, yaw, roll
    // 这里按同顺序解包
    float x = 0.f;
    float y = 0.f;
    float z = 0.f;
    float pitch = 0.f;
    float yaw = 0.f;
    float roll = 0.f;

    *ArrayReaderPtr << x;
    *ArrayReaderPtr << y;
    *ArrayReaderPtr << z;
    *ArrayReaderPtr << pitch;
    *ArrayReaderPtr << yaw;
    *ArrayReaderPtr << roll;

    // 用互斥锁保护多线程访问
    FScopeLock Lock(&DataMutex);
    LatestLocation = FVector(x, y, z);
    // 注意：UE Rotator构造顺序为 (Pitch, Yaw, Roll)
    LatestRotation = FRotator(pitch, yaw, roll);

    // 也可以选择性输出日志查看
    // UE_LOG(LogTemp, Log, TEXT("Recv from %s: loc=(%.2f, %.2f, %.2f), rot=(pitch=%.2f, yaw=%.2f, roll=%.2f)"),
    //        *EndPt.ToString(), x, y, z, pitch, yaw, roll);
}

bool AUDPReceiver::GetLatestTransformData(FVector& OutLocation, FRotator& OutRotation)
{
    FScopeLock Lock(&DataMutex);
    OutLocation = LatestLocation;
    OutRotation = LatestRotation;
    return true;
}
