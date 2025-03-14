#include "UDPReceiver.h"
#include "HAL/RunnableThread.h"
#include "Misc/ScopeLock.h"

AUDPReceiver::AUDPReceiver()
{
    PrimaryActorTick.bCanEverTick = true;

    // 初始化成员变量
    ListenSocket = nullptr;
    UDPReceiver = nullptr;
    LatestLocation = FVector::ZeroVector;
    LatestRotation = FRotator::ZeroRotator;
    bHasReceivedData = false;

    // 预期数据大小：6个float (位置XYZ和旋转Pitch/Yaw/Roll)
    ExpectedDataSize = 6 * sizeof(float);
}

void AUDPReceiver::BeginPlay()
{
    Super::BeginPlay();
    // 启动时立即初始化UDP接收器
    InitializeUDPReceiver();
}

bool AUDPReceiver::InitializeUDPReceiver()
{
    // 确保清理任何现有资源
    CloseSocket();

    // 创建Socket子系统
    ISocketSubsystem* SocketSubsystem = ISocketSubsystem::Get(PLATFORM_SOCKETSUBSYSTEM);
    if (!SocketSubsystem)
    {
        UE_LOG(LogTemp, Warning, TEXT("UDPReceiver: Socket subsystem unavailable"));
        return false;
    }

    // 创建监听端点 (0.0.0.0:8888，与Python脚本对应)
    FIPv4Address Address = FIPv4Address::Any;
    const uint16 Port = 8888;
    FIPv4Endpoint Endpoint(Address, Port);

    // 创建UDP Socket
    ListenSocket = SocketSubsystem->CreateSocket(NAME_DGram, TEXT("UDPSocket"), false);
    if (!ListenSocket)
    {
        UE_LOG(LogTemp, Warning, TEXT("UDPReceiver: Failed to create socket"));
        return false;
    }

    // 可以将Socket设为非阻塞，避免无数据时对引擎造成阻塞
    ListenSocket->SetNonBlocking(true);

    // 绑定Socket到端点
    bool bBindSuccess = ListenSocket->SetReuseAddr(true)
        && ListenSocket->SetRecvErr()
        && ListenSocket->Bind(*Endpoint.ToInternetAddr());

    if (!bBindSuccess)
    {
        UE_LOG(LogTemp, Warning, TEXT("UDPReceiver: Failed to bind socket to port %d"), Port);
        CloseSocket();
        return false;
    }

    // 创建异步接收器
    UDPReceiver = MakeShared<FUdpSocketReceiver>(
        ListenSocket,
        FTimespan::FromMilliseconds(100),
        TEXT("UDPReceiverThread")
    );

    if (UDPReceiver.IsValid())
    {
        // 将回调绑定到本对象
        UDPReceiver->OnDataReceived().BindUObject(this, &AUDPReceiver::Recv);
        UDPReceiver->Start();

        UE_LOG(LogTemp, Log, TEXT("UDPReceiver: Successfully listening on port %d"), Port);
        return true;
    }

    UE_LOG(LogTemp, Warning, TEXT("UDPReceiver: Failed to create receiver"));
    CloseSocket();
    return false;
}

void AUDPReceiver::Tick(float DeltaTime)
{
    Super::Tick(DeltaTime);
    // 在Tick中不执行任何阻塞或复杂的网络操作
    // 如果需要定期检查Socket是否失效，可考虑在此处做自动重连的判断
}

void AUDPReceiver::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
    // 清理资源，停止线程，关闭Socket
    CloseSocket();
    Super::EndPlay(EndPlayReason);
}

void AUDPReceiver::CloseSocket()
{
    // 安全地关闭接收器
    if (UDPReceiver.IsValid())
    {
        // 先请求停止
        UDPReceiver->Stop();
        // UE5 中并没有公开的 IsThreadActive() 接口可用，这里只做简单的 Stop，再重置
        UDPReceiver.Reset();
    }

    // 安全地关闭Socket
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

void AUDPReceiver::Recv(const FArrayReaderPtr& ArrayReaderPtr, const FIPv4Endpoint& EndPt)
{
    // 如果Actor已经被销毁或正在被销毁，或Socket已经关闭，则不处理
    // 对于 UE5，可使用 IsActorBeingDestroyed() 来判断
    if (!IsValid(this) || IsActorBeingDestroyed() || !ListenSocket || !UDPReceiver.IsValid())
    {
        return;
    }

    // 1. 首先进行安全检查
    if (!ArrayReaderPtr.IsValid())
    {
        UE_LOG(LogTemp, Verbose, TEXT("UDPReceiver: Received invalid data pointer"));
        return;
    }

    // 2. 检查数据大小
    int32 ReceivedSize = ArrayReaderPtr->Num();
    if (ReceivedSize != ExpectedDataSize)
    {
        UE_LOG(LogTemp, Verbose, TEXT("UDPReceiver: Data size mismatch. Expected: %d, Received: %d"),
            ExpectedDataSize, ReceivedSize);
        return;
    }

    // 3. 解析数据（已验证过长度，可以安全读取）
    ArrayReaderPtr->Seek(0);

    float x = 0.f, y = 0.f, z = 0.f;
    float pitch = 0.f, yaw = 0.f, roll = 0.f;

    *ArrayReaderPtr << x;
    *ArrayReaderPtr << y;
    *ArrayReaderPtr << z;
    *ArrayReaderPtr << pitch;
    *ArrayReaderPtr << yaw;
    *ArrayReaderPtr << roll;

    // 4. 验证数据
    if (IsValidData(x, y, z, pitch, yaw, roll))
    {
        FScopeLock Lock(&DataMutex);
        LatestLocation = FVector(x, y, z);
        LatestRotation = FRotator(pitch, yaw, roll);
        bHasReceivedData = true;
    }
    else
    {
        UE_LOG(LogTemp, Verbose, TEXT("UDPReceiver: Invalid transform data received"));
    }
}

bool AUDPReceiver::IsValidData(float x, float y, float z, float pitch, float yaw, float roll)
{
    // 使用 FMath::IsFinite 统一检查 NaN 和无穷大
    if (!FMath::IsFinite(x) || !FMath::IsFinite(y) || !FMath::IsFinite(z) ||
        !FMath::IsFinite(pitch) || !FMath::IsFinite(yaw) || !FMath::IsFinite(roll))
    {
        return false;
    }

    // 检查坐标是否在合理范围
    const float MaxPosition = 1000000.0f;
    if (FMath::Abs(x) > MaxPosition || FMath::Abs(y) > MaxPosition || FMath::Abs(z) > MaxPosition)
    {
        return false;
    }

    return true;
}

bool AUDPReceiver::GetLatestTransformData(FVector& OutLocation, FRotator& OutRotation)
{
    FScopeLock Lock(&DataMutex);
    OutLocation = LatestLocation;
    OutRotation = LatestRotation;
    return bHasReceivedData;
}
