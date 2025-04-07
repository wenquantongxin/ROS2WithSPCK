// UDPReceiver.cpp

#include "UDPReceiver.h"
#include "Engine/Engine.h"    // UE_LOG 等
#include "Sockets.h"
#include "SocketSubsystem.h"
#include "IPAddress.h"
#include "Common/UdpSocketBuilder.h"
#include "Common/UdpSocketReceiver.h"
#include "Misc/ScopeLock.h"
#include "HAL/UnrealMemory.h"
#include "Math/UnrealMathUtility.h"
#include "TrainData.h"        // 需要包含你的 FTrainData 结构

/*

 S1 UDPReceiver 中乘以 100（米→厘米）
 S2 对姿态角乘以 -AngleScale（即先弧度转度，再取负号），
 S3 映射到“Z 向上 + 左手系 + 单位 cm + (Yaw、Pitch、Roll) - UE 的惯例

*/

// ---------------------- 转换常量部分 ----------------------
// 如果外部位置是以 "米" 为单位 -> UE 中默认为 "厘米"：     1m = 100cm
static const float DistanceScale = 100.f;

// 外部是 "弧度" => 需要 (180.f / PI)
static const float AngleScale = 180.f / PI;

// 车体默认高度
static const float cb_hc = 1.2f; // 1.2f;

// 转向架提高高度
static const float bg_hc = 0.0; 

// 轮对提高高度
static const float ws_hc = 0.0; 

//----------------------------------------------------------

AUDPReceiver::AUDPReceiver()
{
    PrimaryActorTick.bCanEverTick = true;

    ListenSocket = nullptr;
    UDPReceiver = nullptr;

    // 96 (原77) 个 double，每个 double 8 字节 -> 616 字节
    ExpectedDataSize = 96 * sizeof(double);

    // 刚开始还未收到任何有效数据
    bHasValidData = false;
}

void AUDPReceiver::BeginPlay()
{
    Super::BeginPlay();

    // 初始化 Socket 与 UDPReceiver
    InitializeUDPReceiver();
}

bool AUDPReceiver::InitializeUDPReceiver()
{
    CloseSocket(); // 确保之前的资源清理

    ISocketSubsystem* SocketSubsystem = ISocketSubsystem::Get(PLATFORM_SOCKETSUBSYSTEM);
    if (!SocketSubsystem)
    {
        UE_LOG(LogTemp, Warning, TEXT("UDPReceiver: Socket subsystem unavailable"));
        return false;
    }

    // 绑定地址(0.0.0.0:10099 之类)，视你的需求修改端口
    FIPv4Address Addr = FIPv4Address::Any;
    const uint16 Port = 10099; // 与 ROS2 节点发送端口匹配
    FIPv4Endpoint Endpoint(Addr, Port);

    // 创建 UDP Socket
    ListenSocket = SocketSubsystem->CreateSocket(NAME_DGram, TEXT("UDPReceiverSocket"), false);
    if (!ListenSocket)
    {
        UE_LOG(LogTemp, Warning, TEXT("UDPReceiver: Failed to create socket"));
        return false;
    }

    // 设为非阻塞
    ListenSocket->SetNonBlocking(true);

    // 绑定本地端口
    bool bBindOk = ListenSocket->SetReuseAddr(true) &&
        ListenSocket->SetRecvErr() &&
        ListenSocket->Bind(*Endpoint.ToInternetAddr());
    if (!bBindOk)
    {
        UE_LOG(LogTemp, Warning, TEXT("UDPReceiver: Failed to bind port %d"), Port);
        CloseSocket();
        return false;
    }

    // 创建异步接收器
    UDPReceiver = MakeShared<FUdpSocketReceiver>(
        ListenSocket,
        FTimespan::FromMilliseconds(10),
        TEXT("UDPReceiverThread")
    );

    if (UDPReceiver.IsValid())
    {
        // 绑定回调
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
    // 如果需要检测 Socket 状态可在此处做
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

// 接收回调
void AUDPReceiver::Recv(const FArrayReaderPtr& ArrayReaderPtr, const FIPv4Endpoint& EndPt)
{
    if (!IsValid(this) || !ListenSocket || !UDPReceiver.IsValid())
    {
        return; // Actor被销毁或Socket无效
    }

    // 1) 数据指针和长度检查
    if (!ArrayReaderPtr.IsValid())
        return;

    int32 ReceivedSize = ArrayReaderPtr->Num();
    if (ReceivedSize != ExpectedDataSize)
    {
        UE_LOG(LogTemp, Warning, TEXT("UDPReceiver: Data size mismatch. Expect %d, got %d"),
            ExpectedDataSize, ReceivedSize);
        return;
    }

    // 2) 解析 91(原77) 个 double
    const double* rawPtr = reinterpret_cast<const double*>(ArrayReaderPtr->GetData());
    FTrainData newData; // 临时存储

    // (1) ROS 2 的内部时间戳, (2) y_spcktime, (3) y_cb_vx
    int idx = 0;
    newData.SimTime = rawPtr[idx++];
    newData.SPCKTime =  rawPtr[idx++];
    newData.CarBodyVx = rawPtr[idx++];

    // (4~9) 车体(X, Y, Z, roll, yaw, pitch)
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

    // (10~17) 8个轮的旋转速度 rotw
    for (int i = 0; i < 8; i++)
    {
        newData.WheelsRotSpeed[i] = rawPtr[idx++];
    }

    // (18~23) 转向架 #1 (X,Y,Z, roll,yaw,pitch)
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

    // (24~29) 转向架 #2
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

    // (30~53) 4个轮对 (X, Y, Z, roll, yaw, pitch)
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

    // (54~61) 8个车轮转角 rota
    for (int i = 0; i < 8; i++)
    {
        double wrota = rawPtr[idx++];
        wrota *= AngleScale;
        newData.WheelsRotation[i] = wrota;
    }

    // (62~69) 8个杠杆 pitch
    for (int i = 0; i < 8; i++)
    {
        double barPitch = rawPtr[idx++];
        barPitch *= AngleScale;
        newData.BarsPitch[i] = barPitch;
    }

    // (70~73) 4个轮对 vy
    for (int i = 0; i < 4; i++)
    {
        newData.WheelsetVY[i] = rawPtr[idx++];
    }

    // (74~77) 4个轮对 vyaw
    for (int i = 0; i < 4; i++)
    {
        newData.WheelsetVYaw[i] = rawPtr[idx++];
    }

    // (78~79) 平稳性指标, (80~83) 轮轨力, (84~91) 反射数据-控制转矩
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

    // (92) 运行里程
    newData.TrackS = rawPtr[idx++];

    // (93~94) Sperling指标, (95~96) 脱轨系数
    for (int i = 0; i < 2; i++)
    {
        newData.SperlingYZ[i] = rawPtr[idx++];
    }
    for (int i = 0; i < 2; i++)
    {
        newData.DerailmentIndex[i] = rawPtr[idx++];
    }

    // 如果要检查 NaN / 无穷大等，可使用 IsValidData
    if (!IsValidData(cbX, cbY, cbZ, cbRoll, cbYaw, cbPitch))
    {
        UE_LOG(LogTemp, Verbose, TEXT("Invalid carbody transform data. ignoring."));
        return;
    }

    // 3) 写入成员变量 (线程安全保护)
    {
        FScopeLock Lock(&DataMutex);
        LatestTrainData = newData;

        // ********* 关键逻辑：成功解析到一次有效数据后，置为true *********
        bHasValidData = true;
    }
}

bool AUDPReceiver::IsValidData(double x, double y, double z, double roll, double yaw, double pitch)
{
    // 防止 NaN 或 无穷大
    if (!FMath::IsFinite((float)x) ||
        !FMath::IsFinite((float)roll))
    {
        return false;
    }
    // 位置超出极限也可视作异常
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

    // 当 bHasValidData == false 时，意味着尚未收到过任何有效数据
    // 这时返回 false，让上层组件得以使用“默认位置”。
    return bHasValidData;
}
