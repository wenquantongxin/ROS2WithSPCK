// MoveComponent_Wheelset.h

#pragma once
#include "CoreMinimal.h"
#include "Components/SceneComponent.h"
#include "TrainData.h"
#include "MoveComponent_Wheelset.generated.h"

class AUDPReceiver;
class UStaticMeshComponent;
/**
 * 负责接收并插值更新 轮对的世界变换，以及左右车轮的旋转
 */
UCLASS(ClassGroup = (Custom), meta = (BlueprintSpawnableComponent))
class VEHICLE4WDB_API UMoveComponent_Wheelset : public USceneComponent
{
    GENERATED_BODY()
public:
    UMoveComponent_Wheelset();
protected:
    virtual void BeginPlay() override;
public:
    virtual void TickComponent(float DeltaTime, ELevelTick TickType,
        FActorComponentTickFunction* ThisTickFunction) override;
    /** 轮对的索引（0~3），用来决定从 FTrainData 中取哪组位置、旋转 */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Wheelset")
    int32 WheelsetIndex;
    /** 是否使用世界变换（SetActorLocationAndRotation） */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Wheelset")
    bool bUseWorldTransform;
    /** 插值速度 */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Wheelset")
    float InterpSpeed;
    /** 车轮旋转平滑系数，值越大平滑效果越明显 */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Wheelset")
    float WheelRotationSmoothFactor;
    /** 数据超时阈值（秒），超过此值将视为数据流已停止 */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Wheelset")
    float DataTimeoutThreshold;
    /** 指向 UDPReceiver，获取最新 TrainData */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Wheelset")
    AUDPReceiver* UDPReceiverRef;
    /** 当前插值后的世界位置 */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Wheelset")
    FVector CurrentLocation;
    /** 当前插值后的世界旋转 */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Wheelset")
    FRotator CurrentRotation;
    /**
     * 指向本蓝图中（或本Actor中）用于显示"左车轮"的StaticMeshComponent，
     * 以便在C++的Tick里直接 SetRelativeRotation。
     */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Wheelset")
    UStaticMeshComponent* LeftWheelMesh;
    /**
     * 指向本蓝图中（或本Actor中）用于显示"右车轮"的StaticMeshComponent，
     * 以便在C++的Tick里直接 SetRelativeRotation。
     */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Wheelset")
    UStaticMeshComponent* RightWheelMesh;

    /**
     * 指向本蓝图中（或本Actor中）用于显示"左杠杆"的StaticMeshComponent，
     * 以便在C++的Tick里直接 SetRelativeRotation。
     */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Wheelset")
    UStaticMeshComponent* LeftBarMesh;

    /**
     * 指向本蓝图中（或本Actor中）用于显示"右杠杆"的StaticMeshComponent，
     * 以便在C++的Tick里直接 SetRelativeRotation。
     */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Wheelset")
    UStaticMeshComponent* RightBarMesh;

    /**
     * 通过设置新的索引来变更默认变换等
     */
    UFUNCTION(BlueprintCallable, Category = "Wheelset")
    void SetWheelsetIndex(int32 NewIndex);
protected:
    /** 用来计算默认位置和旋转的辅助函数 */
    void SetupDefaultTransform();
    /** 默认初始世界位置（根据WheelsetIndex得来） */
    FVector DefaultLocation;
    /** 默认初始世界旋转 */
    FRotator DefaultRotation;
    /** 是否已经从UDP收到了有效数据 */
    bool bInitialized;
    /** 对应的左轮索引、右轮索引（在TrainData.WheelsRotation中的下标） */
    int32 LeftWheelIndex;
    int32 RightWheelIndex;
    /** 对应的左杠杆、右杠杆索引（在TrainData.BarsPitch中的下标） */
    int32 LeftBarIndex;
    int32 RightBarIndex;
    /** 记录蓝图中给 LeftWheelMesh / RightWheelMesh 设定的初始相对旋转，用来和外部角度叠加 */
    FRotator InitialLeftWheelRot;
    FRotator InitialRightWheelRot;
    /** 记录蓝图中设定的杠杆初始相对旋转 */
    FRotator InitialLeftBarRot;
    FRotator InitialRightBarRot;
    /** 左右车轮当前累积旋转角度（度） */
    float AccumLeftWheelRotation;
    float AccumRightWheelRotation;
    /** 左右车轮当前旋转速度（度/秒） */
    float LeftWheelRotSpeed;
    float RightWheelRotSpeed;
    /** 目标旋转速度（平滑过渡用） */
    float TargetLeftWheelRotSpeed;
    float TargetRightWheelRotSpeed;
    /** 左右杠杆当前pitch角度 */
    float CurrentLeftBarPitch;
    float CurrentRightBarPitch;
    /** 上次接收到有效数据的时间累积器 */
    float TimeSinceLastValidData;
    /** 当前数据是否超时 */
    bool bDataTimeout;
    /** 弧度到角度的转换常量 */
    static constexpr float RadToDeg = 57.2957795f;
    /** SPCK到UE5中角度制差值 */
    static constexpr float Deg_SPCK2UE = 270.0f;
private:
    // 用于检测数据是否已更新的成员变量
    double LastSimTime;  // 上一帧的模拟时间
    bool bFirstDataReceived; // 是否已收到第一帧数据
};