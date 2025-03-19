// MoveComponent_Carbody.h

#pragma once

#include "CoreMinimal.h"
#include "Components/SceneComponent.h"
#include "MoveComponent_Carbody.generated.h"

class AUDPReceiver;

/**
 * 用于示例性驱动车体运动的组件。
 * 会在Tick中向UDPReceiver拉取 FTrainData，并将车体位姿应用到该组件所在Actor。
 */
UCLASS(ClassGroup = (Custom), meta = (BlueprintSpawnableComponent))
class VEHICLE4WDB_API UMoveComponent_Carbody : public USceneComponent
{
    GENERATED_BODY()

public:
    // 构造函数
    UMoveComponent_Carbody();

    // 引用到UDPReceiver，以便拉取最新数据
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "CarbodyMovement")
    AUDPReceiver* UDPReceiverRef;

    // 该组件是否直接使用世界Transform (默认true)
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "CarbodyMovement")
    bool bUseWorldTransform;

    // 插值速度 (越大 -> 跟随目标越快；越小 -> 越平稳、滞后)
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "CarbodyMovement")
    float InterpSpeed;

protected:
    virtual void BeginPlay() override;
    virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;

private:
    // 设置默认变换
    void SetupDefaultTransform();

    // 应用变换到Actor
    void ApplyTransform();

    // 是否已初始化插值（第一次获取数据时会直接跳到目标位置）
    bool bInitialized;

    // 默认位置和旋转
    FVector DefaultLocation;
    FRotator DefaultRotation;

    // 用于存储当前帧插值后的位姿
    FVector CurrentLocation;
    FRotator CurrentRotation;
};