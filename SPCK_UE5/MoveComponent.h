#pragma once

#include "CoreMinimal.h"
#include "Components/SceneComponent.h"
#include "MoveComponent.generated.h"

class AUDPReceiver;

/**
 * 自定义移动组件，可让小球根据外部UDP数据进行移动(与旋转)
 */
UCLASS(ClassGroup = (Custom), meta = (BlueprintSpawnableComponent))
class MYDEMO_0309A_API UMoveComponent : public USceneComponent
{
    GENERATED_BODY()

public:
    UMoveComponent();

protected:
    virtual void BeginPlay() override;

public:
    virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;

private:
    // 起始位置
    FVector StartRelativeLocation;

    // 移动速度(仅用于演示平移)
    UPROPERTY(EditAnywhere, Category = "Movement")
    float speed = 10.0f;

    // 组件内部的“目标位置”(可与外部UDP数据进行对接)
    UPROPERTY(EditAnywhere, Category = "Movement")
    FVector EndRelativeLocation;

    // 当前记录位置
    FVector CurrentLocation;

    // 距离小于此阈值时停止移动
    float stoppingDistance = 1.0f;

public:
    // 指向场景中放置的UDPReceiver Actor（在蓝图中设定）
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "UDP")
    AUDPReceiver* UDPReceiverActor;
};
