// MoveComponent_Bogie.h

#pragma once
#include "CoreMinimal.h"
#include "Components/SceneComponent.h"
#include "MoveComponent_Bogie.generated.h"
class AUDPReceiver;
/**
 * 用于驱动转向架运动的组件。
 * 通过BogieIndex参数可配置为前部或后部转向架。
 */
UCLASS(ClassGroup = (Custom), meta = (BlueprintSpawnableComponent))
class VEHICLE4WDB_API UMoveComponent_Bogie : public USceneComponent
{
    GENERATED_BODY()
public:
    // 构造函数
    UMoveComponent_Bogie();
    // 引用到UDPReceiver，以便拉取最新数据
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "BogieMovement")
    AUDPReceiver* UDPReceiverRef;
    // 转向架索引 (0=前部转向架, 1=后部转向架)
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "BogieMovement", meta = (ClampMin = "0", ClampMax = "1", ExposeOnSpawn = true))
    int32 BogieIndex;
    // 该组件是否直接使用世界Transform (默认true)
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "BogieMovement")
    bool bUseWorldTransform;
    // 插值速度 (越大 -> 跟随目标越快；越小 -> 越平稳、滞后)
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "BogieMovement")
    float InterpSpeed;

    // 设置转向架索引并更新默认变换
    UFUNCTION(BlueprintCallable, Category = "BogieMovement")
    void SetBogieIndex(int32 NewIndex);

protected:
    virtual void BeginPlay() override;
    virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;
private:
    // 是否已初始化插值（第一次获取数据时会直接跳到目标位置）
    bool bInitialized;
    // 用于存储当前帧插值后的位姿
    FVector CurrentLocation;
    FRotator CurrentRotation;
    // 默认的转向架位置（当未接收UDP数据时使用）
    FVector DefaultLocation;
    // 默认的转向架旋转（当未接收UDP数据时使用）
    FRotator DefaultRotation;
    // 根据BogieIndex设置默认位置和旋转
    void SetupDefaultTransform();
};