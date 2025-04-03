// File: TrainHUDActor.h
#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "TrainData.h"    // 您已有的FTrainData结构
#include "TrainHUDActor.generated.h"

class UTrainDataWidget;
class AUDPReceiver;

/**
 * 该Actor在BeginPlay时创建HUD Widget，Tick里实时从AUDPReceiver获取CarBodyVx并更新UI。
 */
UCLASS()
class VEHICLE4WDB_API ATrainHUDActor : public AActor
{
    GENERATED_BODY()

public:
    ATrainHUDActor();

    // 是否自动在场景中查找第一个AUDPReceiver。如果为false，则使用ManualUDPReceiverRef。
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Train HUD")
    bool bAutoFindUDPReceiver;

    // 手动指定的UDPReceiver引用（仅当 bAutoFindUDPReceiver=false 时生效）
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Train HUD", meta = (EditCondition = "!bAutoFindUDPReceiver"))
    AUDPReceiver* ManualUDPReceiverRef;

    // 指定要创建的Widget类（例如可以是UTrainDataWidget蓝图或者C++子类）
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Train HUD")
    TSubclassOf<UTrainDataWidget> TrainDataWidgetClass;

protected:
    virtual void BeginPlay() override;
    virtual void Tick(float DeltaTime) override;

private:
    // 指向实际创建并AddToViewport的Widget实例
    UPROPERTY()
    UTrainDataWidget* TrainDataWidgetInstance;

    // 最终使用的UDPReceiver引用
    UPROPERTY()
    AUDPReceiver* UDPReceiverPtr;
};
