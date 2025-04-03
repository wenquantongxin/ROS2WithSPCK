// File: TrainDataWidget.h
#pragma once

#include "CoreMinimal.h"
#include "Blueprint/UserWidget.h"
#include "TrainDataWidget.generated.h"

/**
 * 负责显示列车纵向速度的简易UI Widget
 */
UCLASS()
class VEHICLE4WDB_API UTrainDataWidget : public UUserWidget
{
    GENERATED_BODY()

public:
    // 用于外部(TrainHUDActor)调用，每次传入cm/s速度
    UFUNCTION(BlueprintCallable, Category = "TrainData")
    void UpdateSpeed(float SpeedCmS);

    // 里程数值显示
    UFUNCTION(BlueprintCallable, Category = "TrainData")
    void UpdateTrackS(float InTrackS);

protected:
    virtual void NativeConstruct() override;

    // 绑定到UMG中的TextBlock，名称要对应Designer中的控件名字
    UPROPERTY(meta = (BindWidget))
    class UTextBlock* Speed_TextBlock;

    // 是否使用 m/s，否则用 km/h
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "TrainData")
    bool bUseMPS;

    // 纵向里程的TextBlock
    UPROPERTY(meta = (BindWidget))
    UTextBlock* TrackS_TextBlock;

private:
    // 将cm/s转换为显示单位( km/h 或 m/s )
    float ConvertSpeed(float SpeedCmS) const;
};
