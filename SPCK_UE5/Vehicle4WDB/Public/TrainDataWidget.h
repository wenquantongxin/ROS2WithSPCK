// TrainDataWidget.h
#pragma once
#include "CoreMinimal.h"
#include "Blueprint/UserWidget.h"
#include "Components/TextBlock.h" // 添加UTextBlock的头文件引用
#include "TrainDataWidget.generated.h"
/**
 * 负责显示列车运行信息的 UI Widget
 */
UCLASS()
class VEHICLE4WDB_API UTrainDataWidget : public UUserWidget
{
    GENERATED_BODY()
public:

    // 外部(TrainHUDActor)调用

    // 传入 SPCK 内部仿真时间
    UFUNCTION(BlueprintCallable, Category = "TrainData")
    void UpdateSPCKTime(float SPCK_time);
    // 传入车体纵向 cm/s 速度
    UFUNCTION(BlueprintCallable, Category = "TrainData")
    void UpdateSpeed(float SpeedCmS);
    // 里程数值显示
    UFUNCTION(BlueprintCallable, Category = "TrainData")
    void UpdateTrackS(float InTrackS);

protected:
    virtual void NativeConstruct() override;

    // 是否使用 m/s，否则用 km/h
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "TrainData")
    bool bUseMPS;

    // 依次定义TextBlock,绑定到UMG中的TextBlock，名称要对应Designer中的控件名字
     // 内部时间的TextBlock
    UPROPERTY(meta = (BindWidget))
    UTextBlock* SPCKTime_TextBlock;

    // 纵向速度的TextBlock
    UPROPERTY(meta = (BindWidget))
    UTextBlock* Speed_TextBlock; 

    // 纵向里程的TextBlock
    UPROPERTY(meta = (BindWidget))
    UTextBlock* TrackS_TextBlock;

private:
    // 将cm/s转换为显示单位( km/h 或 m/s )
    float ConvertSpeed(float SpeedCmS) const;
};