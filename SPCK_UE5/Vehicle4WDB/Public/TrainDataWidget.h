// File: TrainDataWidget.h
#pragma once

#include "CoreMinimal.h"
#include "Blueprint/UserWidget.h"
#include "TrainDataWidget.generated.h"

/**
 * ������ʾ�г������ٶȵļ���UI Widget
 */
UCLASS()
class VEHICLE4WDB_API UTrainDataWidget : public UUserWidget
{
    GENERATED_BODY()

public:
    // �����ⲿ(TrainHUDActor)���ã�ÿ�δ���cm/s�ٶ�
    UFUNCTION(BlueprintCallable, Category = "TrainData")
    void UpdateSpeed(float SpeedCmS);

    // �����ֵ��ʾ
    UFUNCTION(BlueprintCallable, Category = "TrainData")
    void UpdateTrackS(float InTrackS);

protected:
    virtual void NativeConstruct() override;

    // �󶨵�UMG�е�TextBlock������Ҫ��ӦDesigner�еĿؼ�����
    UPROPERTY(meta = (BindWidget))
    class UTextBlock* Speed_TextBlock;

    // �Ƿ�ʹ�� m/s�������� km/h
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "TrainData")
    bool bUseMPS;

    // ������̵�TextBlock
    UPROPERTY(meta = (BindWidget))
    UTextBlock* TrackS_TextBlock;

private:
    // ��cm/sת��Ϊ��ʾ��λ( km/h �� m/s )
    float ConvertSpeed(float SpeedCmS) const;
};
