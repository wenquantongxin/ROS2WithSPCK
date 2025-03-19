#pragma once

#include "CoreMinimal.h"
#include "Components/SceneComponent.h"
#include "MoveComponent_Carbody.generated.h"

class AUDPReceiver;

/**
 * ����ʾ�������������˶��������
 * ����Tick����UDPReceiver��ȡ FTrainData����������λ��Ӧ�õ����������Actor��
 */
UCLASS(ClassGroup = (Custom), meta = (BlueprintSpawnableComponent))
class VEHICLE4WDB_API UMoveComponent_Carbody : public USceneComponent
{
    GENERATED_BODY()

public:
    // ���캯��
    UMoveComponent_Carbody();

    // ���õ�UDPReceiver���Ա���ȡ��������
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "CarbodyMovement")
    AUDPReceiver* UDPReceiverRef;

    // ������Ƿ�ֱ��ʹ������Transform (Ĭ��true)
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "CarbodyMovement")
    bool bUseWorldTransform;

    // ��ֵ�ٶ� (Խ�� -> ����Ŀ��Խ�죻ԽС -> Խƽ�ȡ��ͺ�)
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "CarbodyMovement")
    float InterpSpeed;

protected:
    virtual void BeginPlay() override;
    virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;

private:
    // �Ƿ��ѳ�ʼ����ֵ����һ�λ�ȡ����ʱ��ֱ������Ŀ��λ�ã�
    bool bInitialized;

    // ���ڴ洢��ǰ֡��ֵ���λ��
    FVector  CurrentLocation;
    FRotator CurrentRotation;
};
