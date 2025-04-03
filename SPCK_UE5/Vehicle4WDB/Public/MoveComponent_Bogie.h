// MoveComponent_Bogie.h
#pragma once
#include "CoreMinimal.h"
#include "Components/SceneComponent.h"
#include "MoveComponent_Bogie.generated.h"
class AUDPReceiver;
/**
 * ��������ת����˶��������
 * ͨ��BogieIndex����������Ϊǰ�����ת��ܡ�
 */
UCLASS(ClassGroup = (Custom), meta = (BlueprintSpawnableComponent))
class VEHICLE4WDB_API UMoveComponent_Bogie : public USceneComponent
{
    GENERATED_BODY()
public:
    // ���캯��
    UMoveComponent_Bogie();
    // ���õ�UDPReceiver���Ա���ȡ��������
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "BogieMovement")
    AUDPReceiver* UDPReceiverRef;
    // ת������� (0=ǰ��ת���, 1=��ת���)
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "BogieMovement", meta = (ClampMin = "0", ClampMax = "1", ExposeOnSpawn = true))
    int32 BogieIndex;
    // ������Ƿ�ֱ��ʹ������Transform (Ĭ��true)
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "BogieMovement")
    bool bUseWorldTransform;
    // ��ֵ�ٶ� (Խ�� -> ����Ŀ��Խ�죻ԽС -> Խƽ�ȡ��ͺ�)
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "BogieMovement")
    float InterpSpeed;

    // ����ת�������������Ĭ�ϱ任
    UFUNCTION(BlueprintCallable, Category = "BogieMovement")
    void SetBogieIndex(int32 NewIndex);

protected:
    virtual void BeginPlay() override;
    virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;
private:
    // �Ƿ��ѳ�ʼ����ֵ����һ�λ�ȡ����ʱ��ֱ������Ŀ��λ�ã�
    bool bInitialized;
    // ���ڴ洢��ǰ֡��ֵ���λ��
    FVector CurrentLocation;
    FRotator CurrentRotation;
    // Ĭ�ϵ�ת���λ�ã���δ����UDP����ʱʹ�ã�
    FVector DefaultLocation;
    // Ĭ�ϵ�ת�����ת����δ����UDP����ʱʹ�ã�
    FRotator DefaultRotation;
    // ����BogieIndex����Ĭ��λ�ú���ת
    void SetupDefaultTransform();
};