#include "mbed.h"
#include "firstpenguin.hpp"

// エンコーダーのピンを定義
DigitalIn encoderPinA(D2);
DigitalIn encoderPinB(D3);

// エンコーダーの状態を追跡するための変数
int encoderPos = 0;
int encoderLastPinA = 0;

void readEncoder()
{
    int encoderPinAValue = encoderPinA.read();
    if ((encoderLastPinA == 0) && (encoderPinAValue == 1)) {
        if (encoderPinB.read() == 0) {
            encoderPos = encoderPos + 1;
        } else {
            encoderPos = encoderPos - 1;
        }
    }
    encoderLastPinA = encoderPinAValue;
}

constexpr uint32_t can_id = 30;
BufferedSerial pc(USBTX, USBRX, 250000); // パソコンとのシリアル通信
CANMessage msg;
CAN can(PA_11, PA_12, (int)1e6);
CAN can2(PB_12, PB_13, (int)1e6);
int16_t pwm0[4] = {0, 0, 0, 0}; // モータドライバ1
Timer timer;
FirstPenguin penguin{can_id, can};
int forwardSpeed = 8000; // 前進速度
int sidewaysSpeed = 5000; // 左右移動速度

double targetXDistance = 1000.0; // 目標x方向の距離 (mm)
double targetYDistance = 500.0; // 目標y方向の距離 (mm)
double wheelDiameter = 100.0; // ホイールの直径 (mm)
double encoderCountsPerRevolution = 1000.0; // 1回転あたりのエンコーダーカウント

int main()
{
    printf("program start\n");
    double xDistanceTravelled = 0.0; // x方向に移動した距離
    double yDistanceTravelled = 0.0; // y方向に移動した距離

    while (xDistanceTravelled < targetXDistance && yDistanceTravelled < targetYDistance)
    {
        // エンコーダーからの読み取り値を取得
        readEncoder();
        double encoderCounts = encoderPos;

        // エンコーダーカウントを距離に変換
        xDistanceTravelled = (encoderCounts / encoderCountsPerRevolution) * (M_PI * wheelDiameter);
        yDistanceTravelled = (encoderCounts / encoderCountsPerRevolution) * (M_PI * wheelDiameter);

        // 前進値と左右の移動値を指定してロボットを制御
        // メカナムホイールの特性を利用して、各ホイールに対して独立した速度を設定
        penguin.pwm[0] = forwardSpeed + sidewaysSpeed; // 左前ホイール
        penguin.pwm[1] = forwardSpeed - sidewaysSpeed; // 右前ホイール
        penguin.pwm[2] = forwardSpeed - sidewaysSpeed; // 左後ホイール
        penguin.pwm[3] = forwardSpeed + sidewaysSpeed; // 右後ホイール

        // モーターのPWM値を送信
        penguin.send();
    }

    // 目標距離に達したら停止
    penguin.pwm[0] = 0;
    penguin.pwm[1] = 0;
    penguin.pwm[2] = 0;
    penguin.pwm[3] = 0;
    penguin.send();

    printf("Reached target distance\n");
}