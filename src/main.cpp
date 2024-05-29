#include "mbed.h"
#include "firstpenguin.hpp"
BufferedSerial pc(USBTX, USBRX, 250000);
CANMessage msg;
CAN can{PA_11, PA_12, (int)1e6};

// エンコーダーのピンを定義
DigitalIn encoderPinA(D2);
DigitalIn encoderPinB(D3);

// エンコーダーの状態を追跡するための変数
int encoderPos = 0;
int encoderLastPinA = 0;

int auto = 0;

void readEncoder()
{
    int encoderPinAValue = encoderPinA.read();
    if ((encoderLastPinA == 0) && (encoderPinAValue == 1))
    {
        if (encoderPinB.read() == 0)
        {
            encoderPos = encoderPos + 1;
        }
        else
        {
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
int forwardSpeed = 8000;  // 前進速度
int sidewaysSpeed = 5000; // 左右移動速度

double targetXDistance = 1000.0;            // 目標x方向の距離 (mm)
double targetYDistance = 500.0;             // 目標y方向の距離 (mm)
double wheelDiameter = 100.0;               // ホイールの直径 (mm)
double encoderCountsPerRevolution = 1000.0; // 1回転あたりのエンコーダーカウント

void readUntilPipe(char *output_buf, int output_buf_size)
{
    char buf[20];
    int output_buf_index = 0;

    while (1)
    {
        if (pc.readable())
        {
            ssize_t num = pc.read(buf, sizeof(buf) - 1); // -1 to leave space for null terminator
            buf[num] = '\0';

            for (int i = 0; i < num; i++)
            {
                if (buf[i] == '|')
                {
                    output_buf[output_buf_index] = '\0';
                    return;
                }
                else if (buf[i] != '\n' && output_buf_index < output_buf_size - 1)
                { // 改行文字を無視します
                    output_buf[output_buf_index++] = buf[i];
                }
            }
        }
        if (output_buf_index >= output_buf_size - 1) // Prevent buffer overflow
        {
            output_buf[output_buf_index] = '\0';
            return;
        }
    }
}

int main()
{
    char output_buf[20]; // 出力用のバッファを作成します

    while (1)
    {
        readUntilPipe(output_buf, sizeof(output_buf)); // パイプ文字が現れるまでデータを読み取ります

        // 受信したデータが"aiueo"で始まるか、autoが0の場合に以下の処理を行います
        if (strncmp(output_buf, "aiueo", 5) == 0 || auto == 0){
            
            auto = 1; // autoを1に設定します
            printf("program start\n"); // プログラム開始のメッセージを出力します
            double xDistanceTravelled = 0.0; // x方向に移動した距離を初期化します
            double yDistanceTravelled = 0.0; // y方向に移動した距離を初期化します

            // x方向とy方向の移動距離が目標距離に達するまで以下の処理を行います
            while (xDistanceTravelled < targetXDistance && yDistanceTravelled < targetYDistance)
            {
                readEncoder(); // エンコーダーからの読み取り値を取得します
                double encoderCounts = encoderPos; // エンコーダーの位置を取得します

                // エンコーダーカウントを距離に変換します
                xDistanceTravelled = (encoderCounts / encoderCountsPerRevolution) * (M_PI * wheelDiameter);
                yDistanceTravelled = (encoderCounts / encoderCountsPerRevolution) * (M_PI * wheelDiameter);

                // 前進値と左右の移動値を指定してロボットを制御します
                // メカナムホイールの特性を利用して、各ホイールに対して独立した速度を設定します
                penguin.pwm[0] = forwardSpeed + sidewaysSpeed; // 左前ホイール
                penguin.pwm[1] = forwardSpeed - sidewaysSpeed; // 右前ホイール
                penguin.pwm[2] = forwardSpeed - sidewaysSpeed; // 左後ホイール
                penguin.pwm[3] = forwardSpeed + sidewaysSpeed; // 右後ホイール

                // モーターのPWM値を送信します
                penguin.send();
            }

            // 目標距離に達したら停止します
            penguin.pwm[0] = 0;
            penguin.pwm[1] = 0;
            penguin.pwm[2] = 0;
            penguin.pwm[3] = 0;
            penguin.send();

            printf("Reached target distance\n"); // 目標距離に達したことを出力します
            
        }
    }
}