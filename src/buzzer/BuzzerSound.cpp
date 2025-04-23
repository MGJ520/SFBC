#include "BuzzerSound.h"
#include "Arduino.h"

#define ledc_channel 5

void BuzzerSoundsClass::Init(int aBuzzerPin) {
    buzzerPin = aBuzzerPin;
    ledcSetup(ledc_channel, 1000, 10);
    ledcAttachPin(buzzerPin, ledc_channel); // pin, channel
}

///////////////////////////////////////////////////////////////////
//-- SOUNDS -----------------------------------------------------//
///////////////////////////////////////////////////////////////////

void BuzzerSoundsClass::_tone(unsigned int noteFrequency, long noteDuration, int silentDuration) const {
    if (silentDuration == 0) {
        silentDuration = 1;
    }
    tone(buzzerPin, noteFrequency, noteDuration);
    delay(noteDuration); // milliseconds
    delay(silentDuration);
}

void BuzzerSoundsClass::bendTones(float initFrequency, float finalFrequency, float prop, long noteDuration,
                                  int silentDuration) {

    // Examples:
    //   bendTones (880, 2093, 1.02, 18, 1);
    //   bendTones (note_A5, note_C7, 1.02, 18, 0);

    if (silentDuration == 0) {
        silentDuration = 1;
    }

    if (initFrequency < finalFrequency) {
        for (int i = initFrequency; i < finalFrequency; i = i * prop) {
            _tone(i, noteDuration, silentDuration);
        }
    } else {

        for (int i = initFrequency; i > finalFrequency; i = i / prop) {
            _tone(i, noteDuration, silentDuration);
        }
    }
}

void BuzzerSoundsClass::play(int soundName) {
    switch (soundName) {

        case S_CONNECTION:
            _tone(NOTE_E5, 50, 30);
            _tone(NOTE_E6, 55, 25);
            _tone(NOTE_A6, 60, 10);
            break;

        case S_DISCONNECTION:
            _tone(NOTE_E5, 50, 30);
            _tone(NOTE_A6, 55, 25);
            _tone(NOTE_E6, 50, 60);
            break;

        case S_BUTTON_PUSHED:
            bendTones(NOTE_E6, NOTE_G6, 1.03, 20, 2);
            delay(30);
            bendTones(NOTE_E6, NOTE_D7, 1.04, 10, 2);
            break;

        case S_MODE1:
            bendTones(NOTE_E6, NOTE_A6, 1.02, 30, 10); // 1318.51 to 1760
            break;

        case S_MODE2:
            bendTones(NOTE_G6, NOTE_D7, 1.03, 30, 10); // 1567.98 to 2349.32
            break;

        case S_MODE3:
            _tone(NOTE_E6, 50, 100); // D6
            _tone(NOTE_G6, 50, 80);  // E6
            _tone(NOTE_D7, 300, 0);  // G6
            break;

        case S_SURPRISE:
            bendTones(800, 2150, 1.02, 10, 1);
            bendTones(2149, 800, 1.03, 7, 1);
            break;

        case S_JUMP:
            bendTones(880, 2000, 1.04, 8, 3); // A5 = 880
            delay(200);
            break;

        case S_OHOOH:
            bendTones(880, 2000, 1.04, 8, 3); // A5 = 880
            delay(200);

            for (int i = 880; i < 2000; i = i * 1.04) {
                _tone(NOTE_B5, 5, 10);
            }
            break;

        case S_OHOOH2:
            bendTones(1880, 3000, 1.03, 8, 3);
            delay(200);

            for (int i = 1880; i < 3000; i = i * 1.03) {
                _tone(NOTE_C6, 10, 10);
            }
            break;

        case S_CUDDLY:
            bendTones(700, 900, 1.03, 16, 4);
            bendTones(899, 650, 1.01, 18, 7);
            break;

        case S_SLEEPING:
            bendTones(100, 500, 1.04, 10, 10);
            delay(500);
            bendTones(400, 100, 1.04, 10, 1);
            break;

        case S_HAPPY:
            bendTones(1500, 2500, 1.05, 20, 8);
            bendTones(2499, 1500, 1.05, 25, 8);
            break;

        case S_SUPER_HAPPY:
            bendTones(2000, 6000, 1.05, 8, 3);
            delay(50);
            bendTones(5999, 2000, 1.05, 13, 2);
            break;

        case S_HAPPY_SHORT:
            bendTones(1500, 2000, 1.05, 15, 8);
            delay(100);
            bendTones(1900, 2500, 1.05, 10, 8);
            break;

        case S_SAD:
            bendTones(880, 669, 1.02, 20, 200);
            break;

        case S_CONFUSED:
            bendTones(1000, 1700, 1.03, 8, 2);
            bendTones(1699, 500, 1.04, 8, 3);
            bendTones(1000, 1700, 1.05, 9, 10);
            break;

        case S_FART1:
            bendTones(1600, 3000, 1.02, 2, 15);
            break;

        case S_FART2:
            bendTones(2000, 6000, 1.02, 2, 20);
            break;

        case S_FART3:
            bendTones(1600, 4000, 1.02, 2, 20);
            bendTones(4000, 3000, 1.02, 2, 20);
            break;

        case PIRATES:
            // This is funny but very experimental
            for (int i = 0; i < 19; i++) { // 203 is the total number of music notes in the song
                int wait = duration[i] * songspeed;
                _tone(notes[i], wait, 0); // tone(pin,frequency,duration)
            }
            break;

        case S_BEEP:
            // This is funny but very experimental
            _tone(800, 200, 0); // tone(pin,frequency,duration)
            break;

        case S_SIREN:
            _tone(400, 200, 0);
            _tone(800, 200, 0);
            delay(50);
            break;
        case TINY_WORLD: {
            int buzzerPin = 9;    // 蜂鸣器连接引脚
            int tempo = 120;       // 速度（根据实际曲谱调整）
            int beatDuration = 60000 / tempo; // 四分音符基准时长

            // C大调音高映射（简谱数字 -> 频率）
            // 1=C4, 2=D4, 3=E4, 4=F4, 5=G4, 6=A4, 7=B4, i=C5
            float melody[] = {
                    NOTE_C4, NOTE_C4, NOTE_G4, NOTE_G4,   // "世" 1 1 5 5
                    NOTE_A4, NOTE_A4, NOTE_G4, 0,         // "界" 6 6 5 -
                    NOTE_F4, NOTE_F4, NOTE_E4, NOTE_E4,   // "真" 4 4 3 3
                    NOTE_D4, NOTE_D4, NOTE_C4, 0,         // "细" 2 2 1 -

                    NOTE_G4, NOTE_G4, NOTE_F4, NOTE_F4,   // "小" 5 5 4 4
                    NOTE_E4, NOTE_E4, NOTE_D4, 0,         // "小" 3 3 2 -
                    NOTE_G4, NOTE_G4, NOTE_F4, NOTE_F4,   // "世" 5 5 4 4
                    NOTE_E4, NOTE_E4, NOTE_D4, 0,         // "界" 3 3 2 -

                    NOTE_C4, NOTE_C4, NOTE_G4, NOTE_G4,   // "真" 1 1 5 5
                    NOTE_A4, NOTE_A4, NOTE_G4, 0,         // "细" 6 6 5 -
                    NOTE_F4, NOTE_F4, NOTE_E4, NOTE_E4,   // "小" 4 4 3 3
                    NOTE_D4, NOTE_D4, NOTE_C4, 0          // "妙" 2 2 1 -
            };

            // 节奏系数（1=四分音符，0.5=八分音符，2=二分音符）
            float durations[] = {
                    0.5, 0.5, 0.5, 0.5,   // 四个八分音符
                    0.5, 0.5, 1.0, 1.0,    // 附点节奏处理
                    0.5, 0.5, 0.5, 0.5,
                    0.5, 0.5, 1.0, 1.0,

                    0.5, 0.5, 0.5, 0.5,
                    0.5, 0.5, 1.0, 1.0,
                    0.5, 0.5, 0.5, 0.5,
                    0.5, 0.5, 1.0, 1.0,

                    0.5, 0.5, 0.5, 0.5,
                    0.5, 0.5, 1.0, 1.0,
                    0.5, 0.5, 0.5, 0.5,
                    0.5, 0.5, 1.0, 1.0
            };

            for (int i = 0; i < sizeof(melody)/sizeof(melody[0]); i++) {
                if (melody[i] == 0) {  // 处理休止符
                    delay(beatDuration * durations[i]);
                    continue;
                }

                int noteDuration = beatDuration * durations[i];
                buzzer._tone(melody[i], noteDuration, 10);
                delay(noteDuration * 1.1); // 增加10%间隔防粘连
            }

            // 添加结尾特效
            buzzer.bendTones(NOTE_C5, NOTE_C4, 1.02, 800, 50);
            break;
        }
        case TRUE_GOODNESS_WORLD: {
            int tempo = 1000;         // 中速欢乐地
            int beatDuration = (60000 * 2) / tempo; // 2/4拍基准（四分音符=60000/100=600ms）

            // ================= 前段 F大调 =================
            float melody_F[] = {
                    // A段 (3 4 | 5 3 | 1 2 1 | 1 7)
                    NOTE_A4, NOTE_Bb4, NOTE_C5, NOTE_A4,   // 3 4 | 5 3
                    NOTE_F4, NOTE_G4, NOTE_F4, NOTE_E4,    // 1 2 1 | 1 7

                    // (6 2 3 | 4 3 2 | 5 4 | 3 2)
                    NOTE_D5, NOTE_G4, NOTE_A4,             // 6 2 3
                    NOTE_Bb4, NOTE_A4, NOTE_G4,            // 4 3 2
                    NOTE_C5, NOTE_Bb4,                     // 5 4
                    NOTE_A4, NOTE_G4,                      // 3 2

                    // (i - | i 0 | 3 4 | 5 3)
                    NOTE_F5, 0,                            // i - | i 0
                    NOTE_A4, NOTE_Bb4, NOTE_C5, NOTE_A4,   // 3 4 | 5 3

                    // (i 2 i | i 7 | 7 2 3 | 4 2)
                    NOTE_F5, NOTE_G4, NOTE_F5,             // i 2 i
                    NOTE_F5, NOTE_E4,                      // i 7
                    NOTE_E4, NOTE_G4, NOTE_A4,             // 7 2 3
                    NOTE_Bb4, NOTE_G4,                     // 4 2

                    // (7 i 7 | 6 5 | 5 3 4 | 5 i 2)
                    NOTE_E4, NOTE_F5, NOTE_E4,            // 7 i 7
                    NOTE_D5, NOTE_C5,                      // 6 5
                    NOTE_C5, NOTE_A4, NOTE_Bb4,            // 5 3 4
                    NOTE_C5, NOTE_F5, NOTE_G5,             // 5 i 2

                    // (3 2 i | 6 2 3 | 4 3 2 | 5 4 | 3 2 | i -)
                    NOTE_A4, NOTE_G4, NOTE_F5,             // 3 2 i
                    NOTE_D5, NOTE_G4, NOTE_A4,             // 6 2 3
                    NOTE_Bb4, NOTE_A4, NOTE_G4,            // 4 3 2
                    NOTE_C5, NOTE_Bb4,                     // 5 4
                    NOTE_A4, NOTE_G4,                      // 3 2
                    NOTE_F5, 0                             // i -
            };

            // ================= 后段 G大调 =================
            float melody_G[] = {
                    // (6 6 6 6 | 5 5 5 5 | 5 5 5 5 4 2)
                    NOTE_E5, NOTE_E5, NOTE_E5, NOTE_E5,    // 前6=后5（G大调）
                    NOTE_D5, NOTE_D5, NOTE_D5, NOTE_D5,
                    NOTE_D5, NOTE_D5, NOTE_D5, NOTE_D5, NOTE_C5, NOTE_G4,

                    // (3. 3 5 3 | 4. 4 4 3 2 | 5 4 | 3 2)
                    NOTE_B4, NOTE_B4, NOTE_D5, NOTE_B4,    // 3. 3 5 3
                    NOTE_C5, NOTE_C5, NOTE_C5, NOTE_B4, NOTE_G4, // 4. 4 4 3 2
                    NOTE_D5, NOTE_C5,                      // 5 4
                    NOTE_B4, NOTE_G4,                      // 3 2

                    // (1. 1 3 1 | 2. 2 2 -)
                    NOTE_G4, NOTE_G4, NOTE_B4, NOTE_G4,     // 1. 1 3 1
                    NOTE_A4, NOTE_A4, NOTE_A4, 0            // 2. 2 2 -
            };

            // =============== 节奏系数（单位：拍） ===============
            float rhythm_F[] = {
                    // A段
                    0.5, 0.5, 0.5, 0.5,    0.5, 0.5, 0.5, 0.5,
                    0.5, 0.5, 0.5,         0.5, 0.5, 0.5,
                    1.0, 1.0,              1.0, 1.0,

                    // B段
                    2.0, 2.0,              0.5, 0.5, 0.5, 0.5,
                    0.5, 0.5, 0.5,        0.5, 0.5,
                    0.5, 0.5, 0.5,        0.5, 0.5,

                    // A'段
                    0.5, 0.5, 0.5,        0.5, 0.5, 0.5,
                    0.5, 0.5, 0.5,        1.0, 1.0,
                    1.0, 1.0,             2.0, 2.0
            };

            float rhythm_G[] = {
                    0.25, 0.25, 0.25, 0.25,  0.25, 0.25, 0.25, 0.25,
                    0.25, 0.25, 0.25, 0.25, 0.5, 0.5,

                    1.5, 0.5, 0.5, 0.5,      1.5, 0.5, 0.5, 0.5, 0.5,
                    1.0, 1.0,               1.0, 1.0,

                    1.5, 0.5, 0.5, 0.5,      2.0, 2.0, 2.0, 2.0
            };

            // =============== 播放逻辑 ===============
            // 第一遍播放（A-B-A'）
            for(int repeat=0; repeat<2; repeat++){ // D.C.反复
                // 前段F大调
                for(int i=0; i<sizeof(melody_F)/sizeof(melody_F[0]); i++){
                    if(melody_F[i] == 0) {
                        delay(beatDuration * rhythm_F[i]);
                        continue;
                    }
                    int duration = beatDuration * rhythm_F[i];
                    buzzer._tone(melody_F[i], duration, 10);
                    delay(duration * 1.1);
                }

                // 转调过渡滑音
                buzzer.bendTones(NOTE_F5, NOTE_G4, 1.02, 300, 50);

                // 后段G大调
                for(int i=0; i<sizeof(melody_G)/sizeof(melody_G[0]); i++){
                    if(melody_G[i] == 0) {
                        delay(beatDuration * rhythm_G[i]);
                        continue;
                    }
                    int duration = beatDuration * rhythm_G[i];
                    buzzer._tone(melody_G[i], duration, 10);
                    delay(duration * 1.1);
                }
            }

            // 结尾处理
            buzzer.bendTones(NOTE_G5, NOTE_G3, 1.05, 800, 100);
            break;
        }
    }
}

BuzzerSoundsClass buzzer;
