#ifndef _OTTO_MOVEMENTS_H_
#define _OTTO_MOVEMENTS_H_

#include <cmath>

#define SERVO_COUNT 6

enum ServoIndex {
    LEFT_LEG = 0,
    RIGHT_LEG = 1,
    LEFT_FOOT = 2,
    RIGHT_FOOT = 3,
    LEFT_HAND = 4,
    RIGHT_HAND = 5
};

enum Direction {
    FORWARD = 1,
    BACKWARD = -1,
    LEFT = 1,
    RIGHT = -1
};

class Otto {
public:
    Otto();
    ~Otto();

    void Init(int left_leg, int right_leg, int left_foot, int right_foot, int left_hand, int right_hand);
    void AttachServos();
    void DetachServos();
    void SetTrims(int left_leg, int right_leg, int left_foot, int right_foot, int left_hand, int right_hand);

    void MoveServos(int time, int servo_target[]);
    void MoveSingle(int position, int servo_number);
    void OscillateServos(int amplitude[SERVO_COUNT], int offset[SERVO_COUNT], int period, double phase_diff[SERVO_COUNT], float cycle = 1);
    void Execute(int amplitude[SERVO_COUNT], int offset[SERVO_COUNT], int period, double phase_diff[SERVO_COUNT], float steps = 1.0);
    void Execute2(int amplitude[SERVO_COUNT], int center_angle[SERVO_COUNT], int period, double phase_diff[SERVO_COUNT], float steps = 1.0);
    
    void Home(bool hands_down = true);
    bool GetRestState();
    void SetRestState(bool state);

    void Jump(float steps, int period);
    void Walk(float steps, int period, int dir, int amount);
    void Turn(float steps, int period, int dir, int amount);
    void Bend(int steps, int period, int dir);
    void ShakeLeg(int steps, int period, int dir);
    void Sit();
    void UpDown(float steps, int period, int height);
    void Swing(float steps, int period, int height);
    void TiptoeSwing(float steps, int period, int height);
    void Jitter(float steps, int period, int height);
    void AscendingTurn(float steps, int period, int height);
    void Moonwalker(float steps, int period, int height, int dir);
    void Crusaito(float steps, int period, int height, int dir);
    void Flapping(float steps, int period, int height, int dir);
    void WhirlwindLeg(float steps, int period, int amplitude);
    
    void HandsUp(int period, int dir);
    void HandsDown(int period, int dir);
    void HandWave(int dir);
    void Windmill(float steps, int period, int amplitude);
    void Takeoff(float steps, int period, int amplitude);
    void Fitness(float steps, int period, int amplitude);
    void Greeting(int dir, float steps);
    void Shy(int dir, float steps);
    void RadioCalisthenics();
    void MagicCircle();
    void Showcase();

    void EnableServoLimit(int diff_limit);
    void DisableServoLimit();

private:
    int servo_pins_[SERVO_COUNT];
    int servo_trim_[SERVO_COUNT];
    int current_positions_[SERVO_COUNT]; // 🌟 记录当前位置，用于插值计算
    bool is_otto_resting_;
    bool has_hands_;
};

#endif // _OTTO_MOVEMENTS_H_
