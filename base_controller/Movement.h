// This class has all the functions related to Movement.
#ifndef Movement_h
#define Movement_h

#include <math.h>
#include "MotorId.h"
#include "Kinematics.h"
#include "BNO.h"
#include "QTR.h"
#include "Motor.h"
#include "ColorSensor.h"
#include "Gripper.h"

enum Direction{
    FORWARD,
    BACKWARD,
    TORIGHT,
    TOLEFT,
    STOP
};

enum colorNum{
    RED,
    GREEN,
    BLUE,
    YELLOW
};

class Movement {
  public:
    ///////////////////////////////////Comunication///////////////////////////////////
    bool detect_tile;
  
    //////////////////////////////////Constructor//////////////////////////////////////
    Movement(BNO *bno, LineSensor *lineSensor, ColorSensor *colorSensor);
    //////////////////////////////////Motors//////////////////////////////////////
    Motor back_right_motor_;
    Motor back_left_motor_;
    Motor front_right_motor_;
    Motor front_left_motor_;
    bool angleOffsetReached;
    bool start_search = false;
    bool outOfColor;
    int kMotorCount = 4;
    colorNum pastColor;
    float past_check;

    // Initialize motor encoders.
    void initEncoders();

    //////////////////////////////////PWM//////////////////////////////////////
    // Set same pwm to all motors.
    void changePwm(const uint8_t pwm);    
  
    // Stop robot.
    void stop();
    
    // Robot linear velocity to rpm per motor. 
    void cmdVelocity(const double linear_x, const double linear_y, const double angular_z);

    // Set motors to velocity. 
    void updatePIDKinematics(double fl_speed, double fr_speed, double bl_speed, double br_speed);

    void orientedMovement(const double linear_x, const double linear_y, double angular_z);

    void setRobotAngle(const double angle);

    float getRobotAngle();

    void moveDirection(Direction direction, const double angleOffset);

    void moveDirection(Direction direction, const double angleOffset, double speed, bool flag);

    void moveDirection(Direction direction, const uint8_t squares, const double angleOffset);

    void driveToTarget(float coord_x);

    void driveToTarget(float coord_x, const double linear_x, const double linear_y, const double angular_z);

    void driveToColor(const double start_x_pos, Direction direction, colorNum color_id);

    void GoToSquare();

    bool detectedTilefromRaspi();

    bool detectedCubefromRaspi();

    int getCurrentPosX();

    void setGlobalPosX(int globalPosX);

    colorNum setColorTile(colorNum colorId);
    
    int getSquareCounter();
    
    void setSquareCounter(int squareCounter);

    Direction getDirectionState();

    void setInitialRobotAngle(double angle);

  private:
    // Pins
    static constexpr uint8_t kDigitalPinsFrontLeftMotor[2] = {44, 43};
    static constexpr uint8_t kAnalogPinFrontLeftMotor = 5;
    static constexpr uint8_t kEncoderPinsFrontLeftMotor = 2;

    static constexpr uint8_t kDigitalPinsBackLeftMotor[2] = {42, 41};
    static constexpr uint8_t kAnalogPinBackLeftMotor = 4;
    static constexpr uint8_t kEncoderPinsBackLeftMotor = 19;

    static constexpr uint8_t kDigitalPinsFrontRightMotor[2] = {40,39};
    static constexpr uint8_t kAnalogPinFrontRightMotor = 10;
    static constexpr uint8_t kEncoderPinsFrontRightMotor = 3;

    static constexpr uint8_t kDigitalPinsBackRightMotor[2] = {38,37};
    static constexpr uint8_t kAnalogPinBackRightMotor = 9;
    static constexpr uint8_t kEncoderPinsBackRightMotor = 18;

    // Velocity maximums
    static constexpr double kWheelBase = 0.120;
    static constexpr double kWheelTrack = 0.235;
    static constexpr double kWheelDiameter = 0.072;
    static constexpr double kRPM = 380.0;
    static constexpr double kRPS = kRPM / 60;
    static constexpr double kMaxVelocity = kRPS * (M_PI * kWheelDiameter);

    static constexpr double kLinearXMaxVelocity = kMaxVelocity;
    static constexpr double kLinearYMaxVelocity = kMaxVelocity; 
    static constexpr uint8_t kPwmBits = 8;
    static constexpr double kBnoKP = 2.2;
    static constexpr double kBnoKI = 5.5;
    static constexpr double kBnoKD = 0.0004;
    static constexpr double kBNO_time = 10;
    static constexpr double kMaxErrorSum = 100;
    static constexpr double kMaxLinearY = 0.40;
    static constexpr double kMaxLinearX = 0.40;
    static constexpr double kMaxAngularZ = 1.1;
    int globalPosX_ = 0;
    Direction globalDirection_ = STOP;
    uint8_t kCentered2Image = 10; //Error in pixels
    uint8_t xError = 0;
    // Kinematics.
    Kinematics kinematics_;
    BNO *bno;
    LineSensor *lineSensor;
    ColorSensor *colorSensor;
    ColorSensor::colorData rgbData;
    double originAngle;
    
    PID pidBno;

    // Velocity.
    double delta_x_ = 0;
    double delta_y_ = 0;
    double delta_angular_ = 0;
    double linear_x_ = 0.0;
    double linear_y_ = 0.0;
    double angular_z_ = 0.0;
    float current_angle = 0.0;

    // Angle
    float angle_error_ = 0;
    float movementKp = 0.45;
    double robotAngle_ = 0;
    bool firstLineDetected = false;
    int squaresCount = 0;
    SignalSide prevSideDetected;
    SignalSide sideDetected_[4];
    static constexpr double kAngleTolerance_ = 5;
};

#endif
