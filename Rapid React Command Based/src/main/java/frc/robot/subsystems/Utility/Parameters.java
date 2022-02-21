package frc.robot.subsystems.Utility;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.controller.PIDController;

public class Parameters {
    // Controller
    public static final double CONTROLLER_DEADBAND = 0.25;
    
    // Gyro
    public static final byte GYRO_UPDATE_HZ = 50;
    public static final double GYRO_OFFSET = 0;

    // Ball Chute
    public static final int BALL_SENSOR_ID = 0;

    // Sensors
    public static final int PROXIMITY_SENSOR_ID = 4;

    // Swerve
    // Distance between right and left wheels
    public static final double kTrackWidth = Units.inchesToMeters(22.85);
    // Distance between front and back wheels
    public static final double kWheelBase = Units.inchesToMeters(25.5);

    public static final SwerveDriveKinematics DRIVE_KINEMATICS = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));
    public static final int ROTATION_MOTOR_A_ID = 11; // Front Left
    public static final int ROTATION_MOTOR_B_ID = 12; // Front Right
    public static final int ROTATION_MOTOR_C_ID = 13; // AFT Left
    public static final int ROTATION_MOTOR_D_ID = 14; // AFT Right
    public static final int TRACTION_MOTOR_A_ID = 21; // Front Left
    public static final int TRACTION_MOTOR_B_ID = 22; // Front Right
    public static final int TRACTION_MOTOR_C_ID = 23; // AFT Left
    public static final int TRACTION_MOTOR_D_ID = 24; // AFT Right
    public static final int ROTATION_ENCODER_A_ID = 0; // Front Left
    public static final int ROTATION_ENCODER_B_ID = 1; // Front Right
    public static final int ROTATION_ENCODER_C_ID = 2; // Aft Left
    public static final int ROTATION_ENCODER_D_ID = 3; // Aft Right
    public static final double ABSOLUTE_ENCODER_A_TARE = 3.4; // Front Left
    public static final double ABSOLUTE_ENCODER_B_TARE = 2.9; // Front Left
    public static final double ABSOLUTE_ENCODER_C_TARE = 2.4; // Front Left
    public static final double ABSOLUTE_ENCODER_D_TARE = 3.4033; // Front Left
    public static final double MAX_METERS_PER_SECOND = 3.83; // Max Speed
    public static final double MAX_ANGULAR_SPEED_RADIANS_PER_SECOND = 2 * 2 * Math.PI;
    public static final double MAX_ANGULAR_ACCELERATION_RADIANS_PER_SECOND_SQUARED = Math.PI / 4;
    public static final double TELEOP_SPEED_LIMIT_MPS = MAX_METERS_PER_SECOND / 1.5;
    public static final double TELEOP_ANGULAR_SPEED_LIMIT_RADIANS_PER_SECOND = MAX_ANGULAR_SPEED_RADIANS_PER_SECOND / 2;
    public static final double MAX_ACCELERATION = 3;
    public static final double TELEOP_MAX_ANGULAR_ACCELERATION = 3;


    // Automomous
    public static final TrapezoidProfile.Constraints THETA_CONTROLLER_CONSTRAINTS = //
    new TrapezoidProfile.Constraints(
    MAX_ANGULAR_SPEED_RADIANS_PER_SECOND,
    MAX_ANGULAR_ACCELERATION_RADIANS_PER_SECOND_SQUARED);

    // Climber
    public static final int CLIMBER_LOCK_FORWARD_CHANNEL = 4; // Lock gear engage for climbing on Falcons
    public static final int CLIMBER_LOCK_REVERSE_CHANNEL = 5; // Lock gear disengage for climbing on Falcons
    public static final int CLIMBER_FORWARD_CHANNEL = 6; // Climber rotate arms up channel Solonoid Port Extend
    public static final int CLIMBER_REVERSE_CHANNEL = 7; // Climber rotate arms down channel solenoid port Retract
    public static final double CLIMBING_SPEED_LOW = 0.2;
    public static final int R_CLIMBER_MOTOR_ID = 29; // Climber Motor right
    public static double MED_HEIGHT_COUNT = 10;
    public static double MAX_HEIGHT_COUNT = 10;
    public static final double CLIMBER_MOTOR_SPEED_DPAD = 0.2; // need the actual speed. This is for the Dpad when both
                                                               // poles retract at the same time
    public static final double CLIMBER_MOTOR_SPEED_INDIVIDUAL = 0.2;// This is for when the joysticks are controlling
                                                                    // individual motors
    public static final int CLIMBER_LEFT_RETRACT_LIMIT_SWITCH_ID = 0;
    public static final int CLIMBER_RIGHT_RETRACT_LIMIT_SWITCH_ID = 1;

    // INTAKE
    public static final int INTAKE_MOTOR_ID = 16;
    public static final int SHOOTER_MOTOR_ID = 16;
    public static final int CONVEYOR_MOTOR_ID = 17;
    public static final double INTAKE_MOTOR_SPEED = 0.5;
    public static final double CONVEYOR_MOTOR_SPEED = 0.5; 
    public static final double SHOOTER_MOTOR_SPEED = 0.8;
    public static final int INTAKE_BALL_SENSOR_ID = 0;
    public static final int CONVEYOR_BALL_SENSOR_ID = 1;
    public static final int INTAKE_FORWARD_CHANNEL = 0; // get real value
    public static final int INTAKE_REVERSE_CHANNEL = 1; //get real value

    // shooter
    public static final int SHOOTERMOTOR_L_ID = 26;
    public static final int SHOOTERMOTOR_R_ID = 27;
    public static final int STIRRERMOTOR_ID = 18;
    public static final int FEEDERMOTOR_ID = 17;
    public static final int SHROUD_DOWN_CHANNEL = 3;
    public static final int SHROUD_UP_CHANNEL = 2;
    public static double SHOOTER_MOTOR_SPEED_LINE = 0.9;
    public static double SHOOTER_MOTOR_SPEED_TRENCH_CLOSE = 0.70;
    public static double SHOOTER_MOTOR_SPEED_TRENCH_FAR = 0.80;

    // Hopper
    public static double FEEDER_STIRRER_MOTOR_SPEED = -0.6; // was 0.5 on march 6
    public static double STIRRER_MOTOR_SPEED = 0.3; // was .4 on march 6
    public static double STIRRER_MOTOR_DELAY = 0.75;
    public static double DISTANCE_LOW_MIN = 4.5;
    public static double DISTANCE_LOW_MAX = 5.5;
    public static double DISTANCE_MED_MIN = 9.5;
    public static double DISTANCE_MED_MAX = 10.5;
    public static double DISTANCE_HIGH_MIN = 14.5;
    public static double DISTANCE_HIGH_MAX = 15.5;

    // Minimum and Maximum Voltage

    // public static final double angleEncoderMinVoltage[] = {.0354, .0219, .0073,
    // .017};
    // public static final double angleEncoderMaxVoltage[] = {4.9218, 4.9243,
    // 4.9145, 4.923};
    // public static final double angleEncoderTareVoltage[] =
    // {3.5864,2.8491,2.3742,1.3354};//Swerve Number(A,B,C,D): PRACTICE

    // PRACTICE ROBOT
    // public static final double angleEncoderMinVoltage[] = { 0.00366, 0.01099,
    // 0.009766, 0.01343};
    // public static final double angleEncoderMaxVoltage[] = {4.9133, 4.9255,
    // 4.9219, 4.9243};
    // public static final double angleEncoderTareVoltage[] = {4.155,1.752,1.297,
    // 4.234};//Swerve Number(A,B,C,D): PRACTICE
    // PRACTICE BOT
    // * A (Swerve Number XX) MIN. VOL. = 0.00366, MAX. VOL. = 4.9133, TARE VOL. =
    // 4.155
    // * B (Swerve Number XX) MIN. VOL. = 0.01099, MAX. VOL. = 4.9255, TARE VOL. =
    // 1.752
    // * C (Swerve Number XX) MIN. VOL. = 0.009766, MAX. VOL. = 4.9219, TARE VOL. =
    // 1.297
    // * D (Swerve Number XX) MIN. VOL. = 0.01343, MAX. VOL. = 4.9243, TARE VOL. =
    // 4.234

    // COMPETITION ROBOT (to be deleted)
    // public static final double angleEncoderMinVoltage[] = {0.0183, 0.0292,
    // 0.0073, 0.0170};
    // public static final double angleEncoderMaxVoltage[] = {4.9243, 4.9218,
    // 4.9145, 4.9194};
    // public static final double angleEncoderTareVoltage[] =
    // {3.527,2.863,2.382,1.335};

    // COMPETITION ROBOT
    public static final double angleEncoderMinVoltage[] = { .008544921, .020751951, 0.01220703, 0.026855466 };
    public static final double angleEncoderMaxVoltage[] = { 4.921874496, 4.921874496, 4.913329475000001, 4.887694812 };
    public static final double angleEncoderTareVoltage[] = { 3.4033, 2.877, 2.388, 1.355 };

    // Aligner tolerances

    public static final double POSITION_TOLERANCE = 5.00;
    public static final double VELOCITY_TOLERANCE = 5.00;

}