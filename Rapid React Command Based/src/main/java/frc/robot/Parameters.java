package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

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
  public static final double TRACK_WIDTH = Units.inchesToMeters(22.5);
  // Distance between front and back wheels
  public static final double WHEEL_BASE = Units.inchesToMeters(23.25);
  public static final double WHEEL_DIAMETER = Units.inchesToMeters(4); // inches
  public static final SwerveDriveKinematics DRIVE_KINEMATICS = new SwerveDriveKinematics(
    new Translation2d(WHEEL_BASE / 2, TRACK_WIDTH / 2),
    new Translation2d(WHEEL_BASE / 2, -TRACK_WIDTH / 2),
    new Translation2d(-WHEEL_BASE / 2, TRACK_WIDTH / 2),
    new Translation2d(-WHEEL_BASE / 2, -TRACK_WIDTH / 2)
  );

  /**
   * Swerve #1 = 
   * Swerve #2 = 1.22
   * Swerve #3 = 4.35
   * Swerve #4 = 
   * Swerve #5 = 2.89
   * Swerve #6 = 
   * Swerve #7 = 
   * Swerve #8 = 
   * Swerve #9 = 1.00
   */
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
  public static final double ABSOLUTE_ENCODER_A_TARE = 1.22; // Front Left
  public static final double ABSOLUTE_ENCODER_B_TARE = 2.89; // Front Right
  public static final double ABSOLUTE_ENCODER_C_TARE = 4.29; // Aft Left
  public static final double ABSOLUTE_ENCODER_D_TARE = 1.01; // Aft Right
  public static final double ANGLE_A_TARE = 9.074; // Front Left (angle)
  public static final double ANGLE_B_TARE = 6.264; // Front Right
  public static final double ANGLE_C_TARE = 3.840; // Aft Left
  public static final double ANGLE_D_TARE = 6.723; // Aft Right
  
  public static final double MAX_METERS_PER_SECOND = 3.83; // Max Speed
  public static final double MAX_ANGULAR_SPEED_RADIANS_PER_SECOND = 2 * 2 * Math.PI;
  public static final double MAX_ANGULAR_ACCELERATION_RADIANS_PER_SECOND_SQUARED =  Math.PI / 4;
  public static final double TELEOP_SPEED_LIMIT_MPS = MAX_METERS_PER_SECOND;
  public static final double TELEOP_ANGULAR_SPEED_LIMIT_RADIANS_PER_SECOND = MAX_ANGULAR_SPEED_RADIANS_PER_SECOND / 2;
  public static final double MAX_ACCELERATION = 3;
  public static final double TELEOP_MAX_ANGULAR_ACCELERATION = 3;
  public static final double ENCODER_CONVERSION_TO_REVOLUTIONS_PER_SECONDS = 1 / 7.04; //gear ratio * encoder conversion *
  public static final double RPS_TO_METERS_PER_SECOND = ENCODER_CONVERSION_TO_REVOLUTIONS_PER_SECONDS * Math.PI * WHEEL_DIAMETER;
  // Automomous
  public static final TrapezoidProfile.Constraints THETA_CONTROLLER_CONSTRAINTS = new TrapezoidProfile.Constraints( //
    MAX_ANGULAR_SPEED_RADIANS_PER_SECOND,
    MAX_ANGULAR_ACCELERATION_RADIANS_PER_SECOND_SQUARED
  );

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
  public static final double CLIMBER_MOTOR_SPEED_INDIVIDUAL = 0.2; // This is for when the joysticks are controlling
  // individual motors
  public static final int CLIMBER_LEFT_RETRACT_LIMIT_SWITCH_ID = 0;
  public static final int CLIMBER_RIGHT_RETRACT_LIMIT_SWITCH_ID = 1;

  // Conveyor
  public static final int INTAKE_MOTOR_ID = 16;
  public static final int SHOOTER_MOTOR_ID = 26;
  public static final int CONVEYOR_MOTOR_ID = 15;
  public static final double INTAKE_MOTOR_SPEED = 0.5;
  public static final double CONVEYOR_MOTOR_SPEED = 0.5;
  public static final double SHOOTER_MOTOR_SPEED = 0.8;
  public static final int INTAKE_BALL_SENSOR_ID = 0;
  public static final int CONVEYOR_BALL_SENSOR_ID = 1;
  public static final int INTAKE_FORWARD_CHANNEL = 0; // get real value
  public static final int INTAKE_REVERSE_CHANNEL = 1; //get real value

  // COMPETITION ROBOT
  public static final double angleEncoderMinVoltage[] = {
  //   .008544921,
  //   .020751951,
  //   0.01220703,
  //   0.026855466,
  0.021,
  0.017,
  0.010,
  0.011
   };
  public static final double angleEncoderMaxVoltage[] = {
    // 4.921874496,
    // 4.921874496,
    // 4.913329475000001,
    // 4.887694812,
    4.951,
    4.952,
    4.952,
    4.949
  };
  public static final double angleEncoderTareVoltage[] = {
    // 3.4033,
    // 2.877,
    // 2.388,
    // 1.355,
    1.22,
    2.89,
    4.29,
    1.01
  };

  // Aligner tolerances

  public static final double POSITION_TOLERANCE = 5.00;
  public static final double VELOCITY_TOLERANCE = 5.00;
}
