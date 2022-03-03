package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Parameters;

public class SwerveSubsystem extends SubsystemBase {

  private Gyro gyro = Gyro.getInstance();
  private static SwerveSubsystem instance = null;
  private static NetworkTableInstance nt;
  private static NetworkTable table;
  private double modAMax = 0;
  private double modBMax = 0;
  private double modCMax = 0;
  private double modDMax = 0;
  private double modAMin = 10;
  private double modBMin = 10;
  private double modCMin = 10;
  private double modDMin = 10;

  public static synchronized SwerveSubsystem getInstance() {
    if (instance == null) {
      instance = new SwerveSubsystem();
    }
    return instance;
  }

  private final SwerveModule moduleA = new SwerveModule(
    Parameters.TRACTION_MOTOR_A_ID,
    Parameters.ROTATION_MOTOR_A_ID,
    Parameters.ROTATION_ENCODER_A_ID,
    "A"
  );
  private final SwerveModule moduleB = new SwerveModule(
    Parameters.TRACTION_MOTOR_B_ID,
    Parameters.ROTATION_MOTOR_B_ID,
    Parameters.ROTATION_ENCODER_B_ID,
    "B"
  );
  private final SwerveModule moduleC = new SwerveModule(
    Parameters.TRACTION_MOTOR_C_ID,
    Parameters.ROTATION_MOTOR_C_ID,
    Parameters.ROTATION_ENCODER_C_ID,
    "C"
  );
  private final SwerveModule moduleD = new SwerveModule(
    Parameters.TRACTION_MOTOR_D_ID,
    Parameters.ROTATION_MOTOR_D_ID,
    Parameters.ROTATION_ENCODER_D_ID,
    "D"
  );

  private final SwerveDriveOdometry odometer = new SwerveDriveOdometry(
    Parameters.DRIVE_KINEMATICS,
    new Rotation2d(0)
  );

  public SwerveSubsystem() {
    zeroHeading();
    nt = NetworkTableInstance.getDefault();
    table = nt.getTable("table");
  }

  public void zeroHeading() {
    gyro.reset();
  }

  public void setXFormation() {
    
  }

  public double getHeading() {
    return Math.IEEEremainder(gyro.getAngleWithOffset(), 360);
  } //  public double getHeading() {

  //    return Math.IEEEremainder(gyro.getCurrentAngle(), 360);
  //  }
  //
  public Rotation2d getRotation2d() {
    return Rotation2d.fromDegrees(getHeading());
  }

  public Pose2d getPose() {
    return odometer.getPoseMeters();
  }

  public void resetOdometer(Pose2d pose) {
    odometer.resetPosition(pose, getRotation2d());
  }

  public void resetGyro() {
    gyro.reset();
  }

  @Override
  public void periodic() {
    odometer.update(
      getRotation2d(),
      moduleA.getState(),
      moduleB.getState(),
      moduleC.getState(),
      moduleD.getState()
    );

    SmartDashboard.putString("moduleAOdometerFeed", moduleA.getState().toString());
    SmartDashboard.putString("Odometer", odometer.getPoseMeters().toString());

    SmartDashboard.putNumber("moduleAPosition", moduleA.getAngle());
    SmartDashboard.putNumber("moduleBPosition", moduleB.getAngle());
    SmartDashboard.putNumber("moduleCPosition", moduleC.getAngle());
    SmartDashboard.putNumber("moduleDPosition", moduleD.getAngle());

    double modA = moduleA.getTurningMotor().getEncoderVoltage();
    double modB = moduleB.getTurningMotor().getEncoderVoltage();
    double modC = moduleC.getTurningMotor().getEncoderVoltage();
    double modD = moduleD.getTurningMotor().getEncoderVoltage();

    modAMax = Math.max(modAMax, modA);
    modBMax = Math.max(modBMax, modB);
    modCMax = Math.max(modCMax, modC);
    modDMax = Math.max(modDMax, modD);
    modAMin = Math.min(modAMin, modA);
    modBMin = Math.min(modBMin, modB);
    modCMin = Math.min(modCMin, modC);
    modDMin = Math.min(modDMin, modD);

    table.getEntry("ModuleA Angle").setNumber(moduleA.getTurningMotor().getCurrentAngle());//angle
    table.getEntry("ModuleB Angle").setNumber(moduleB.getTurningMotor().getCurrentAngle());
    table.getEntry("ModuleC Angle").setNumber(moduleC.getTurningMotor().getCurrentAngle());
    table.getEntry("ModuleD Angle").setNumber(moduleD.getTurningMotor().getCurrentAngle());

    table.getEntry("ModuleA Tare").setNumber(modA);//voltage
    table.getEntry("ModuleB Tare").setNumber(modB);
    table.getEntry("ModuleC Tare").setNumber(modC);
    table.getEntry("ModuleD Tare").setNumber(modD);

    table.getEntry("ModuleA Max").setNumber(modAMax);//voltage
    table.getEntry("ModuleB Max").setNumber(modBMax);
    table.getEntry("ModuleC Max").setNumber(modCMax);
    table.getEntry("ModuleD Max").setNumber(modDMax);

    table.getEntry("ModuleA Min").setNumber(modAMin);//voltage
    table.getEntry("ModuleB Min").setNumber(modBMin);
    table.getEntry("ModuleC Min").setNumber(modCMin);
    table.getEntry("ModuleD Min").setNumber(modDMin);
  }

  public void stopModules() {
    moduleA.stop();
    moduleB.stop();
    moduleC.stop();
    moduleD.stop();
  }

  public void driveModules() {
    moduleA.driveToDirection(0);
    moduleB.driveToDirection(0);
    moduleC.driveToDirection(0);
    moduleD.driveToDirection(0);
  }

  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds( 
      desiredStates, 
      Parameters.MAX_METERS_PER_SECOND
      );

    moduleA.setDesiredState(desiredStates[0]);
    moduleB.setDesiredState(desiredStates[1]);
    moduleC.setDesiredState(desiredStates[2]);
    moduleD.setDesiredState(desiredStates[3]);
  }
}
