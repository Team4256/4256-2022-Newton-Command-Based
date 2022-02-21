package frc.robot.subsystems.Swerve;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.*;
import frc.robot.subsystems.Utility.Gyro;
import frc.robot.subsystems.Utility.Parameters;
public class SwerveSubsystem extends SubsystemBase {

  private final SwerveModule moduleA = new SwerveModule(
    Parameters.TRACTION_MOTOR_A_ID,
    Parameters.ROTATION_MOTOR_A_ID,
    Parameters.ROTATION_ENCODER_A_ID,
    Parameters.ABSOLUTE_ENCODER_A_TARE,
    "A"
  );
  private final SwerveModule moduleB = new SwerveModule(
    Parameters.TRACTION_MOTOR_B_ID,
    Parameters.ROTATION_MOTOR_B_ID,
    Parameters.ROTATION_ENCODER_B_ID,
    Parameters.ABSOLUTE_ENCODER_B_TARE,
    "B"
  );
  private final SwerveModule moduleC = new SwerveModule(
    Parameters.TRACTION_MOTOR_C_ID,
    Parameters.ROTATION_MOTOR_C_ID,
    Parameters.ROTATION_ENCODER_C_ID,
    Parameters.ABSOLUTE_ENCODER_C_TARE,
    "C"
  );
  private final SwerveModule moduleD = new SwerveModule(
    Parameters.TRACTION_MOTOR_D_ID,
    Parameters.ROTATION_MOTOR_D_ID,
    Parameters.ROTATION_ENCODER_D_ID,
    Parameters.ABSOLUTE_ENCODER_D_TARE,
    "D"
  );

  private Gyro gyro = Gyro.getInstance();

  private final SwerveDriveOdometry odometer = new SwerveDriveOdometry(
    Parameters.DRIVE_KINEMATICS,
    new Rotation2d(0)
  );

  public SwerveSubsystem() {
    zeroHeading();
  }

  public void zeroHeading() {
    gyro.reset();
  }

  public double getHeading() {
    return Math.IEEEremainder(gyro.getCurrentAngle(), 360);
  }

  public Rotation2d getRotation2d() {
    return Rotation2d.fromDegrees(getHeading());
  }

  public Pose2d getPose() {
    return odometer.getPoseMeters();
  }

  public void resetOdometer(Pose2d pose) {
    odometer.resetPosition(pose, getRotation2d());
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
  }


  public void stopModules() {
    moduleA.stop();
    moduleB.stop();
    moduleC.stop();
    moduleD.stop();
    SmartDashboard.putBoolean("I'm Stopped! :D", true);
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
