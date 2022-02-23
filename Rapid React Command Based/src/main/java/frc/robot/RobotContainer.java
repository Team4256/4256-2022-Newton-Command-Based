// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.commands.Swerve.SwerveXboxCmd;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.Xbox;
import java.util.List;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();

  public Xbox driver = new Xbox(0);
  public Xbox gunner = new Xbox(1);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    swerveSubsystem.setDefaultCommand(
      new SwerveXboxCmd(
        swerveSubsystem,
        () -> driver.getLeftStickY(),
        () -> -driver.getLeftStickX(),
        () -> -driver.getRightStickX(),
        () -> !driver.startButton.get()
      )
    );
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    driver.bButton.whenPressed(() -> swerveSubsystem.zeroHeading());
    driver.xButton.whenPressed(() -> swerveSubsystem.stopModules());
    driver.yButton.whenPressed(() -> swerveSubsystem.driveModules());
    driver.dPadUp.whenPressed(
      () -> swerveSubsystem.resetOdometer(new Pose2d(0, 0, new Rotation2d(0)))
    );
  }

  public Command getAutonomousCommand() {
    // 3. Define PID controllers for tracking trajectory
    PIDController xController = new PIDController(1, 0, 0);
    PIDController yController = new PIDController(1, 0, 0);
    ProfiledPIDController thetaController = new ProfiledPIDController(
      5,
      0,
      0,
      Parameters.THETA_CONTROLLER_CONSTRAINTS
    );
    thetaController.enableContinuousInput(-180, 180);

    PathPlannerTrajectory auto1Path = PathPlanner.loadPath("Auto3", 1, 1);
    PPSwerveControllerCommand command = new PPSwerveControllerCommand(
      auto1Path,
      swerveSubsystem::getPose,
      Parameters.DRIVE_KINEMATICS,
      xController,
      yController,
      thetaController,
      swerveSubsystem::setModuleStates,
      swerveSubsystem
    );
    return new SequentialCommandGroup(
      new InstantCommand(
        () -> swerveSubsystem.resetOdometer(auto1Path.getInitialPose())
      ),
      command,
      new InstantCommand(() -> swerveSubsystem.stopModules())
    );
  }
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  // public Command getAutonomousCommand() {
  //   // An ExampleCommand will run in autonomous
  //   return m_autoCommand;
  // }
}
