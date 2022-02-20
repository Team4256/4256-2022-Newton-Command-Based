// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj2.command.CommandBase;
import java.util.List;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.SwerveXboxCmd;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.*;
import frc.robot.Parameters;
public class Auto1 extends CommandBase {
  SwerveSubsystem swerve;
  public Auto1(SwerveSubsystem swerve) {
    addRequirements(swerve);
  }
Trajectory trajectory;
SwerveControllerCommand swerveControllerCommand;




 public SwerveControllerCommand returnAutoCommand() {
  return swerveControllerCommand;
 }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    swerve.resetOdometer(trajectory.getInitialPose());
    // 1. Create trajectory settings
    TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
            Parameters.MAX_METERS_PER_SECOND,
            Parameters.MAX_ACCELERATION)
                    .setKinematics(Parameters.DRIVE_KINEMATICS);

    // 2. Generate trajectory
     this.trajectory = TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, 0, new Rotation2d(0)),
            List.of(
                    new Translation2d(1, 0),
                    new Translation2d(1, -1)),
            new Pose2d(2, -1, Rotation2d.fromDegrees(180)),
            trajectoryConfig);

    // 3. Define PID controllers for tracking trajectory
    PIDController xController = new PIDController(1.5, 0, 0);
    PIDController yController = new PIDController(1.5, 0, 0);
    ProfiledPIDController thetaController = new ProfiledPIDController(
            3, 0, 0, Parameters.THETA_CONTROLLER_CONSTRAINTS);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    this.swerveControllerCommand = new SwerveControllerCommand(
            trajectory,
            swerve::getPose,
            Parameters.DRIVE_KINEMATICS,
            xController,
            yController,
            thetaController,
            swerve::setModuleStates,
            swerve);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
