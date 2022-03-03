// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.*;
import frc.robot.commands.Conveyor.*;
import frc.robot.commands.Swerve.*;
import frc.robot.subsystems.*;

public class TwoBallAutoMiddle extends SequentialCommandGroup {

  SwerveSubsystem swerve = SwerveSubsystem.getInstance();
  Gyro gyro = Gyro.getInstance();
  Conveyor conveyor = Conveyor.getInstance();
  ReverseShooter shootBalls = ReverseShooter.getInstance();
  IntakeBall intakeBall = IntakeBall.getInstance();
    PIDController xController = new PIDController(1, 0, 0);
    PIDController yController = new PIDController(1, 0, 0);
    ProfiledPIDController thetaController = new ProfiledPIDController(
      5,
      0,
      0,
      Parameters.THETA_CONTROLLER_CONSTRAINTS
    );
    
  PathPlannerTrajectory autoPath = PathPlanner.loadPath("2 ball middle", 1, 1);
  PPSwerveControllerCommand command = new PPSwerveControllerCommand(
    autoPath,
    swerve::getPose,
    Parameters.DRIVE_KINEMATICS,
    xController,
    yController,
    thetaController,
    swerve::setModuleStates,
    swerve
  );

  /** Creates a new ThreeBallAutoBottom. */
  public TwoBallAutoMiddle() { 
    addCommands(
       new InstantCommand(() -> gyro.setOffset(180)),
      new InstantCommand(() -> thetaController.enableContinuousInput(0, 360)),
      new InstantCommand(() -> swerve.resetOdometer(autoPath.getInitialPose())),
      command,
      new InstantCommand(() -> swerve.stopModules())
      //new InstantCommand(() -> gyro.reset()),
      //new InstantCommand(() -> swerve.resetOdometer(autoPath.getInitialPose())),
      //new InstantCommand(() -> thetaController.enableContinuousInput(-180, 180)),
      //new ParallelDeadlineGroup(command, intakeBall),
      //new InstantCommand(() -> swerve.stopModules()),
      //new ParallelDeadlineGroup( new WaitCommand(5), shootBalls)
    );
  }
}
