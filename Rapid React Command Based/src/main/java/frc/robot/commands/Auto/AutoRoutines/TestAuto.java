// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto.AutoRoutines;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.*;
import frc.robot.commands.Conveyor.*;
import frc.robot.subsystems.*;

public class TestAuto extends SequentialCommandGroup {

  SwerveSubsystem swerve = SwerveSubsystem.getInstance();
  Gyro gyro = Gyro.getInstance();
  Conveyor conveyor = Conveyor.getInstance();
  ReverseShooter shootBalls = ReverseShooter.getInstance();
  IntakeBall intakeBall = IntakeBall.getInstance();
    PIDController xController = new PIDController(1, 0, 0);
    PIDController yController = new PIDController(1, 0, 0);
    ProfiledPIDController thetaController = new ProfiledPIDController(
      4,
      0,
      0,
      Parameters.THETA_CONTROLLER_CONSTRAINTS
    );
    
  PathPlannerTrajectory autoPath = PathPlanner.loadPath("Test Path", 1, 1);
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
  public TestAuto() { 
    addCommands(
    
      //new InstantCommand(() -> gyro.reset()),
      //new InstantCommand(() -> gyro.setOffset(160)),
      //new InstantCommand(() -> thetaController.enableContinuousInput(0,2*Math.PI)),
      //new InstantCommand(() -> thetaController.reset(Math.toRadians(160))),
      //new InstantCommand(() -> swerve.resetOdometer(autoPath.getInitialPose())),
      //command,
      //new InstantCommand(() -> swerve.stopModules())
    );
  }
}
