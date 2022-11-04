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
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.*;
import frc.robot.commands.Auto.GeneralAutoCommands.AutoBeltUp;
import frc.robot.commands.Auto.GeneralAutoCommands.AutoLowerIntake;
import frc.robot.commands.Auto.GeneralAutoCommands.AutoShootBalls;
import frc.robot.commands.Auto.GeneralAutoCommands.AutoShootBallsHigh;
import frc.robot.commands.Auto.GeneralAutoCommands.AutoSwerveIntake;
import frc.robot.commands.Conveyor.*;
import frc.robot.subsystems.*;
import frc.robot.commands.Auto.GeneralAutoCommands.AutoShootBallsLow;

public class ThreeBallAutoBottom extends SequentialCommandGroup {

  SwerveSubsystem swerve = SwerveSubsystem.getInstance();
  Gyro gyro = Gyro.getInstance();
  Conveyor conveyor = Conveyor.getInstance();
  ShootBalls shootBalls = ShootBalls.getInstance();
  LowerIntake lowerIntake = LowerIntake.getInstance();
  IntakeBall intakeBall = IntakeBall.getInstance();
  AutoShootBallsHigh shootBallsHigh = AutoShootBallsHigh.getInstance();
  AutoLowerIntake autoIntake = AutoLowerIntake.getInstance();

  PIDController xController = new PIDController(1, 0, 0);
  PIDController yController = new PIDController(1, 0, 0);
  ProfiledPIDController thetaController = new ProfiledPIDController(
      5,
      0,
      .5,
      Parameters.THETA_CONTROLLER_CONSTRAINTS);

  PathPlannerTrajectory autoPath = PathPlanner.loadPath("5 ball 1", 1, 1);
  PPSwerveControllerCommand command = new PPSwerveControllerCommand(
      autoPath,
      swerve::getPose,
      Parameters.DRIVE_KINEMATICS,
      xController,
      yController,
      thetaController,
      swerve::setModuleStates,
      swerve);

  /** Creates a new ThreeBallAutoBottom. */
  public ThreeBallAutoBottom() {
    addCommands(
        new InstantCommand(() -> gyro.reset()),
        new InstantCommand(() -> gyro.setOffset(-111)),
        new InstantCommand(() -> thetaController.enableContinuousInput(0, 2*Math.PI)),
        new InstantCommand(() -> thetaController.reset(Math.toRadians(-111))),
        new InstantCommand(() -> swerve.resetOdometer(autoPath.getInitialPose())),
        //new AutoShootBallsLow(),
        new AutoShootBallsHigh(),
        new AutoLowerIntake(),
        new AutoSwerveIntake(command),
        new InstantCommand(() -> swerve.stopModules()),
        new ParallelDeadlineGroup(new WaitCommand(.5), new AutoShootBallsLow()),
        new AutoBeltUp(),
        new InstantCommand(() -> gyro.setOffset(0))    
    );
  }
}
