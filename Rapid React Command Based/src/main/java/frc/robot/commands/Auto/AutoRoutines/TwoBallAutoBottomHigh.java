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
import frc.robot.commands.Auto.GeneralAutoCommands.AutoRaiseIntake;
import frc.robot.commands.Auto.GeneralAutoCommands.AutoShootBalls;
import frc.robot.commands.Auto.GeneralAutoCommands.AutoSwerveIntake;
import frc.robot.commands.Conveyor.*;
import frc.robot.subsystems.*;
import frc.robot.commands.Auto.GeneralAutoCommands.AutoShootBallsHigh;
import frc.robot.commands.Auto.GeneralAutoCommands.AutoShootBallsLow;
public class TwoBallAutoBottomHigh extends SequentialCommandGroup {

  SwerveSubsystem swerve = SwerveSubsystem.getInstance();
  Gyro gyro = Gyro.getInstance();
  Conveyor conveyor = Conveyor.getInstance();
  ReverseShooter shootBalls = ReverseShooter.getInstance();
  IntakeBall intakeBall = IntakeBall.getInstance();
  Shooter shooter = Shooter.getInstance();
  AutoShootBallsHigh shootBallsHigh = AutoShootBallsHigh.getInstance();
    PIDController xController = new PIDController(1, 0, 0);
    PIDController yController = new PIDController(1, 0, 0);
    ProfiledPIDController thetaController = new ProfiledPIDController(
      5,
      0,
      0,
      Parameters.THETA_CONTROLLER_CONSTRAINTS
    );
    
  PathPlannerTrajectory autoPath = PathPlanner.loadPath("2 ball bottom High", 1, 1);
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
   public TwoBallAutoBottomHigh() { 
    addCommands(
      //new InstantCommand(() -> gyro.setOffset(180)),
      //new InstantCommand(() -> thetaController.enableContinuousInput(0, 360)),
      //new InstantCommand(() -> swerve.resetOdometer(autoPath.getInitialPose())),
      //command,
        //new InstantCommand(() -> swerve.stopModules())
        //new InstantCommand(() -> gyro.reset()),
        //new InstantCommand(() -> gyro.setOffset(-111)),
        //new InstantCommand(() -> thetaController.enableContinuousInput(0,2*Math.PI)),
        //new InstantCommand(() -> thetaController.reset(Math.toRadians(-111))),
        //new InstantCommand(() -> swerve.resetOdometer(autoPath.getInitialPose())),
        //new AutoShootBalls(),
        //new AutoLowerIntake(),
        //new AutoSwerveIntake(command),
        //new InstantCommand(() -> swerve.stopModules()),
        //new AutoShootBalls(),
        //new InstantCommand(() -> gyro.setOffset(0))
        new InstantCommand(() -> shooter.setShootHigh()),
        new InstantCommand(() -> gyro.reset()),
        new InstantCommand(() -> gyro.setOffset(-111)),
        new InstantCommand(() -> thetaController.enableContinuousInput(0, 2*Math.PI)),
        new InstantCommand(() -> thetaController.reset(Math.toRadians(-111))),
        new InstantCommand(() -> swerve.resetOdometer(autoPath.getInitialPose())),
        //new AutoShootBallsLow(),
        new AutoLowerIntake(),
        new AutoSwerveIntake(command),
        new AutoRaiseIntake(),
        new InstantCommand(() -> swerve.stopModules()),
        

        //new AutoShootBallsLow(),
        new ParallelDeadlineGroup(new WaitCommand(3), new AutoShootBallsHigh()),
        //new AutoBeltUp(),
        new InstantCommand(() -> gyro.setOffset(0))    
    );
  }
}