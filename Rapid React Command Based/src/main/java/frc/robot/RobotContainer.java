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
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.commands.Auto.*;
import frc.robot.commands.Climber.DisengageSmallHooks;
import frc.robot.commands.Climber.EngageSmallHooks;
import frc.robot.commands.Climber.LowerClimberArms;
import frc.robot.commands.Climber.LowerClimberHooks;
import frc.robot.commands.Climber.RaiseClimberArms;
import frc.robot.commands.Climber.RaiseClimberHooks;
import frc.robot.commands.Conveyor.IntakeBall;
import frc.robot.commands.Conveyor.LowerIntake;
import frc.robot.commands.Conveyor.OuttakeBall;
import frc.robot.commands.Conveyor.RaiseIntake;
import frc.robot.commands.Conveyor.ReverseShooter;
import frc.robot.commands.Conveyor.ShootBalls;
import frc.robot.commands.Swerve.SwerveXboxCmd;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Gyro;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.Xbox;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  private final SwerveSubsystem swerveSubsystem = SwerveSubsystem.getInstance();
  private final Climber climber = Climber.getInstance();
  private final Conveyor conveyor = Conveyor.getInstance();

  public Xbox driver = new Xbox(0);
  public Xbox gunner = new Xbox(1);
  Gyro gyro = Gyro.getInstance();
  SendableChooser<Command> chooser = new SendableChooser<>();
  public final Command threeBallAutoBottom = new ThreeBallAutoBottom();
  public final Command twoBallAutoBottom = new TwoBallAutoBottom();
  public final Command twoBallAutoMiddle = new TwoBallAutoMiddle();
  public final Command twoBallAutoTop = new TwoBallAutoTop();
  public final Command testAuto = new TestAuto();
  public final Command intakeBall = IntakeBall.getInstance();
  public final Command outtakeBall = OuttakeBall.getInstance();
  public final Command shootBalls = ShootBalls.getInstance();
  public final Command reverseShooter = ReverseShooter.getInstance();
  public final Command raiseIntake = RaiseIntake.getInstance();
  public final Command lowerIntake = LowerIntake.getInstance();
  public final Command raiseClimberArms = RaiseClimberArms.getInstance();
  public final Command lowerClimberArms = LowerClimberArms.getInstance();
  public final Command engageSmallHooks = EngageSmallHooks.getInstance();
  public final Command disengageSmallHooks = DisengageSmallHooks.getInstance();
  public final Command raiseClimberHooks = RaiseClimberHooks.getInstance();
  public final Command lowerClimberHooks = LowerClimberHooks.getInstance();

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

    //conveyor.setDefaultCommand(raiseIntake);
    //climber.setDefaultCommand(disengageSmallHooks);

    configureButtonBindings();
    chooser.setDefaultOption("Three Ball Auto Bottom", threeBallAutoBottom);
    chooser.addOption("Two Ball Auto Bottom", twoBallAutoBottom);
    chooser.addOption("Two Ball Auto middle", twoBallAutoMiddle);
    chooser.addOption("Two Ball Auto top", twoBallAutoTop);
    chooser.addOption("Test Auto", testAuto);

    // Put the chooser on the dashboard
    Shuffleboard.getTab("Competition").add(chooser);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    //swerve
    driver.bButton.whenPressed(() -> swerveSubsystem.zeroHeading());
    driver.xButton.whenPressed(() -> swerveSubsystem.stopModules());
    driver.dPadUp.whenPressed(
      () -> swerveSubsystem.resetOdometer(new Pose2d(0, 0, new Rotation2d(0)))
    );
    driver.dPadRight.whenPressed(() -> gyro.setOffset(0));

    //intake
    driver.leftTriggerButton.whenHeld(intakeBall);
    driver.rightTriggerButton.whenHeld(outtakeBall);
    driver.yButton.whenHeld(raiseIntake);
    driver.aButton.whenHeld(lowerIntake);

    //shooter
    gunner.rightTriggerButton.whenHeld(shootBalls);
    gunner.leftTriggerButton.whenHeld(reverseShooter);

    //climber
    gunner.aButton.whenHeld(raiseClimberArms);
    gunner.yButton.whenHeld(lowerClimberArms);
    gunner.rightBumper.whenHeld(engageSmallHooks);
    gunner.leftBumper.whenHeld(disengageSmallHooks);
    gunner.dPadUp.whenHeld(raiseClimberHooks);
    gunner.dPadDown.whenHeld(lowerClimberHooks);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return chooser.getSelected();
  }
}
