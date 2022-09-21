// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.Auto.AutoRoutines.FiveBallAuto;
import frc.robot.commands.Auto.AutoRoutines.MoveBackBottomEdge;
import frc.robot.commands.Auto.AutoRoutines.MoveBackBottomHub;
import frc.robot.commands.Auto.AutoRoutines.MoveBackTopEdge;
import frc.robot.commands.Auto.AutoRoutines.MoveBackTopHub;
import frc.robot.commands.Auto.AutoRoutines.TestAuto;
import frc.robot.commands.Auto.AutoRoutines.ThreeBallAutoBottom;
import frc.robot.commands.Auto.AutoRoutines.TwoBallAutoBottom;
import frc.robot.commands.Auto.AutoRoutines.TwoBallAutoBottomEdge;
import frc.robot.commands.Auto.AutoRoutines.TwoBallAutoMiddle;
import frc.robot.commands.Auto.AutoRoutines.TwoBallAutoMiddleEdge;
import frc.robot.commands.Auto.AutoRoutines.TwoBallAutoTop;
import frc.robot.commands.Auto.AutoRoutines.TwoBallAutoTopEdge;
import frc.robot.commands.Auto.AutoRoutines.ThreeBallAutoFar;
import frc.robot.commands.Climber.ClimberXboxControl;
import frc.robot.commands.Climber.DisengageSmallHooks;
import frc.robot.commands.Climber.EngageSmallHooks;
import frc.robot.commands.Climber.LowerClimberArms;
import frc.robot.commands.Climber.LowerClimberHooks;
import frc.robot.commands.Climber.RaiseClimberArms;
import frc.robot.commands.Climber.RaiseClimberHooks;
import frc.robot.commands.Conveyor.BeltUp;
import frc.robot.commands.Conveyor.RunShooter;
import frc.robot.commands.Conveyor.ShootBalls;
import frc.robot.commands.Conveyor2.Conveyor.IntakeBall2;
import frc.robot.commands.Conveyor2.Conveyor.LowerIntake2;
import frc.robot.commands.Conveyor2.Conveyor.OuttakeBall2;
import frc.robot.commands.Conveyor2.Conveyor.RaiseIntake2;
import frc.robot.commands.Conveyor2.Conveyor.ReverseShooter2;
import frc.robot.commands.Conveyor2.Conveyor.ShootBalls2;
import frc.robot.commands.Swerve.SwerveXboxCmd;
import frc.robot.commands.UpperShooterCommands.ShootHigh2;
import frc.robot.commands.UpperShooterCommands.ShootHighWithDelay;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Gyro;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.Xbox;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */

public class RobotContainer {

  private final SwerveSubsystem swerveSubsystem = SwerveSubsystem.getInstance();
  private final Climber climber = Climber.getInstance();
  private final Conveyor conveyor = Conveyor.getInstance();
  private final Shooter shooter = Shooter.getInstance();
  public static boolean smallHooksEngaged;
  public static final Xbox driver = new Xbox(0);
  public static final Xbox gunner = new Xbox(1);
  Gyro gyro = Gyro.getInstance();
  SendableChooser<Command> chooser = new SendableChooser<>();
  public final Command fiveBallAuto = new FiveBallAuto();
  public final Command threeBallAutoBottom = new ThreeBallAutoBottom();
  public final Command twoBallAutoBottom = new TwoBallAutoBottom();
  public final Command twoBallAutoMiddle = new TwoBallAutoMiddle();
  public final Command twoBallAutoTop = new TwoBallAutoTop();
  public final Command twoBallAutoTopEdge = new TwoBallAutoTopEdge();
  public final Command twoBallAutoBottomEdge = new TwoBallAutoBottomEdge();
  public final Command twoBallAutoMiddleEdge = new TwoBallAutoMiddleEdge();
  public final Command threeBallAutoFar = new ThreeBallAutoFar();
  public final Command moveBackBottomEdge = new MoveBackBottomEdge();
  public final Command moveBackBottomHub = new MoveBackBottomHub();
  public final Command moveBackTopEdge = new MoveBackTopEdge();
  public final Command moveBackTopHub = new MoveBackTopHub();
  public final Command testAuto = new TestAuto();
  public final Command intakeBall = IntakeBall2.getInstance();
  public final Command outtakeBall = OuttakeBall2.getInstance();
  public final Command shootBalls = ShootBalls2.getInstance();
  public final Command reverseShooter = ReverseShooter2.getInstance();
  public final Command raiseIntake = RaiseIntake2.getInstance();
  public final Command lowerIntake = LowerIntake2.getInstance();
  public final Command raiseClimberArms = RaiseClimberArms.getInstance();
  public final Command lowerClimberArms = LowerClimberArms.getInstance();
  public final Command engageSmallHooks = EngageSmallHooks.getInstance();
  public final Command disengageSmallHooks = DisengageSmallHooks.getInstance();
  public final Command raiseClimberHooks = RaiseClimberHooks.getInstance();
  public final Command lowerClimberHooks = LowerClimberHooks.getInstance();
  public final Command runShooter = RunShooter.getInstance();
  public final Command shootHigh = ShootHigh2.getInstance();
  public final Command shootHighWithDelay = new ShootHighWithDelay();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    swerveSubsystem.setDefaultCommand(
        new SwerveXboxCmd(
            swerveSubsystem,
            () -> driver.getLeftStickY(),
            () -> -driver.getLeftStickX(),
            () -> -driver.getRightStickX(),
            () -> !driver.startButton.get())
            );

    climber.setDefaultCommand(
        new ClimberXboxControl(
            climber,
            () -> gunner.getLeftStickY(), 
            () -> gunner.getRightStickY())
            );

    configureButtonBindings();
    chooser.addOption("Five Ball Auto", fiveBallAuto);
    chooser.setDefaultOption("Three Ball Auto Bottom", threeBallAutoBottom);
    chooser.addOption("Three Ball Auto Far", threeBallAutoFar);
    chooser.addOption("Two Ball Auto Bottom", twoBallAutoBottom);
    chooser.addOption("Two Ball Auto Middle", twoBallAutoMiddle);
    chooser.addOption("Two Ball Auto Top", twoBallAutoTop);
    chooser.addOption("Two Ball Auto Top Edge", twoBallAutoTopEdge);
    chooser.addOption("Two Ball Auto Middle Edge", twoBallAutoBottomEdge);
    chooser.addOption("Two Ball Auto Bottom Edge ", twoBallAutoMiddle);
    chooser.addOption("Move Back Bottom Edge", moveBackBottomEdge);
    chooser.addOption("Move Back Bottom Hub", moveBackBottomHub);
    chooser.addOption("Move Back Top Edge", moveBackTopEdge);
    chooser.addOption("Move Back Top Hub", moveBackTopHub);
    chooser.addOption("Test Auto", testAuto);

    // Put the chooser on the dashboard
    Shuffleboard.getTab("Competition").add(chooser);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
   * it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {


    // swerve
    driver.bButton.whenPressed(() -> swerveSubsystem.zeroHeading());
    driver.xButton.whenPressed(() -> swerveSubsystem.stopModules());
    driver.dPadUp.whenPressed(
        () -> swerveSubsystem.resetOdometer(new Pose2d(0, 0, new Rotation2d(0))));
    driver.dPadRight.whenPressed(() -> gyro.setOffset(0));

    // intake
    driver.leftTriggerButton.whenHeld(intakeBall);
    driver.rightTriggerButton.whenHeld(outtakeBall);
    driver.yButton.whenHeld(raiseIntake);
    driver.aButton.whenHeld(lowerIntake);

    // shooter
    //gunner.rightTriggerButton.whenHeld(shootBalls);
    gunner.rightTriggerButton.whenHeld(shootHighWithDelay);
    //gunner.rightTriggerButton.whenHeld(shootHigh);
    gunner.leftTriggerButton.whenHeld(reverseShooter);
    gunner.dPadRight.whenPressed(() -> shooter.setShootHigh());
    gunner.dPadLeft.whenPressed(() -> shooter.setShootLow());

    // climber
    gunner.aButton.whenHeld(raiseClimberArms);
    gunner.yButton.whenHeld(lowerClimberArms);
    gunner.rightBumper.whenHeld(disengageSmallHooks);
    gunner.leftBumper.whenHeld(engageSmallHooks);
    gunner.dPadUp.whenHeld(raiseClimberHooks);
    gunner.dPadDown.whenHeld(lowerClimberHooks);
    gunner.startButton.whenHeld(new InstantCommand(() -> climber.resetClimberEncoders()));

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
