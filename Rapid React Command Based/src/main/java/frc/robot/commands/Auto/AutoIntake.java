// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.Conveyor.IntakeBall;
import frc.robot.commands.Conveyor.LowerIntake;
import frc.robot.commands.Conveyor.ReverseShooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoIntake extends ParallelDeadlineGroup {
  /** Creates a new AutoIntake. */
  private static AutoIntake instance = null;
  ReverseShooter shootBalls = ReverseShooter.getInstance();
  LowerIntake lowerIntake = LowerIntake.getInstance();
  IntakeBall intakeBall = IntakeBall.getInstance();

  public static synchronized AutoIntake getInstance() {
    if (instance == null) {
      instance = new AutoIntake();
    }
    return instance;
  }

  public AutoIntake() {
    super(new WaitCommand(1));
addCommands(
      new ParallelDeadlineGroup( new WaitCommand(1), lowerIntake)
    );
  }
}
