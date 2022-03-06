// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto.GeneralAutoCommands;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.Conveyor.ShootBalls;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoShootBalls extends ParallelDeadlineGroup {
  /** Creates a new AutoIntake. */
  private static AutoLowerIntake instance = null;
  ShootBalls shootBalls = ShootBalls.getInstance();
  
  
  AutoLowerIntake getInstance() {
    if (instance == null) {
      instance = new AutoLowerIntake();
    }
    return instance;
  }
  
  public AutoShootBalls() {
    super(new WaitCommand(5));
    addCommands(shootBalls);
  }
}
