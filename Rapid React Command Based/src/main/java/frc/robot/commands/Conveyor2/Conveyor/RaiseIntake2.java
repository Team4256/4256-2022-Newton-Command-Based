// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Conveyor2.Conveyor;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Conveyor;

public class RaiseIntake2 extends CommandBase {
 
  private static RaiseIntake2 instance = null;
  Conveyor conveyor;

  public static synchronized RaiseIntake2 getInstance() {
    if (instance == null) {
      instance = new RaiseIntake2();
    }
    return instance;
  }

public RaiseIntake2() {
   this.conveyor = Conveyor.getInstance();
   addRequirements(conveyor);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    conveyor.raiseIntake();
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