// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Climber;

public class DisengageSmallHooks extends CommandBase {
 
  private static DisengageSmallHooks instance = null;
  Conveyor conveyor;
  Climber climber;

  public static synchronized DisengageSmallHooks getInstance() {
    if (instance == null) {
      instance = new DisengageSmallHooks();
    }
    return instance;
  }

public DisengageSmallHooks() {
   this.climber = Climber.getInstance();
   addRequirements(climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    climber.disengageSmallHooks();
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
