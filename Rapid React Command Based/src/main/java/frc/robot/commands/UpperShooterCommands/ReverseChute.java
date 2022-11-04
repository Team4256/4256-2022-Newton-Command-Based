// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.UpperShooterCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Shooter;

public class ReverseChute extends CommandBase {

  private static ReverseChute instance = null;
  Shooter shooter;
  Conveyor conveyor;

  public static synchronized ReverseChute getInstance() {
    if (instance == null) {
      instance = new ReverseChute();
    }
    return instance;
  }

  public ReverseChute() {
    this.shooter = Shooter.getInstance();
    this.conveyor = Conveyor.getInstance();
    addRequirements(shooter, conveyor);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooter.reverseChute();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
