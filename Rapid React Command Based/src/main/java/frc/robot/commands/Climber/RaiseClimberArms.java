// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Climber;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Conveyor;

import frc.robot.subsystems.Climber;

public class RaiseClimberArms extends CommandBase {
 
  private static RaiseClimberArms instance = null;
  Climber climber;

  public static synchronized RaiseClimberArms getInstance() {
    if (instance == null) {
      instance = new RaiseClimberArms();
    }
    return instance;
  }

public RaiseClimberArms() {
   this.climber = Climber.getInstance();
   addRequirements(climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    climber.raiseBigArms();
    SmartDashboard.putNumber("rightClimberPosition", climber.getEncoderCounts());
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