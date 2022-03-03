// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Climber;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.Conveyor.RaiseIntake;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Conveyor;

public class RaiseClimberHooks extends CommandBase {
  private static RaiseClimberHooks instance = null;
  Climber climber;

  public static synchronized RaiseClimberHooks getInstance() {
    if (instance == null) {
      instance = new RaiseClimberHooks();
    }
    return instance;
  }

public RaiseClimberHooks() {
   this.climber = Climber.getInstance();
   addRequirements(climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    climber.raiseHooks();
  }
// Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climber.stopClimberMotors();
  }
  
}
