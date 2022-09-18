// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Conveyor2.Conveyor;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Shooter;

public class ShootHigh2 extends CommandBase {

  private static ShootHigh2 instance = null;
  Shooter shooter;

  public static synchronized ShootHigh2 getInstance() {
    if (instance == null) {
      instance = new ShootHigh2();
    }
    return instance;
  }

  public ShootHigh2() {
    this.shooter = Shooter.getInstance();
    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooter.spinOuterShooterHigh();
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
