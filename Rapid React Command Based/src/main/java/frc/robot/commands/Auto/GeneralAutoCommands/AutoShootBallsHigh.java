// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto.GeneralAutoCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Shooter;

public class AutoShootBallsHigh extends CommandBase {
  
  private static AutoShootBallsHigh instance = null;
  Conveyor conveyor;
  Shooter shooter;

  public static synchronized AutoShootBallsHigh getInstance() {
    if (instance == null) {
      instance = new AutoShootBallsHigh();
    }
    return instance;
  }
  
public AutoShootBallsHigh() {
   this.conveyor = Conveyor.getInstance();
   this.shooter = Shooter.getInstance();
   addRequirements(conveyor, shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() { 
    // conveyor.spinConveyorShooter();
    // shooter.spinOuterShooterLow();
    shooter.shootCurrentBalls();
    shooter.spinConveyorShooter();
    //shooter.spinOuterShooterHigh();
    conveyor.conveyorBeltUp();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    conveyor.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
