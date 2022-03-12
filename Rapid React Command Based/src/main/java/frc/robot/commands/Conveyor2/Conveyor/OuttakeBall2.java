// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Conveyor2.Conveyor;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Conveyor;

public class OuttakeBall2 extends CommandBase {
  private static Conveyor conveyor;
  private static OuttakeBall2 instance = null;
  public static synchronized OuttakeBall2 getInstance() {
    if (instance == null) {
      instance = new OuttakeBall2();
    }
    return instance;
  }
  /** Creates a new ConveyorCmd. */
public OuttakeBall2() {
   this.conveyor = Conveyor.getInstance();
   addRequirements(conveyor);
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    conveyor.conveyorBeltDown();
    conveyor.outtakeBall();
  }
// Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    conveyor.stop();
  }
  
}