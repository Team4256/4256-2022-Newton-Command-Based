// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Conveyor;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Conveyor;

public class OuttakeBall extends CommandBase {
  private static Conveyor conveyor;
  private static OuttakeBall instance = null;
  public static synchronized OuttakeBall getInstance() {
    if (instance == null) {
      instance = new OuttakeBall();
    }
    return instance;
  }
  /** Creates a new ConveyorCmd. */
public OuttakeBall() {
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
    SmartDashboard.putBoolean("outtaking", true);
  }
// Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    conveyor.stop();
  }
  
}