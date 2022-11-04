// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto.GeneralAutoCommands;

import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.Conveyor.IntakeBall;
import frc.robot.commands.Conveyor.LowerIntake;
import frc.robot.commands.Conveyor.RaiseIntake;
import frc.robot.commands.Conveyor.ReverseShooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoRaiseIntake extends ParallelDeadlineGroup {
  /** Creates a new AutoIntake. */
  private static AutoLowerIntake instance = null;
  //LowerIntake lowerIntake = LowerIntake.getInstance();
  RaiseIntake raiseIntake = new RaiseIntake();

  public static synchronized AutoLowerIntake getInstance() {
    if (instance == null) {
      instance = new AutoLowerIntake();
    }
    return instance;
  }

  public AutoRaiseIntake() {
    super(new WaitCommand(.5));
    addCommands(raiseIntake);
  }
}
