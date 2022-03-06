// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto.GeneralAutoCommands;

import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import frc.robot.commands.Conveyor.IntakeBall;
import frc.robot.commands.Conveyor.LowerIntake;
import frc.robot.commands.Conveyor.ReverseShooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoSwerveIntake extends ParallelDeadlineGroup {
  /** Creates a new AutoIntake. */
  ReverseShooter shootBalls = ReverseShooter.getInstance();
  LowerIntake lowerIntake = LowerIntake.getInstance();
  IntakeBall intakeBall = IntakeBall.getInstance();
  PPSwerveControllerCommand command;
  
  //public static synchronized AutoSwerveIntake getInstance() {
    //if (instance == null) {
      //instance = new AutoSwerveIntake();
    //}
    //return instance;
  //}

  public AutoSwerveIntake(PPSwerveControllerCommand swerveCommand) {
    super(swerveCommand);
    addCommands(lowerIntake);
  }
}
