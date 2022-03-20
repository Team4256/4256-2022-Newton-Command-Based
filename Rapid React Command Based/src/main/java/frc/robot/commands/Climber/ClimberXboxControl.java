// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Climber;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Parameters;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.SwerveSubsystem;
public class ClimberXboxControl extends CommandBase {
  private final Climber climber;
  private final Supplier<Double> rightSpdFunction;
  private final Supplier<Double> leftSpdFunction;



  public ClimberXboxControl(Climber climber, Supplier<Double> leftSpdFunction, Supplier<Double> rightSpdFunction) {
    this.climber = climber;
    this.rightSpdFunction = rightSpdFunction;
    this.leftSpdFunction = leftSpdFunction;
    addRequirements(climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
     // Get Xbox inputs
     double leftSpeed = leftSpdFunction.get();
     double rightSpeed = rightSpdFunction.get();
     
     climber.rightHookControl(rightSpeed);
     climber.leftHookControl(leftSpeed);
     //SmartDashboard.putNumber("rightSpeed", rightSpeed);
     //SmartDashboard.putNumber("leftSpeed", leftSpeed);
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climber.stopClimberMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
