// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Climber;
import java.util.function.Supplier;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Parameters;
import frc.robot.subsystems.Climber;

public class ClimberMotorSpeed extends CommandBase {
  private final Supplier<Double> ySpdLeftFunction;
  private final Supplier<Double> ySpdRightFunction;
  private final SlewRateLimiter yLimiter;
  public double ySpeedLeft;
  public double ySpeedRight;

  Climber climber;
  private static ClimberMotorSpeed instance = null;
  public ClimberMotorSpeed(Climber climber, Supplier<Double> ySpdLeftFunction, Supplier<Double> ySpdRightFunction) {
    this.climber = climber;
    this.ySpdLeftFunction = ySpdLeftFunction;
    this.ySpdRightFunction = ySpdRightFunction;
    this.yLimiter = new SlewRateLimiter(Parameters.MAX_ACCELERATION);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
     // Get Xbox inputs
     double ySpeedLeft = ySpdLeftFunction.get();
     double ySpeedRight = ySpdRightFunction.get();
     SmartDashboard.putNumber("ySpdLeft", ySpeedLeft);
     SmartDashboard.putNumber("ySpdRight", ySpeedRight);
     // 2. Apply deadband
    ySpeedLeft = MathUtil.applyDeadband(ySpeedLeft, Parameters.CONTROLLER_DEADBAND);
    ySpeedRight = MathUtil.applyDeadband(ySpeedRight, Parameters.CONTROLLER_DEADBAND);
  
     // 3. Make the driving smoother
     ySpeedLeft = yLimiter.calculate(ySpeedLeft) * Parameters.CLIMBER_MOTOR_SPEED;
     ySpeedRight = yLimiter.calculate(ySpeedRight) * Parameters.CLIMBER_MOTOR_SPEED;
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
