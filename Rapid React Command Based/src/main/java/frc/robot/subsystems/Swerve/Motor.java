package frc.robot.subsystems.Swerve;

public interface Motor{

public double getPosition();

public void setSpeed(final double speed); 

public double getCurrentAngle();

public double getEncoderVoltage();

public void setAngle(double targetAngle);

public double getRPM();

public double getRPS();

public double getPIDError();

public void init();

public void completeLoopUpdate();

public void resetEncoder();

public double getPositionFromIntegratedSensor();









}