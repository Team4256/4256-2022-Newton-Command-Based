package frc.robot.subsystems;

import java.util.logging.Logger;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
public final class SwerveModule {
	public static final double ROTATOR_GEAR_RATIO = 1.0;
	public static final double TRACTION_GEAR_RATIO = 52.0 / 9.0;// updated 2019
	public static final double TRACTION_WHEEL_CIRCUMFERENCE = 4.0 * Math.PI;// inches
	private final TractionControl driveMotor;
	private final RotationControl turningMotor;
	private double decapitated = 1.0;
	private double tractionDeltaPathLength = 0.0;
	private double tractionPreviousPathLength = 0.0;

	private final PIDController turningPidController;
  private final double tareAngle;

	// This constructor is intended for use with the module which has an encoder on
	// the traction motor.

	public SwerveModule(int driveMotorId, int turningMotorId, int absoluteEncoderId, double tareAngle) {

        this.tareAngle = tareAngle;

        driveMotor = new TractionControl(driveMotorId);
        turningMotor = new RotationControl(turningMotorId, absoluteEncoderId);


        turningPidController = new PIDController(.03, 0, 0);
        turningPidController.enableContinuousInput(-Math.PI, Math.PI);

        driveMotor.resetEncoder();
		turningMotor.resetEncoder();
    }


	/**
	 * This function prepares each motor individually, including setting PID values
	 * for the rotator.
	 **/
	public void init() {
	}

	/**
	 * This sets the tare angle. Positive means clockwise and negative means
	 * counter-clockwise.
	 **/
	public void setTareAngle(final double tareAngle) {
		setTareAngle(tareAngle, false);
	}

	/**
	 * This sets the tare angle. Positive means clockwise and negative means
	 * counter-clockwise. If relativeReference is true, tareAngle will be
	 * incremented rather than set.
	 **/
	public void setTareAngle(double tareAngle, final boolean relativeReference) {
		if (relativeReference)
			tareAngle += turningMotor.GetTareAngle();
		turningMotor.SetTareAngle(tareAngle);
	}

	/**
	 * Use wheel_chassisAngle to specify the wheel's orientation relative to the
	 * robot in degrees.
	 **/
	// public void swivelTo(final double wheel_chassisAngle) {
	// rotation.quickSet(decapitateAngle(wheel_chassisAngle), true);

	// }
	public void swivelTo(double targetAngle) {
		turningMotor.SetAngle(decapitateAngle(targetAngle));
	}

	/**
	 * Use wheel_fieldAngle to specify the wheel's orientation relative to the field
	 * in degrees.
	 **/
	public void swivelWith(final double wheel_fieldAngle, final double chassis_fieldAngle) {
		swivelTo(convertToRobot(wheel_fieldAngle, chassis_fieldAngle));
	}

	/**
	 * This function sets the master and slave traction motors to the specified
	 * speed, from -1 to 1. It also makes sure that they turn in the correct
	 * direction, regardless of decapitated state.
	 **/
	public void set(final double speed) {
		driveMotor.set(speed * decapitated);
	}

	public void checkTractionEncoder() {
		final double currentPathLength = tractionPathLength();
		tractionDeltaPathLength = currentPathLength - tractionPreviousPathLength;
		tractionPreviousPathLength = currentPathLength;
	}

	/**
	 * A shortcut to call completeLoopUpdate on all the Talons in the module.
	 **/
	public void completeLoopUpdate() {
		turningMotor.completeLoopUpdate();
		driveMotor.completeLoopUpdate();
	}

	/**
	 * Threshold should be specified in degrees. If the rotator is within that many
	 * degrees of its target, this function returns true.
	 **/
	public boolean isThere(final double threshold) {
		return Math.abs(turningMotor.getRotationMotor().getPIDError()) <= threshold;
	}

	/**
	 * This function makes sure the module rotates no more than 90 degrees from its
	 * current position. It should be used every time a new angle is being set to
	 * ensure quick rotation.
	 **/
	public double decapitateAngle(double endAngle) {
		double encoderPosition = turningMotor.getCurrentAngle();
		while (endAngle <= -180) {
			endAngle += 360;
		}
		while (endAngle > 180) {
			endAngle -= 360;
		}

		while (endAngle - encoderPosition > 180) {
			encoderPosition += 360;
		}

		while (endAngle - encoderPosition < -180) {
			encoderPosition -= 360;
		}

		if (Math.abs(endAngle - encoderPosition) > 90) {
			decapitated = -1;
		} else {
			decapitated = 1;
		}

		return decapitated == -1 ? endAngle + 180 : endAngle;

	}

	public double tractionSpeed() {
		return TRACTION_WHEEL_CIRCUMFERENCE * driveMotor.getRPS();// returns in/sec
	}

	public double tractionPathLength() {
		// return driveMotor.getPosition()*TRACTION_WHEEL_CIRCUMFERENCE/12.0;
		return 0;
	}

	public double deltaDistance() {
		return tractionDeltaPathLength;
	}

	public double deltaXDistance() {
		return tractionDeltaPathLength
				* Math.sin(convertToField(turningMotor.getCurrentAngle(), Robot.gyroHeading) * Math.PI / 180.0);
	}

	public double deltaYDistance() {
		return tractionDeltaPathLength
				* Math.cos(convertToField(turningMotor.getCurrentAngle(), Robot.gyroHeading) * Math.PI / 180.0);
	}

	public RotationControl getRotationMotor() {
		return turningMotor;
	}

	public TractionControl getTractionMotor() {
		return driveMotor;
	}

	public double getDecapitated() {
		return decapitated;
	}

	public void setParentLogger(final Logger logger) {
		// rotationControl.setParentLogger(logger);
		// driveMotor.setParentLogger(logger);
	}

	/**
	 * This function translates angles from the robot's perspective to the field's
	 * orientation. It requires an angle and input from the gyro.
	 **/
	public static double convertToField(final double wheel_robotAngle, final double chassis_fieldAngle) {
		return Compass.validate(wheel_robotAngle + chassis_fieldAngle);
	}

	/**
	 * This function translates angles from the field's orientation to the robot's
	 * perspective. It requires an angle and input from the gyro.
	 **/
	public static double convertToRobot(final double wheel_fieldAngle, final double chassis_fieldAngle) {
		return Compass.validate(wheel_fieldAngle - chassis_fieldAngle);
	}
	
	
	public void resetEncoderValue(){
		driveMotor.resetEncoder();
	}


    public double getIntegratedSensorENcoderCounts(){
		return driveMotor.getPositionFromIntegratedSensor();
	}

	public double getRPM() {
		return driveMotor.getRPM();
	}
	public double getAngle() {
		return turningMotor.getCurrentAngle();
	}
	public SwerveModuleState getState() {
        return new SwerveModuleState(getRPM(), new Rotation2d(turningMotor.getPositionFromIntegratedSensor()));
    }

    public void setDesiredState(SwerveModuleState state) {
        if (Math.abs(state.speedMetersPerSecond) < 0.001) {
            stop();
            return;
        }
        state = SwerveModuleState.optimize(state, getState().angle);
        driveMotor.set(state.speedMetersPerSecond / 3.83);
        turningMotor.SetAngle(turningPidController.calculate(turningMotor.getPositionFromIntegratedSensor(), state.angle.getRadians()));
		//SmartDashboard.putString("Swerve[" + absoluteEncoder.getChannel() + "] state", state.toString());
    }

    public void stop() {
        driveMotor.set(0);
        turningMotor.SetAngle(0);
    }
    
}