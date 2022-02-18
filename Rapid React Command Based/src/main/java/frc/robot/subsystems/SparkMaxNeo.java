package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;

import java.util.logging.Level;
import java.util.logging.Logger;

/**
 * SparkMax Motor Controller Used With a Neo Brushless Motor.
 * <p>
 * <i>Do not attempt to use followers with this class as it is not intended to
 * be used in such a way and may cause errors.</i>
 * 
 * @author Ian Woodard
 */
public class SparkMaxNeo extends CANSparkMax implements Motor{
    private static final int TIMEOUT_MS = 10;
    private static final double RAMP_RATE = 1.0;
    private static final int STALL_CURRENT_LIMIT = 90;
    private static final int FREE_CURRENT_LIMIT = 50;
    private static final int NEO_COUNTS_PER_REV = 42;
    private final RelativeEncoder encoder;
    private final int deviceID;
    public final IdleMode idleMode;
    private final boolean isInverted;
    private boolean updated = false;
    private double lastSetpoint = 0.0;
    private Logger logger;
    private PIDController anglePIDController = new PIDController(.02, 0.0, 0);
    //private CANEncoder angleEncoder = new CANEncoder(this);
    private AnalogInput encoderPort;
    private AnalogEncoder angleEncoder;



    /**
     * Offers a simple way of initializing and using NEO Brushless motors with a
     * SparkMax motor controller.
     * 
     * @param deviceID   CAN ID of the SparkMax
     * @param idleMode   IdleMode (Coast or Brake)
     * @param isInverted Indication of whether the SparkMax's motor is inverted
     */
    public SparkMaxNeo(final int deviceID, final IdleMode idleMode, final boolean isInverted) {
        this(deviceID, IdleMode.kCoast, isInverted, -1);

    }


    public SparkMaxNeo(final int deviceID, final IdleMode idleMode, final boolean isInverted, int analogEncoderID) {
        super(deviceID, MotorType.kBrushless);
        encoder = getEncoder();
        this.deviceID = deviceID;
        this.idleMode = idleMode;
        this.isInverted = isInverted;
        logger = Logger.getLogger("SparkMax " + Integer.toString(deviceID));
        if (analogEncoderID >= 0 && analogEncoderID <= 3) {
            encoderPort = new AnalogInput(analogEncoderID);
            angleEncoder = new AnalogEncoder(encoderPort);
            
    }

    }

    /**
     * Offers a simple way of initializing and using NEO Brushless motors with a
     * SparkMax motor controller.
     * <p>
     * This constructor is for NEO Brushless motors set by default to coast
     * <code>IdleMode</code>.
     * 
     * @param deviceID   CAN ID of the SparkMax
     * @param isInverted Indication of whether the SparkMax's motor is inverted
     */
    public SparkMaxNeo(final int deviceID, final boolean isInverted) {
        this(deviceID, IdleMode.kCoast, isInverted);
    }


    /**
     * Performs necessary initialization
     */
    public void init() {
        if (clearFaults() != REVLibError.kOk) {
            DriverStation.reportError("SparkMax " + deviceID + " could not clear faults.", false);
        }
        if (setIdleMode(idleMode) != REVLibError.kOk) {
            DriverStation.reportError("SparkMax " + deviceID + " could not set idle mode.", false);
        }
        if (setOpenLoopRampRate(RAMP_RATE) != REVLibError.kOk) {
            DriverStation.reportError("SparkMax " + deviceID + " could not set open loop ramp rate.", false);
        }
        if (setClosedLoopRampRate(RAMP_RATE) != REVLibError.kOk) {
            DriverStation.reportError("SparkMax " + deviceID + " could not set closed loop ramp rate.", false);
        }
        if (setCANTimeout(TIMEOUT_MS) != REVLibError.kOk) {
            DriverStation.reportError("SparkMax " + deviceID + " could not set can timeout.", false);
        }
        if (setSmartCurrentLimit(STALL_CURRENT_LIMIT, FREE_CURRENT_LIMIT) != REVLibError.kOk) {
            DriverStation.reportError("SparkMax " + deviceID + " could not set smart current limit.", false);
        }
        setInverted(isInverted);
        set(0.0);
        anglePIDController.enableContinuousInput(-180.0, 180.0);
    }

    /**
     * @return Counts of the motor
     */
    public int getCounts() {
        return (int) (encoder.getPosition() * NEO_COUNTS_PER_REV);
    }

    /**
     * @return Rotations of the motor
     */
    public double getPosition() {
        return encoder.getPosition();
    }

    public double getPIDError() {
        return 0.0;
    }

    public double getEncoderVoltage() {
        return 0.0;
    }
    /**
     * @return Revolutions per minute of the motor
     */
    public double getRPM() {
        return encoder.getVelocity();
    }

    /**
     * @return Revolutions per second of the motor
     */
    public double getRPS() {
        return (getRPM() / 60.0);
    }

    // get angle
    public double getCurrentAngle() {
       // return Math.toDegrees(this.angleEncoder.getDistance());
        return this.angleEncoder.get()*360;
    }

    // Set Speed
    @Override
    public void setSpeed(final double speed) {
        super.set(speed);
        lastSetpoint = speed;
        updated = true;
        logger.log(Level.FINE, Double.toString(speed));
    }

    // Set Angle
    public void setAngle(double targetAngle) {

        double encoderPosition = getCurrentAngle();
        while (encoderPosition > 180) {
            encoderPosition -= 360;
        }
        while (encoderPosition < -180) {
            encoderPosition += 360;
        }
        //SmartDashboard.putNumber("encoder position", encoderPosition);

        if (Math.abs(targetAngle - encoderPosition) < 2) {
            super.set(0.);
            return;
        }

        double percentSpeed = anglePIDController.calculate(encoderPosition, targetAngle);

        if (Math.abs(percentSpeed) > .5) {
            percentSpeed = Math.signum(percentSpeed) * .5;
        } else if (Math.abs(percentSpeed) < .01) {
            percentSpeed = Math.signum(percentSpeed) * .01;
        }

        super.set(percentSpeed);
        //SmartDashboard.putNumber("Percent Output", percentSpeed);
    }
    public void completeLoopUpdate() {
        if (!updated) {
            super.set(lastSetpoint);
        }
        updated = false;
    }

    public void setParentLogger(final Logger logger) {
        this.logger = logger;
    }

    public void getCurrentAngle(double angle) {

    }

    public void resetEncoder() {
        angleEncoder.reset();
    }

    public double getPositionFromIntegratedSensor(){
        return 0.0;
       
     }
}
