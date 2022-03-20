package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Parameters;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import java.util.logging.Level;
import java.util.logging.Logger;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.AnalogInput;

/**
 * SparkMax Motor Controller Used With a Neo Brushless Motor.
 * <p>
 * <i>Do not attempt to use followers with this class as it is not intended to
 * be used in such a way and may cause errors.</i>
 * 
 */
public class TalonFXFalcon extends WPI_TalonFX implements Motor {

    private final int deviceID;
    public final NeutralMode idleMode;
    private final boolean isInverted;
    private boolean updated = false;
    private double lastSetpoint = 0.0;
    private Logger logger;
    private PIDController anglePIDController = new PIDController(.005, 0.0, 0.00);
    private PIDController falconVelocityPID = new PIDController(0.000005, 0.0, 0.0);
    public final Compass compass = new Compass();
    private double lastLegalDirection = 1.0;
    private AnalogInput encoderPort;
    public AnalogEncoder angleEncoder;
    private double encoderMinVoltage;
    private double encoderMaxVoltage;
    private double encoderTareVoltage;
    private double tractionSpeedAdjustment = 0.0;
    double maxAngle = 0;
    double minAngle = 360;
    public static final int kTimeoutMS = 10;
   
    
    

    /**
     * Offers a simple way of initializing and using NEO Brushless motors with a
     * SparkMax motor controller.
     * 
     * @param deviceID    CAN ID of the SparkMax
     * @param neutralMode IdleMode (Coast or Brake)
     * @param isInverted  Indication of whether the SparkMax's motor is inverted
     */
    public TalonFXFalcon(final int deviceID, final boolean isInverted) {

        super(deviceID);

        idleMode = NeutralMode.Brake;

        this.deviceID = deviceID;

        this.isInverted = isInverted;

        logger = Logger.getLogger("SparkMax " + Integer.toString(deviceID));

    }



    public TalonFXFalcon(final int deviceID, final NeutralMode neutralMode, final boolean isInverted, int analogEncoderID) {

        super(deviceID);

        idleMode = NeutralMode.Brake;

        this.deviceID = deviceID;

        this.isInverted = isInverted;

        encoderMaxVoltage = Parameters.angleEncoderMaxVoltage[analogEncoderID];

        encoderMinVoltage = Parameters.angleEncoderMinVoltage[analogEncoderID];

        encoderTareVoltage = Parameters.angleEncoderTareVoltage[analogEncoderID];

        encoderPort = new AnalogInput(analogEncoderID);

        angleEncoder = new AnalogEncoder(encoderPort);

        angleEncoder.setDistancePerRotation(360);

        SupplyCurrentLimitConfiguration limit = new SupplyCurrentLimitConfiguration(true, 30, 45, .25);
        
        configSupplyCurrentLimit(limit);
        // enableVoltageCompensation(true);

        // configVoltageCompSaturation(40);

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
    

    /**
     * Performs necessary initialization
     */
    public void init() {

        setInverted(isInverted);
        set(0.0);
        //anglePIDController.enableContinuousInput(-180.0, 180.0);
    }

    /**
     * @return Counts of the motor
     */

    @Override
    public void resetEncoder() {
       getSensorCollection().setIntegratedSensorPosition(0.0, kTimeoutMS);

    }

    /*
    returns the rotations of the motor from the integrated sensor
    */
    public double getPositionFromIntegratedSensor(){
       return getSensorCollection().getIntegratedSensorPosition();
    }


    /**
     * @return Rotations of the motor in encoder 
     */
    public double getPosition() {
        return angleEncoder.get();
    }

    /**
     * @Conversion the numbers multiplied are the encoder conversions times 10 to return 
     * @return Revolutions per minute of the motor
     */
    public double getRPM() {
        return getRPS() * 60;
    }

    
    /**
     * 
     * @Conversion the numbers multiplied are the encoder conversions times 10 to return 
     * @return Revolutions per second of the motor
     */
    public double getRPS() {
        return getSensorCollection().getIntegratedSensorVelocity() / 2048 * 10; 
    }

    // get angle
    public double getCurrentAngle() {

        return Math.toRadians(360 - (encoderPort.getVoltage() - encoderTareVoltage)/(encoderMaxVoltage - encoderMinVoltage) * 360);

    }

    public double getEncoderVoltage() {

        return encoderPort.getVoltage();

    }



    // Set Speed
    @Override
    public void setSpeed(final double inputPercentageSpeed) {

       
        // double motorSpeed = getSensorCollection().getIntegratedSensorVelocity();
        // double expectedSpeed = inputPercentageSpeed*Parameters.FALCON_PERCENT_TO_ENCODER_SPEED;
        // double adjustmentToSpeedAdjustment = falconVelocityPID.calculate(motorSpeed, expectedSpeed);

        // adjustmentToSpeedAdjustment = Math.abs(adjustmentToSpeedAdjustment) <=0.01 ? adjustmentToSpeedAdjustment: Math.signum(adjustmentToSpeedAdjustment )*0.01;
        // tractionSppeedAdjustment += adjustmentToSpeedAdjustment;

        // tractionSppeedAdjustment = Math.abs(tractionSppeedAdjustment) <=0.01 ? tractionSppeedAdjustment: Math.signum(tractionSppeedAdjustment )*0.01;

        // if(getDeviceID() == 21){
        //     SmartDashboard.putNumber("Speed adjustment", tractionSppeedAdjustment);
        //     SmartDashboard.putNumber("Adjustment to Speed adjustment", adjustmentToSpeedAdjustment);
        //     SmartDashboard.putNumber("input percentage speed", inputPercentageSpeed);
        //     SmartDashboard.putNumber("motor speed", motorSpeed);
        //     SmartDashboard.putNumber("expected speed", expectedSpeed);
        // }

        double adjustedPercentageSpeed = inputPercentageSpeed;
        //adjustedPercentageSpeed = inputPercentageSpeed;
        
        super.set(adjustedPercentageSpeed);
        lastSetpoint = adjustedPercentageSpeed;
        updated = true;
        logger.log(Level.FINE, Double.toString(adjustedPercentageSpeed));
        
    }



    // Set Angle
    public void setAngle(double targetAngle) {
        double targetPoint = Math.toDegrees(targetAngle);
        int channelID = encoderPort.getChannel();
        double encoderPosition = getCurrentAngle();
        while (targetPoint <= -180) {
            targetPoint += 360;
        } 

        while (targetPoint > 180) {
            targetPoint -= 360;
        }
        double error = targetPoint - encoderPosition;

        while (targetPoint - encoderPosition > 180) {
            encoderPosition += 360;
        }

        while (targetPoint - encoderPosition < -180) {
            encoderPosition -= 360;
        }

        double percentSpeed = anglePIDController.calculate(encoderPosition, targetPoint);

        if (Math.abs(percentSpeed) > .5) {
            percentSpeed = Math.signum(percentSpeed) * .5;
        }

        super.set(percentSpeed);
        updated = true;
        lastSetpoint = percentSpeed; 
    }



    public double getPIDError() {
        return anglePIDController.getPositionError();
    }
    public double getCurrentAngleDegrees() {
            return 360 - (encoderPort.getVoltage() - encoderTareVoltage)/(encoderMaxVoltage - encoderMinVoltage) * 360;
        }
    
    public double getTareAngle(double module) {
            return 360 - (encoderTareVoltage)/(encoderMaxVoltage - encoderMinVoltage) * 360;
    }

    public double pathTo(double target) {// ANGLE
        final double current = getCurrentAngle();
        double path = compass.legalPath(current, target);
        if (current == compass.legalize(current))
            lastLegalDirection = Math.signum(path);
        else if (Math.signum(path) != -lastLegalDirection)
            path -= Math.copySign(360, path);

        return path;
    }

    public void completeLoopUpdate() {
        if (!updated) {
            super.set(0);
        }
        updated = false;
    }

}
