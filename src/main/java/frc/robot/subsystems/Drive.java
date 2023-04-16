package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class Drive extends SubsystemBase{

    // Controllers
    private CANSparkMax leftMaster= new CANSparkMax(Constants.DriveConstants.MASTER_LEFT_ID, MotorType.kBrushless);
    private CANSparkMax leftSlave = new CANSparkMax(Constants.DriveConstants.SLAVE_LEFT_ID, MotorType.kBrushless);
    
    private CANSparkMax rightMaster=new CANSparkMax(Constants.DriveConstants.MASTER_RIGHT_ID, MotorType.kBrushless);
    private CANSparkMax rightSlave=new CANSparkMax(Constants.DriveConstants.SLAVE_RIGHT_ID, MotorType.kBrushless);

    //private PIDController speedPIDController = new PIDController(2, 0, 0);
    //private PIDController rotationPIDController = new PIDController(0.06, 0, 0);

    public ProfiledPIDController speedPIDController = new ProfiledPIDController(3, 0, 0, new Constraints(2, 1.5));
    public ProfiledPIDController rotationPIDController = new ProfiledPIDController(0.07, 0, 0, new Constraints(1.5, 1.5));
    public ProfiledPIDController balancePIDController = new ProfiledPIDController(0.022, 0.001, 0.0005, new Constraints(0.5, 0.5));
    public PIDController turnPidController = new PIDController(0.012, 0.00001, 0.00002);

    // Sensors
    AHRS navx;
    private final RelativeEncoder leftEncoder = leftMaster.getEncoder();
    private final RelativeEncoder rightEncoder = rightMaster.getEncoder();

    private DifferentialDrive differentialDrive = new DifferentialDrive(rightMaster, leftMaster);
    private DifferentialDrivePoseEstimator differentialDrivePoseEstimator;

    // PID Controllers for velocity based drive
    private SparkMaxPIDController leftMasterPidController = leftMaster.getPIDController();
    private SparkMaxPIDController rightMasterPidController = rightMaster.getPIDController();
    private SparkMaxPIDController leftSlavePidController = leftSlave.getPIDController();
    private SparkMaxPIDController rightSlavePidController = rightSlave.getPIDController();

    private PWMSparkMax max = new PWMSparkMax(0);

    private Joystick joystick;
    public PathPlannerTrajectory pathPlannerTrajectory = PathPlanner.loadPath("simple_straight", new PathConstraints(1, 1));

    public enum DriveMode{
        MANUAL,
        VELOCITY,
        BALANCE
    }

    double inputFactor = 1;
  

    public Drive(Joystick joystick){
        // Initialization
        navx = new AHRS(Port.kMXP);

        this.joystick = joystick;

        leftMaster.restoreFactoryDefaults();
        leftSlave.restoreFactoryDefaults();
        rightMaster.restoreFactoryDefaults();
        rightSlave.restoreFactoryDefaults();

        leftMaster.setIdleMode(IdleMode.kBrake);
        rightMaster.setIdleMode(IdleMode.kBrake);

        leftEncoder.setPosition(0);
        rightEncoder.setPosition(0);

        // Follow Master's
        leftSlave.follow(leftMaster);
        rightSlave.follow(rightMaster);

        // Reverse right side!
        rightMaster.setInverted(true);
        rightSlave.setInverted(true);

        /*leftEncoder.setPositionConversionFactor(Constants.DriveConstants.POSITION_CONVERSION_FACTOR);
        rightEncoder.setPositionConversionFactor(Constants.DriveConstants.POSITION_CONVERSION_FACTOR);

        leftEncoder.setVelocityConversionFactor(Constants.DriveConstants.VELOCITY_CONVERSION_FACTOR);
        rightEncoder.setVelocityConversionFactor(Constants.DriveConstants.VELOCITY_CONVERSION_FACTOR);*/


        // Tele-Op Drive pid
        // Left Side
        leftMasterPidController.setP(Constants.DriveConstants.LEFT_KP);
        leftMasterPidController.setI(Constants.DriveConstants.LEFT_KI);
        leftMasterPidController.setD(Constants.DriveConstants.LEFT_KD);
        leftMasterPidController.setFF(Constants.DriveConstants.LEFT_KF);

        leftSlavePidController.setP(Constants.DriveConstants.LEFT_KP);
        leftSlavePidController.setI(Constants.DriveConstants.LEFT_KI);
        leftSlavePidController.setD(Constants.DriveConstants.LEFT_KD);
        leftSlavePidController.setFF(Constants.DriveConstants.LEFT_KF);

        // Right Side
        rightMasterPidController.setP(Constants.DriveConstants.RIGHT_KP);
        rightMasterPidController.setI(Constants.DriveConstants.RIGHT_KI);
        rightMasterPidController.setD(Constants.DriveConstants.RIGHT_KD);
        rightMasterPidController.setFF(Constants.DriveConstants.RIGHT_KF);

        rightSlavePidController.setP(Constants.DriveConstants.RIGHT_KP);
        rightSlavePidController.setI(Constants.DriveConstants.RIGHT_KI);
        rightSlavePidController.setD(Constants.DriveConstants.RIGHT_KD);
        rightSlavePidController.setFF(Constants.DriveConstants.RIGHT_KF);

        differentialDrivePoseEstimator = new DifferentialDrivePoseEstimator(Constants.DriveConstants.DIFFERENTIAL_DRIVE_KINEMATICS, getRotation2d(), getLeftEncoderMeter(), getRightEncoderMeter(), new Pose2d());
        navx.reset();
    }


    public void resetOdometry(){
        differentialDrivePoseEstimator.resetPosition(new Rotation2d(), 0, 0, pathPlannerTrajectory.getInitialPose());
    }

    @Override
    public void periodic(){
        
        if(joystick.getRawButtonPressed(1)){
            inputFactor = inputFactor * -1;
        }

        SmartDashboard.putNumber("navx angle", navx.getYaw());
        SmartDashboard.putNumber("NAVX PITCH:", navx.getPitch());  
        SmartDashboard.putNumber("NAVX ROLL:", navx.getRoll());   
        SmartDashboard.putNumber("Left meter:", getLeftEncoderMeter());
        SmartDashboard.putNumber("Right meter:", getRightEncoderMeter());
        differentialDrivePoseEstimator.update(getRotation2d(), getLeftEncoderMeter(), getRightEncoderMeter());
    }

    public void cleverDrive(){
        curvatureDrive();
    }

    private void curvatureDrive(){
        differentialDrive.curvatureDrive(joystick.getRawAxis(1)*0.8*inputFactor, joystick.getRawAxis(2)*0.5, true);
    }

    // Driver Methods
    private void arcadeDrive(){
       differentialDrive.arcadeDrive(joystick.getRawAxis(1)*0.8*inputFactor, joystick.getRawAxis(2)*0.5);
    }

    public void autoControl(double meterSetpoint, double degreeSetpoint){
        double speed = speedPIDController.calculate(getLeftEncoderMeter(), meterSetpoint);
        double rotation = rotationPIDController.calculate(getYaw(), degreeSetpoint);

        differentialDrive.arcadeDrive(speed, rotation);
    }

    public void turnDrive(double degrees){
        double demand = turnPidController.calculate(navx.getYaw(), degrees);
        differentialDrive.arcadeDrive(0, demand);
    }

    public void balanceDrive(){
        double speed = balancePIDController.calculate(-getPitch(), 0);
        differentialDrive.arcadeDrive(speed, 0);
    }

    public Pose2d robotPosition(){
        return differentialDrivePoseEstimator.getEstimatedPosition();
    }

    public void tankDrive(double leftSpeed, double rightSpeed){
        differentialDrive.tankDrive(leftSpeed, rightSpeed, false);
    }

    // Sensors Configurations
    public void resetEncoders(){
        leftMaster.getEncoder().setPosition(0);
        rightMaster.getEncoder().setPosition(0);
    }

    public void resetGyro(){
        navx.reset();
    }

    public void resetAll(){
        resetEncoders();
        resetGyro();
    }

    // Encoder Informations
    public double getEncodersAvgMeter(){
        return (getLeftEncoderMeter() + getRightEncoderMeter()) / 2;
    }

    public double getLeftEncoderMeter(){
        return leftEncoder.getPosition() * Constants.DriveConstants.kPOSITION_2_METER;
    }

    public double getRightEncoderMeter(){
        return rightEncoder.getPosition() * Constants.DriveConstants.kPOSITION_2_METER;
    }

    // IMU Informations
    public double getAngle(){
        return Math.abs(navx.getAngle() % 360);
    }

    public Rotation2d getRotation2d(){
        return navx.getRotation2d().unaryMinus();
    }

    // Get Pitch, Roll, Yaw
    public double getYaw(){
        return navx.getYaw();
    }

    public double getPitch(){
        return navx.getPitch();
    }

    public double getRoll(){
        return navx.getRoll();
    }

    public DifferentialDriveWheelSpeeds wheelSpeeds(){
        return new DifferentialDriveWheelSpeeds(leftEncoder.getVelocity(), rightEncoder.getVelocity());
    }   
    
    public void stopMotor(){
        leftMaster.stopMotor();
        rightMaster.stopMotor();
    }
}
