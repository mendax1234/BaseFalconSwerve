package frc.robot;

import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.math.Conversions;
import frc.lib.util.CTREModuleState;
import frc.lib.util.SwerveModuleConstants;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.sensors.CANCoder;

import org.ejml.data.DEigenpair;

public class SwerveModule {
    public int moduleNumber;
    private double angleOffset;
    private WPI_TalonSRX mAngleMotor;
    private WPI_TalonFX mDriveMotor;
    private CANCoder angleEncoder;
    private double lastAngle;
    private double lastTargetAngle = 0;
    private double mLastError = 0;

    SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(Constants.Swerve.driveKS, Constants.Swerve.driveKV, Constants.Swerve.driveKA);

    public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants){
        this.moduleNumber = moduleNumber;
        angleOffset = moduleConstants.angleOffset;
        
        /* Angle Encoder Config */
        //angleEncoder = new CANCoder(moduleConstants.cancoderID);
        //configAngleEncoder();

        /* Angle Motor Config */
        mAngleMotor = new WPI_TalonSRX(moduleConstants.angleMotorID);
        configAngleMotor();

        /* Drive Motor Config */
        mDriveMotor = new WPI_TalonFX(moduleConstants.driveMotorID);
        configDriveMotor();

        lastAngle = getState().angle.getDegrees();
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop){
        desiredState = CTREModuleState.optimize(desiredState, getState().angle); //Custom optimize command, since default WPILib optimize assumes continuous controller which CTRE is not

        if(isOpenLoop){
            double percentOutput = desiredState.speedMetersPerSecond / Constants.Swerve.maxSpeed;
            mDriveMotor.set(ControlMode.PercentOutput, percentOutput);
        }
        else {
            double velocity = Conversions.MPSToFalcon(desiredState.speedMetersPerSecond, Constants.Swerve.wheelCircumference, Constants.Swerve.driveGearRatio);
            mDriveMotor.set(ControlMode.Velocity, velocity, DemandType.ArbitraryFeedForward, feedforward.calculate(desiredState.speedMetersPerSecond));
        }

        double angle = (Math.abs(desiredState.speedMetersPerSecond) <= (Constants.Swerve.maxSpeed * 0.01)) ? lastAngle : desiredState.angle.getDegrees(); //Prevent rotating module if speed is less then 1%. Prevents Jittering.
        //mAngleMotor.set(ControlMode.Position, Conversions.degreesToTalon(angle, Constants.Swerve.angleGearRatio)); 
        setTargetAngle(desiredState.angle.getDegrees());
        lastAngle = angle;
    }

    private void resetToAbsolute(){
        double absolute = getCanCoder().getDegrees();
        //double absolutePosition = Conversions.degreesToTalon(getCanCoder().getDegrees() - angleOffset, Constants.Swerve.angleGearRatio);
        double absolutePosition = Conversions.degreesToTalon(absolute - angleOffset, Constants.Swerve.angleGearRatio);
        mAngleMotor.setSelectedSensorPosition(absolutePosition);
    }

    //private void configAngleEncoder(){        
    //    angleEncoder.configFactoryDefault();
    //    angleEncoder.configAllSettings(Robot.ctreConfigs.swerveCanCoderConfig);
    //}

    private void configAngleMotor(){
        mAngleMotor.configFactoryDefault();
        mAngleMotor.configAllSettings(Robot.ctreConfigs.swerveAngleSRXConfig);
        mAngleMotor.setInverted(Constants.Swerve.angleMotorInvert);
        mAngleMotor.setNeutralMode(Constants.Swerve.angleNeutralMode);
        mAngleMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
        //resetToAbsolute();
    }

    private void configDriveMotor(){        
        mDriveMotor.configFactoryDefault();
        mDriveMotor.configAllSettings(Robot.ctreConfigs.swerveDriveFXConfig);
        mDriveMotor.setInverted(Constants.Swerve.driveMotorInvert);
        mDriveMotor.setNeutralMode(Constants.Swerve.driveNeutralMode);
        mDriveMotor.setSelectedSensorPosition(0);
    }

    public Rotation2d getCanCoder(){
        //return Rotation2d.fromDegrees(angleEncoder.getAbsolutePosition());
        return Rotation2d.fromDegrees(Conversions.talonToDegrees(
            mAngleMotor.getSensorCollection().getPulseWidthPosition(), 
            Constants.Swerve.angleGearRatio));
    }

    public SwerveModuleState getState(){
        double velocity = Conversions.falconToMPS(mDriveMotor.getSelectedSensorVelocity(), Constants.Swerve.wheelCircumference, Constants.Swerve.driveGearRatio);
        Rotation2d angle = Rotation2d.fromDegrees(Conversions.talonToDegrees(mAngleMotor.getSelectedSensorPosition(), Constants.Swerve.angleGearRatio));
        return new SwerveModuleState(velocity, angle);
    }
    
    public void setTargetAngle(double targetAngle) {
        //    	if(angleMotorJam) {
        //    		mAngleMotor.set(ControlMode.Disabled, 0);
        //    		return;
        //    	}
          
            lastTargetAngle = targetAngle;
      
            targetAngle %= 360;
      
            SmartDashboard.putNumber("Module Target Angle " + moduleNumber, targetAngle % 360);
      
            targetAngle += angleOffset;
      
            double currentAngle = mAngleMotor.getSelectedSensorPosition(0) * (360.0 / 4096.0);
            double currentAngleMod = currentAngle % 360;
            if (currentAngleMod < 0) currentAngleMod += 360;
      
            double delta = currentAngleMod - targetAngle;
      
            if (delta > 180) {
                targetAngle += 360;
            } else if (delta < -180) {
                targetAngle -= 360;
            }
      
            delta = currentAngleMod - targetAngle;
            if (delta > 90 || delta < -90) {
                if (delta > 90)
                    targetAngle += 180;
                else if (delta < -90)
                    targetAngle -= 180;
                mDriveMotor.setInverted(false);
            } else {
                mDriveMotor.setInverted(true);
            }
      
            targetAngle += currentAngle - currentAngleMod;
      
            double currentError = mAngleMotor.getClosedLoopError(0);
        //        if (Math.abs(currentError - mLastError) < 7.5 &&
        //                Math.abs(currentAngle - targetAngle) > 5) {
        //            if (mStallTimeBegin == Long.MAX_VALUE) {
        //            	mStallTimeBegin = System.currentTimeMillis();
        //            }
        //            if (System.currentTimeMillis() - mStallTimeBegin > STALL_TIMEOUT) {
        //            	angleMotorJam = true;
        //            	mAngleMotor.set(ControlMode.Disabled, 0);
        //            	mDriveMotor.set(ControlMode.Disabled, 0);
        //            	SmartDashboard.putBoolean("Motor Jammed" + moduleNumber, angleMotorJam);
        //            	return;
        //            }
        //        } else {
        //            mStallTimeBegin = Long.MAX_VALUE;
        //        }
            mLastError = currentError;
            targetAngle *= 4096.0 / 360.0;
            mAngleMotor.set(ControlMode.Position, targetAngle);
        }
}