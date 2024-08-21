package frc.robot.Subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.constants.DriveConstants;
import frc.robot.constants.ModuleConstants;

public class SwerveModule {

    private CANSparkMax driveMotor, steerMotor;
    private RelativeEncoder driveEncoder, steerEncoder;
    private CANcoder absolutNcoder;
    private boolean absolutNcoderReversed;
    private double absolutNcoderOffSetDeg;
    private PIDController steeringPidController;

    public SwerveModule(int driveMotorId, int steerMotorId, boolean driveMotorReversed, boolean steerMotorReversed,
                        int CANcoderID, double absolutNcoderOffSet, boolean CANcoderReversed) {

        this.absolutNcoderOffSetDeg = absolutNcoderOffSet;
        this.absolutNcoderReversed = CANcoderReversed;

        absolutNcoder = new CANcoder(CANcoderID, "6348 Horus CANivore");
        driveMotor = new CANSparkMax(driveMotorId, MotorType.kBrushless);
        steerMotor = new CANSparkMax(steerMotorId, MotorType.kBrushless);

        driveMotor.setInverted(driveMotorReversed);
        steerMotor.setInverted(steerMotorReversed);

        driveEncoder = driveMotor.getEncoder();
        steerEncoder = steerMotor.getEncoder();

        driveEncoder.setPositionConversionFactor(ModuleConstants.kDriveEncoderRot2Meter);
        driveEncoder.setVelocityConversionFactor(ModuleConstants.kDriveEncoderRPM2MeterPerSec);
        steerEncoder.setPositionConversionFactor(ModuleConstants.kTurningEncoderRot2Rad);
        steerEncoder.setVelocityConversionFactor(ModuleConstants.kTurningEncoderRPM2RadPerSec);

        steeringPidController = new PIDController(
            ModuleConstants.kPTurning,
            ModuleConstants.kITurning,
            ModuleConstants.KDTurning
        );
        steeringPidController.enableContinuousInput(-Math.PI, Math.PI);

        resetEncoders();
    }

    public double getDrivePosition() {
        return driveEncoder.getPosition();
    }

    public double getSteeringPosition() {
        return steerEncoder.getPosition();
    }

    public double getDriveVelocity() {
        return driveEncoder.getVelocity();
    }

    public double getSteeringVelocity() {
        return steerEncoder.getVelocity();
    }

    public double getAbsolutePosition() {
        double angle = (absolutNcoder.getAbsolutePosition().getValue()) * 360;
        angle -= (absolutNcoderOffSetDeg * 360);
        return angle * (absolutNcoderReversed ? -1 : 1);
    }

    public void resetEncoders() {
        SmartDashboard.putNumber("Encoder: " + absolutNcoder.getDeviceID(), getAbsolutePosition());
        driveEncoder.setPosition(0);
        steerEncoder.setPosition(getAbsolutePosition());
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getSteeringPosition()));
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getDrivePosition(), new Rotation2d(getSteeringPosition()));
    }

    public void stop() {
        driveMotor.set(0);
        steerMotor.set(0);
    }

    public void setDesiredState(SwerveModuleState state) {
        if (Math.abs(state.speedMetersPerSecond) < 0.001) {
            stop();
            return;
        }

        state = SwerveModuleState.optimize(state, getState().angle);
        driveMotor.set(state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        steerMotor.set(steeringPidController.calculate(getSteeringPosition(), state.angle.getRadians()));
    }
}