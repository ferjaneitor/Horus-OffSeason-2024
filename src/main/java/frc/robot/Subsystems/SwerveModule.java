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

/*
 * @Author: Juan Felipe Zepeda del Toro
 * @Author: Fernando Joel Cruz Briones
 * Version: 1.0
 */

public class SwerveModule {

    //Creamos nuestros motores
    private CANSparkMax driveMotor, steerMotor;

    //Creamos nuestros encoders relativos
    private RelativeEncoder driveEncoder, steerEncoder;

    //Creamos nuestro encoder absoluto
    private CANcoder absolutNcoder;

    //Guardamos el valor si es que esta invertido el encoder
    private boolean absolutNcoderReversed;

    //Guardamos el offset del encoder
    private double absolutNcoderOffSetDeg;

    //Creamos el control PID
    private PIDController steeringPidController;

    //Creamos nuestro modulo
    public SwerveModule(int driveMotorId, int steerMotorId, boolean driveMotorReversed, boolean steerMotorReversed,
        int CANcoderID, double absolutNcoderOffSet, boolean CANcoderReversed) {
        
        //Conseguimos el offset y si el el encoder absoluto esta invertido
        this.absolutNcoderOffSetDeg = absolutNcoderOffSet;
        this.absolutNcoderReversed = CANcoderReversed;
        
        //Creamos nuestros objetos que seran los motores y el encoder absoluto
        absolutNcoder = new CANcoder(CANcoderID, "6348 Horus CANivore");
        driveMotor = new CANSparkMax(driveMotorId, MotorType.kBrushless);
        steerMotor = new CANSparkMax(steerMotorId, MotorType.kBrushless);

        //Invertimos los motores si es que deben que ser invertidos
        driveMotor.setInverted(driveMotorReversed);
        steerMotor.setInverted(steerMotorReversed);

        //Conseguirmos los encoder relativos
        driveEncoder = driveMotor.getEncoder();
        steerEncoder = steerMotor.getEncoder();

        //Convertimos la posicion y volcidad de los disitntos encoders relativos para despues poder conseguirlos
        driveEncoder.setPositionConversionFactor(ModuleConstants.kDriveEncoderRot2Meter);
        driveEncoder.setVelocityConversionFactor(ModuleConstants.kDriveEncoderRPM2MeterPerSec);
        steerEncoder.setPositionConversionFactor(ModuleConstants.kSteeringEncoderRot2Deg);
        steerEncoder.setVelocityConversionFactor(ModuleConstants.kSteeringEncoderRPM2RadPerSec);

        //Creamos nuestro control PID
        steeringPidController = new PIDController(
            ModuleConstants.kPSteering,
            ModuleConstants.kISteering,
            ModuleConstants.KDSteering
        );
        //Hacemos que pueda aceptar entradas conitnuas para que este se modifique dependiendo de lo que reciba
        steeringPidController.enableContinuousInput(-360, 360);

        //reiniciamos los encoders relativos
        resetEncoders();
    }

    //Funcion para conseguir la posicion del drive motor
    public double getDrivePosition() {
        return driveEncoder.getPosition();
    }

    //funcion para conseguir la posicion del steering motor
    public double getSteeringPosition() {
        return steerEncoder.getPosition();
    }

    //funcion para conseguir la velocidad del drive motor
    public double getDriveVelocity() {
        return driveEncoder.getVelocity();
    }

    //funcion para conseguir la velocidad del steering motor
    public double getSteeringVelocity() {
        return steerEncoder.getVelocity();
    }

    //funcion para conseguir la posicion absoluta de la llanta y aplicamos lo que es nuestro offset
    public double getAbsolutePosition() {
        double angle = (absolutNcoder.getAbsolutePosition().getValue()) * 360;
        angle -= (absolutNcoderOffSetDeg * 360);
        return angle * (absolutNcoderReversed ? -1 : 1);
    }

    // Funcion para Reiniciar los encoders
    public void resetEncoders() {
        SmartDashboard.putNumber("Encoder: " + absolutNcoder.getDeviceID(), getAbsolutePosition());
        driveEncoder.setPosition(0);
        steerEncoder.setPosition(getAbsolutePosition());
    }

    //funcion para conseguir el estado de nuestro modulo (El estado es la potencia que tendran nuestros motores)
    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getSteeringPosition()));
    }

    //funcion para conseguir la posicion del modulo
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getDrivePosition(), new Rotation2d(getSteeringPosition()));
    }

    //Funcion para detener la posicion
    public void stop() {
        driveMotor.set(0);
        steerMotor.set(0);
    }

    //Funcion para poner el modulo en su estado deseado
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