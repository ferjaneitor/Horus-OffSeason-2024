package frc.robot.Subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.DriveConstants;

/*
 * @Author: Juan Felipe Zepeda del Toro
 * @Author: Fernando Joel Cruz Briones
 * Version 1.0
 */

public class SwerveSubsystem extends SubsystemBase {
 
    //Creamos nuestro Gyroscopio
    private AHRS gyro = new AHRS(SPI.Port.kMXP);

    //Creamos nuestros modulos Swerve
    private final SwerveModule frontLeft = new SwerveModule(
        DriveConstants.kFrontLeftDriveMotorPort,
        DriveConstants.kFrontLeftSteeringMotorPort,
        DriveConstants.kFrontLeftDriveEncoderReversed,
        DriveConstants.kFrontLeftSteeringEncoderReversed,
        DriveConstants.kFrontLeftDriveAbsoluteEncoderPort,
        DriveConstants.kFrontLeftDriveAbsoluteEncoderOffsetRad,
        DriveConstants.kFrontLeftDriveAbsoluteEncoderReversed);

    private final SwerveModule frontRight = new SwerveModule(
        DriveConstants.kFrontRightDriveMotorPort,
        DriveConstants.kFrontRightSteeringMotorPort, 
        DriveConstants.kFrontRightDriveEncoderReversed,
        DriveConstants.kFrontRightSteeringEncoderReversed,
        DriveConstants.kFrontRightDriveAbsoluteEncoderPort,
        DriveConstants.kFrontRightDriveAbsoluteEncoderOffsetRad,
        DriveConstants.kFrontRightDriveAbsoluteEncoderReversed);

    private final SwerveModule backLeft = new SwerveModule(
        DriveConstants.kBackLeftDriveMotorPort,
        DriveConstants.kBackLeftSteeringMotorPort, 
        DriveConstants.kBackLeftDriveEncoderReversed, 
        DriveConstants.kBackLeftSteeringEncoderReversed,
        DriveConstants.kBackLeftDriveAbsoluteEncoderPort,
        DriveConstants.kBackLeftDriveAbsoluteEncoderOffsetRad,
        DriveConstants.kBackLeftDriveAbsoluteEncoderReversed);

    private final SwerveModule backRight = new SwerveModule(
        DriveConstants.kBackRightDriveMotorPort, 
        DriveConstants.kBackRightSteeringMotorPort, 
        DriveConstants.kBackRightDriveEncoderReversed, 
        DriveConstants.kBackRightSteeringEncoderReversed,
        DriveConstants.kBackRightDriveAbsoluteEncoderPort,
        DriveConstants.kBackRightDriveAbsoluteEncoderOffsetRad, 
        DriveConstants.kBackRightDriveAbsoluteEncoderReversed);

    //Sacamos La posicion de cada uno de los modulos 
    SwerveModulePosition[] modulePositions = {
        frontLeft.getPosition(),
        frontRight.getPosition(),
        backLeft.getPosition(),
        backRight.getPosition()
    };

    //Creamos nuestr Odometria 
    // @return Rotation2d
    private final SwerveDriveOdometry odometer = new SwerveDriveOdometry(
        DriveConstants.kDriveKinematics,
        new Rotation2d(0), modulePositions
    );

    //Cuando se inicialize el subsystema vamos a dormir el robotp or un segundo y le aplicaremos la orientacion por campo
    public SwerveSubsystem() {
        new Thread(() -> {
            try {
                Thread.sleep(1000);
                zeroHeading();
            } catch (Exception e) {
            }
        }).start();
    }

    // Reiniciamos el frente y hacia donde apunta nuestro robot, asignando una nueva orientacion por campo
    public void zeroHeading() {
        gyro.reset();
    }

    // Funcion para cambiar a donde apunta el robot a 0 grados
    public void changeHeading(int angle) {
        gyro.reset();
        gyro.setAngleAdjustment(angle);
    }

    //Funcion para reuniciar los Encoders de todos los modulos
    public void resetEncoders() {
        frontLeft.resetEncoders();
        frontRight.resetEncoders();
        backLeft.resetEncoders();
        backRight.resetEncoders();
    }

    //Funcion para conseguir a donde apunta nuestro robot actualmente
    public double getHeading() {
        return Math.IEEEremainder(gyro.getAngle(), 360);
    }

    //Funcion para conseguir la rotacion 2d de nuestro robot
    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getHeading());
    }

    //Funcion para sacar la posicion del robot apartir de la odometria
    public Pose2d getPose() {
        return odometer.getPoseMeters();
    }

    //Funcion para Reiniciar el Odometro
    public void resetOdometry(Pose2d pose) {
        odometer.resetPosition(getRotation2d(), modulePositions, pose);
    }

    //Funcion que se actualizara constantemente que se encargara de actualizar el Odometro y
    //mostrar en el smartDashBoard a donde apunta nuestro robot y su ubicacion
    @Override
    public void periodic() {        
        odometer.update(getRotation2d(), modulePositions);
        SmartDashboard.putNumber("Robot Heading", getHeading());
        SmartDashboard.putString("Robot Location", getPose().getTranslation().toString());
    }

    //Funcion para detener todos los modulos
    public void stopModules() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }
    
    //Funcion para asignar los estados de todos los modulos
    public void setModulesState(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        frontLeft.setDesiredState(desiredStates[0]);
        frontRight.setDesiredState(desiredStates[1]);
        backLeft.setDesiredState(desiredStates[2]);
        backRight.setDesiredState(desiredStates[3]);
    }

    //Funcion para aliniarnos a un objetivo con la limelight
    public void alignToTarget(double yaw, double distance) {
        // Aquí, implementarás tu lógica para ajustar la orientación y la posición del robot.
        // Esto es solo un ejemplo simplificado.

        double ySpeed = distance * -0.05; // Coeficiente para ajustar la velocidad basada en la distancia
        double xSpeed = yaw * -0.3; // Coeficiente para ajustar la velocidad de rotación basada en el yaw

        // Limita la velocidad y la velocidad de rotación para evitar valores extremos
        ySpeed = Math.min(ySpeed, 1.0);
        xSpeed = Math.min(xSpeed, 1.0);

        // Convierte a estados de módulos Swerve y aplica al robot
        SwerveModuleState[] AutoAimState = DriveConstants.kDriveKinematics.toSwerveModuleStates(
            new ChassisSpeeds(ySpeed, xSpeed, 0.0));
        setModulesState(AutoAimState);
    }

}
