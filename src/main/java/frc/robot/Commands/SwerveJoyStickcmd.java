package frc.robot.Commands;

import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
//import frc.robot.Subsystems.LimeLightSubsystem;
import frc.robot.Subsystems.SwerveSubsystem;
import frc.robot.constants.DriveConstants;
import frc.robot.constants.OIConstants;

public class SwerveJoyStickcmd extends Command {

    //Creamos los subsystemas que necesitamos
    private final SwerveSubsystem swerveSubsystem;
    //private final LimeLightSubsystem LimeLightSubsystem;

    //Creamos los suppliers necesarios
    private final Supplier<Double> xSpdFunction, ySpdFunction, zSpdFunction;
    private final Supplier<Boolean> fieldOrientedSupplier;

    //Creamos los limitadores de velocidad
    private final SlewRateLimiter xLimiter, yLimiter, zLimiter;

    //creamos variable para asignar si el robot esta orientado a campo
    private boolean isFieldOriented;

    public boolean isFieldOriented() {
        return isFieldOriented;
    }

    public SwerveJoyStickcmd (
        SwerveSubsystem SwerveSubsystems,
        //LimeLightSubsystem LimeLightSubsystems,
        Supplier<Double> X,
        Supplier<Double> Y,
        Supplier<Double> Z,
        Supplier<Boolean> FieldOriented
    ){
        //Creamos los subsystemas
        this.swerveSubsystem = SwerveSubsystems;
        //this.LimeLightSubsystem = LimeLightSubsystems;

        //Sacamos los valores de X Y Z de los controles
        this.xSpdFunction = X;
        this.ySpdFunction = Y;
        this.zSpdFunction = Z;

        //Sacamos el valor si queremos orientarnos de campo
        this.fieldOrientedSupplier = FieldOriented;

        //Asignamos los limitadores de velocidad
        this.xLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
        this.yLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
        this.zLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond);
        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {}    

    //Cuando se Ejecute:
    @Override
    public void execute (){

        //1. Sacamos la velocidad de los controles
        double xSpeed = xSpdFunction.get(), ySpeed = -ySpdFunction.get(), zSpeed = -zSpdFunction.get();

        //2. Aplicamos el DeadBand
        xSpeed= Math.abs(xSpeed) > OIConstants.kDeadband ? xSpeed: 0.0;
        ySpeed= Math.abs(ySpeed) > OIConstants.kDeadband ? ySpeed: 0.0;
        zSpeed= Math.abs(zSpeed) > OIConstants.kDeadband ? zSpeed: 0.0;

        //3. Hacemos el viaje mas suave
        xSpeed = xLimiter.calculate(xSpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
        ySpeed = yLimiter.calculate(ySpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
        zSpeed = zLimiter.calculate(zSpeed) * DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond;

        //4. Construimos las velocidades del Chasis
        ChassisSpeeds ChassisSpeeds;
        if (fieldOrientedSupplier.get()) {
            // Relative to robot
            ChassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, zSpeed);
            isFieldOriented = false;
        } else {
            ChassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, zSpeed);
            // Relative to field
            ChassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, zSpeed, swerveSubsystem.getRotation2d());
            isFieldOriented = true;
        }
        
        /*          Este de aqui es cuando empezemos con el uso de la LimeLight
        if ((AimAmpCmd.AutoAimRunning == false) && (AimSpeakerCmd.AutoAimRunning == false)) {
            SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(ChassisSpeeds);
            swerveSubsystem.setModulesState(moduleStates);

        }*/


        // 5. Output each module states to wheels

    }

    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.stopModules();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
    
}
