package frc.robot.Subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.LimeConstants;

/*
 * @Author: Juan Felipe Zepeda del Toro
 * @Author: Fernando Joel Cruz Briones
 * Version 1.0
 */

public class LimeLightSubsystem extends SubsystemBase {
    
    //Creamos la red de la limeLight
    private final NetworkTable limeLightTable;

    //Creamos ahora si lo que es el subsystema de nuestra LimeLight, 
    public LimeLightSubsystem() {
        limeLightTable = NetworkTableInstance.getDefault().getTable("limelight");
        limeLightTable.getEntry("pipeline").setNumber(0); // Remove when its solved
    }
    
    //Publicamos de manera peoriodica en el SmarthDashBoard los valores que obtendremos
    @Override
    public void periodic() {        
        SmartDashboard.putNumber("Target ID", getID());
        SmartDashboard.putNumber("Target TX", getYaw());
        SmartDashboard.putNumber("Target TY", getTY());
        SmartDashboard.putNumber("Speaker Distance", getDistance(LimeConstants.kTargetSpeakertHeight));
        SmartDashboard.putNumber("Speaker Error", getDistanceError(LimeConstants.kTargetSpeakertHeight, LimeConstants.kDistanceToSpeaker));
        SmartDashboard.putNumber("Amp Distance", getDistance(LimeConstants.kTargetAmpHeight));
        SmartDashboard.putNumber("Amp Error", getDistanceError(LimeConstants.kTargetAmpHeight, LimeConstants.kDistanceToAmp));
    }

    //Funcion para conseguir el Yaw o la posicion horizontal del objetivo
    public double getYaw() {
        return limeLightTable.getEntry("tx").getDouble(0);
    }
    
    //Funcion para conseguir la posicion vertical del objetivo
    public double getTY() {
        return limeLightTable.getEntry("ty").getDouble(0);
    }
    
    //Funcion para conseguir el ID de la April Tag
    public double getID() {
        return limeLightTable.getEntry("tid").getDouble(0);
    }

    //Funcion para calcular la distnacia horizontal entre el objetivo y el robot
    public double getDistance(double targetHeight) { 
        double targetOffsetAngle_Vertical = getTY();
        return (targetHeight - LimeConstants.cameraHeight) / Math.tan(Math.toRadians(LimeConstants.cameraAngle + targetOffsetAngle_Vertical));
    }

    //Funcion para conseguir el Error de la distancia entre el robot y el objetivo
    public double getDistanceError(double height, double distance) { // Calculates error from optimal distance
        return (getDistance(height) - distance);
    }

    //Funcion para asegurarnos que el robot este aliniado
    public boolean isTargetAligned(double height, double distance, double dE, double gY) { // Checks if robot is aligned
        return Math.abs(getDistanceError(height, distance)) < dE && Math.abs(getYaw()) < gY;
    }

    //Funcion para asignar una Pipeline
    public void setPipeline(int pipeline) {
        limeLightTable.getEntry("pipeline").setNumber(pipeline);
    }

    //Funcion para cambiar los leds de la limelight
    public void setLimeLed(int mode) {
        limeLightTable.getEntry("ledMode").setNumber(mode);
        // 0: Solid
        // 1: Off
        // 2: Blink
    }
}
