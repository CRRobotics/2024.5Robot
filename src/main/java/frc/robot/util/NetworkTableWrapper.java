package frc.robot.util;

import edu.wpi.first.networktables.NetworkTableInstance;

public class NetworkTableWrapper {
    public static double getDouble(String table, String key) {
        return NetworkTableInstance.getDefault().getTable(String.valueOf(table)).getEntry(key).getDouble(0);
    }
    
    public static double[] getArray(String table, String key) {
        return NetworkTableInstance.getDefault().getTable(String.valueOf(table)).getEntry(key).getDoubleArray(new double[]{});
    }
}
