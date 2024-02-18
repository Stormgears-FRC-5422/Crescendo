package frc.utils.configfile;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.io.File;
import java.io.FileInputStream;
import java.io.IOException;
import java.util.Arrays;
import java.util.HashMap;
import java.util.Properties;
import java.util.Set;

public class StormProp {
    // Maybe rename this file to something better
    private static final String path =
      Filesystem.getDeployDirectory().getPath(); // "/home/lvuser/deploy";
    private static final String name = "config.properties";
    private static final String backUP = "config_backup.properties";
    private static final File configFile = new File(path, name);

    // In support of auto-detection
    private static final String rcPath = "/home/lvuser";
    private static final String rcName = ".stormrc";
    private static final File rcFile = new File(rcPath, rcName);

    private static final HashMap<String, Double> m_number_map = new HashMap<>();
    private static final HashMap<String, Integer> m_int_map = new HashMap<>();
    private static final HashMap<String, Boolean> m_bool_map = new HashMap<>();
    private static final HashMap<String, String> m_string_map = new HashMap<>();
    private static Properties properties;
    private static Properties overrideProperties;
    private static Properties simProperties;
    private static boolean initialized = false;
    private static boolean overrideInitialized = false;
    private static boolean simInitialized = false;
    private static boolean debug = false;



    public static void init() {
        System.out.println("Running in directory " + System.getProperty("user.dir"));
        System.out.println("Trying to use file " + configFile.getAbsolutePath());
        properties = new Properties();
        FileInputStream inputStream = null;
        try {
            inputStream = new FileInputStream(configFile);
            properties.load(inputStream);
            System.out.println("*****************");
            System.out.println("Loading Properties");
            System.out.println("*****************");
        } catch (IOException e) {
            System.out.println("Using backup config file");
            try {
                inputStream = new FileInputStream(new File("/home/lvuser/deploy", backUP));
                properties.load(inputStream);
            } catch (IOException w) {
                System.out.println("Failed to find back up file. " + e.getMessage());
            }
        } finally {
            if (inputStream != null) {
                try {
                    inputStream.close();
                } catch (IOException e) {
                    System.out.println("Error coming from config. This should not run. "  + e.getMessage());
                }
            }
        }
        initialized = true;
        if (!overrideInitialized) {
            overrideInit();
        }

        debug = getBoolean("debugProperties", false);
    }

    public static void overrideInit() {
        String overrideName = RobotBase.isSimulation()
            ? properties.getProperty("simOverride")
            : properties.getProperty("override");

        overrideName = removeCast(overrideName);

        if ( overrideName.equalsIgnoreCase("auto")) {
            System.out.println("Using AUTOMATIC configuration");
            Properties rcProperties = new Properties();
            FileInputStream rcStream = null;
            try {
                System.out.println("rcFile path: " + rcFile.getAbsolutePath());
                rcStream = new FileInputStream(rcFile);
                rcProperties.load(rcStream);
                System.out.println("autoConfig setting " + rcProperties.getProperty("autoConfig"));
                overrideName = properties.getProperty(rcProperties.getProperty("autoConfig"));
            } catch (Exception e) {
                System.out.println("Failed to find autoConfig file... " + e.getMessage());
            } finally {
                if (rcStream != null) {
                    try {
                        rcStream.close();
                    } catch (IOException e) {
                        System.out.println("Error coming from autoConfig. This should not run. " + e.getMessage());
                    }
                }
            }
        }

        System.out.println("Using override file " + overrideName);
        File overrideConfigFile = new File(path, overrideName);
        overrideProperties = new Properties();

        FileInputStream OverrideInputStream = null;
        try {
            OverrideInputStream = new FileInputStream(overrideConfigFile);
            overrideProperties.load(OverrideInputStream);
        } catch (IOException e) {
            System.out.println("!!! No override file detected !!!");
        }
        if (OverrideInputStream != null) {
            try {
                OverrideInputStream.close();
            } catch (IOException e) {
                System.out.println("Error coming from override config. This should not run");
            }
        }

        overrideInitialized = true;
    }

    public static void simInit() {
        simProperties = new Properties();

        // Ignore the sim overrides file unless we are in simulation!
        if (!RobotBase.isSimulation()) {
            simInitialized = true;
            return;
        }

        String simName = properties.getProperty("simOverrideOverride");
        simName = removeCast(simName);

        System.out.println("Using simulation override file " + simName);
        File simConfigFile = new File(path, simName);

        FileInputStream SimOverrideInputStream = null;
        try {
            SimOverrideInputStream = new FileInputStream(simConfigFile);
            simProperties.load(SimOverrideInputStream);
        } catch (IOException e) {
            System.out.println("!!! No simulation override file detected !!!");
        }
        if (SimOverrideInputStream != null) {
            try {
                SimOverrideInputStream.close();
            } catch (IOException e) {
                System.out.println("Error coming from simulation override config. This should not run");
            }
        }

        simInitialized = true;
    }

    private static String removeCast(String value) {
        return value.substring(value.indexOf(")") + 1).trim();
    }
    private static String getPropString(String key) {
        if (!initialized) init();
        if (!overrideInitialized) overrideInit();
        if (!simInitialized) simInit();

        if (simProperties.containsKey(key)) return(removeCast(simProperties.getProperty(key)));
        if (overrideProperties.containsKey(key)) return(removeCast(overrideProperties.getProperty(key)));
        if (properties.containsKey(key)) return(removeCast(properties.getProperty(key)));

        return "";
    }

    public static String getString(String key, String defaultVal) {
        String result = getStringInternal(key,defaultVal);
        if(debug) System.out.println("debug property " + key + " = " + result);

        return result;
    }

    public static String getString(String prefix, String key, String defaultVal) {
        String result = (prefix.equals("general")) ? getStringInternal(key,defaultVal) : getStringInternal(prefix + "." + key,defaultVal);

        if(debug) System.out.println("debug property " + prefix + "." + key + " = " + result);

        return result;
    }

    private static String getStringInternal(String key, String defaultVal) {
        try {
            if (m_string_map.containsKey(key)) return (m_string_map.get(key));
            else if (getPropString(key) != null) {
                m_string_map.put(key, getPropString(key));
                return (m_string_map.get(key));
            } else {
                System.out.println("WARNING: default used for key " + key);
                return (defaultVal);
            }
        } catch (Exception e) {
            System.out.println("WARNING: default used for key " + key);
            return (defaultVal);
        }
    }

    public static double getNumber(String key, Double defaultVal) {
        double result = getNumberInternal(key, defaultVal);
        if(debug) System.out.println("debug property " + key + " = " + result);

        return result;
    }

    public static double getNumber(String prefix, String key, Double defaultVal) {
        double result = (prefix.equals("general")) ? getNumberInternal(key,defaultVal) : getNumberInternal(prefix + "." + key,defaultVal);
        if(debug) System.out.println("debug property " + prefix + "." + key + " = " + result);

        return result;
    }
    private static double getNumberInternal(String key, Double defaultVal) {
        try {
            if (m_number_map.containsKey(key)) return (m_number_map.get(key));
            else if (getPropString(key) != null) {
                m_number_map.put(key, Double.parseDouble(getPropString(key)));
                return (m_number_map.get(key));
            } else {
                System.out.println("WARNING: default used for key " + key);
                return (defaultVal);
            }
        } catch (Exception e) {
            System.out.println("WARNING: default used for key " + key);
            return (defaultVal);
        }
    }

    public static int getInt(String key, int defaultVal) {
        int result = getIntInternal(key, defaultVal);
        if(debug) System.out.println("debug property " + key + " = " + result);

        return result;
    }
    public static int getInt(String prefix, String key, int defaultVal) {
        int result = (prefix.equals("general")) ? getIntInternal(key,defaultVal) : getIntInternal(prefix + "." + key,defaultVal);
        if(debug) System.out.println("debug property " + prefix + "." + key + " = " + result);

        return result;
    }


    private static int getIntInternal(String key, int defaultVal) {
        try {
            if (m_int_map.containsKey(key)) return (m_int_map.get(key));
            else if (getPropString(key) != null) {
                m_int_map.put(key, Integer.parseInt(getPropString(key)));
                return (m_int_map.get(key));
            } else {
                System.out.println("WARNING: default used for key " + key);
                return (defaultVal);
            }
        } catch (Exception e) {
            System.out.println("WARNING: default used for key " + key);
            return (defaultVal);
        }
    }

    public static boolean getBoolean(String key, Boolean defaultVal) {
        boolean result = getBooleanInternal(key, defaultVal);
        if(debug) System.out.println("debug property " + key + " = " + result);

        return result;
    }

    public static boolean getBoolean(String prefix, String key, Boolean defaultVal) {
        boolean result = (prefix.equals("general")) ? getBooleanInternal(key,defaultVal) : getBooleanInternal(prefix + "." + key,defaultVal);
        if(debug) System.out.println("debug property " + key + " = " + result);

        return result;
    }

    private static boolean getBooleanInternal(String key, Boolean defaultVal) {
        try {
            if (m_bool_map.containsKey(key)) return (m_bool_map.get(key));
            else if (getPropString(key) != null) {
                m_bool_map.put(key, getPropString(key).equalsIgnoreCase("true"));
                return (m_bool_map.get(key));
            } else {
                System.out.println("WARNING: default used for key " + key);
                return (defaultVal);
            }
        } catch (Exception e) {
            System.out.println("WARNING: default used for key " + key);
            return (defaultVal);
        }
    }

    public static void toSmartDashBoard() {
        if (!initialized) {
            init();
        }
        if (!overrideInitialized) {
            overrideInit();
        }
        String[] Blacklist = {"robotName", "hasNavX", "rearRightTalonId", "rearLeftTalonId", "frontRightTalonId", "frontLeftTalonId", "wheelRadius", "navXconnection"};
        Set<String> keys = properties.stringPropertyNames();
        for (String key : keys) {
      if (!Arrays.asList(Blacklist).contains(key)) {
        //                SmartDashboard.putString(key, getPropString(key));
      }
        }
    }

    //updates properties object using values from SmartDashboard
    public static void updateProperties() {
        if (!initialized) {
            init();
        }
        Set<String> keys = properties.stringPropertyNames();
        for (String key : keys) {
            String str = SmartDashboard.getString(key, "MISSING");
            if (!str.equals("MISSING")) {
                overrideProperties.setProperty(key, str);
                if (m_int_map.containsKey(key)) m_int_map.put(key, Integer.parseInt(str));
                else if (m_number_map.containsKey(key)) m_number_map.put(key, Double.parseDouble(str));
                else if (m_bool_map.containsKey(key)) m_bool_map.put(key, str.equalsIgnoreCase("true"));
                else if (m_string_map.containsKey(key)) m_string_map.put(key, str);
            }
        }
    }



}
