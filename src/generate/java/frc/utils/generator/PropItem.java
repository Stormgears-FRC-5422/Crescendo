package frc.utils.configfile;

public class propItem {
    String type;
    String key;
    String defaultVal;

    public propItem(String type, String key, String defaultVal) {
        this.type = type;
        this.key = key;
        this.defaultVal = defaultVal;
    }

    private String quote(String s) {
        return "\"" + s + "\"";
    }

    @Override
    public String toString() {
        switch (type.toLowerCase()) {
            case "string":
                return "public static final String " + key + " = " + "StormProp.getString(" + quote(key) + "," + quote(defaultVal) + ");";
            case "int":
                return "public static final int " + key + " = " + "StormProp.getInt(" + quote(key) + "," + defaultVal + ");";
            case "number":
                return "public static final double " + key + " = " + "StormProp.getNumber(" + quote(key) + "," + defaultVal + ");";
            case "boolean":
                return "public static final boolean " + key + " = " + "StormProp.getBoolean(" + quote(key) + "," + defaultVal + ");";
            default:
                return "Error-valid method not given";
        }
    }
}
