package frc.utils.generator;

public class PropItem {
    String type;
    String key;
    String defaultVal;

    public PropItem(String type, String key, String defaultVal) {
        this.type = type;
        this.key = key;
        this.defaultVal = defaultVal;
    }

    private String quote(String s) {
        return "\"" + s + "\"";
    }

    @Override
    public String toString() {
        return switch (type.toLowerCase()) {
            case "string" ->
                    "public static final String " + key + " = " + "StormProp.getString(" + quote(key) + "," + quote(defaultVal) + ");";
            case "int" ->
                    "public static final int " + key + " = " + "StormProp.getInt(" + quote(key) + "," + defaultVal + ");";
            case "number" ->
                    "public static final double " + key + " = " + "StormProp.getNumber(" + quote(key) + "," + defaultVal + ");";
            case "boolean" ->
                    "public static final boolean " + key + " = " + "StormProp.getBoolean(" + quote(key) + "," + defaultVal + ");";
            default -> "Error-valid method not given";
        };
    }
}
