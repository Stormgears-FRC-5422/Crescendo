package frc.utils.data;

import edu.wpi.first.networktables.*;

import java.util.HashMap;
import java.util.Vector;

public class StormStruct {
    // Describes a data structure that has been published and described in network tables
    // Can unpack binary data once intialized
    private String[] m_fieldNames;
    private final String m_struct_name;
    private final HashMap<String, Integer> m_fields; // key is field name, data is position index in binary data
    private final HashMap<String, Integer> m_encodings; // key is field name, data is byte size in binary data
    private int m_struct_size;
    private int m_typeid;
    private final NetworkTableInstance m_ntinst;
    private final NetworkTable m_base_table;
    private final StringArraySubscriber m_names_sub;
    private final IntegerArraySubscriber m_encodings_sub;
    private final IntegerSubscriber m_type_sub;
    private final HashMap<String,RawSubscriber> m_data_subscribers;
    private boolean m_initialized;


    /**  
    * Populate HashMap fields and size by reading definition of struct_name from network tables (populated by struct provider)
    * 
    * @param nt_inst Network Tables instance
    * @param base_table table name where struct data is stored
    * @param struct_name name of struct being used
    */
    public StormStruct(final NetworkTableInstance nt_inst,final String base_table,final String struct_name) {
        this.m_ntinst = nt_inst;
        this.m_fields = new HashMap<String,Integer>();
        this.m_encodings = new HashMap<String,Integer>();
        this.m_data_subscribers = new HashMap<String,RawSubscriber>();
        this.m_struct_name = struct_name;
        this.m_struct_size = 0;  // number of bytes in struct
        this.m_initialized = false;

        this.m_base_table = nt_inst.getTable(base_table);

//        name_topic = table.getStringArrayTopic("structs/" + type + "/names")

        StringArrayTopic names_topic = m_base_table.getStringArrayTopic("structs/" + struct_name + "/names");
        IntegerArrayTopic encodings_topic = m_base_table.getIntegerArrayTopic("structs/" + struct_name + "/encodings");
        IntegerTopic type_topic = m_base_table.getIntegerTopic("structs/" + struct_name + "/type");

        this.m_names_sub = names_topic.subscribe(new String[0]);
        this.m_encodings_sub = encodings_topic.subscribe(new long[0]);
        this.m_type_sub = type_topic.subscribe(-1);

        this.initialize();
    }

    /** Call this when the data structure has been published to network tables */
    public boolean initialize() {
        if (!m_initialized) {
            this.m_typeid = (int) this.m_type_sub.get();
            if (this.m_typeid != -1) {
                String[] names = this.m_names_sub.get();
                long[] encodings = this.m_encodings_sub.get();
                this.m_fieldNames = new String[names.length];

                for (int i=0;i<names.length; i++) {
                    this.m_fields.put(names[i],i);
                    this.m_encodings.put(names[i],(int) encodings[i]);
                    // RESUME HERE - change for new encoding format
                    this.m_struct_size += (encodings[i] & 0x3) + 1;  // bits 1:0 encode the number of bytes (max 4)
                    this.m_fieldNames[i] = names[i];
                }
                m_initialized = true;
            }
        }
        return(m_initialized);
    }

    /**
     * Get specifc data by name from network tables. returns a list of hashmaps.
     * @return Vector<HashMap<String,Double>>
     */
    public Vector<HashMap<String,Double>> get_data(String name) {
        if (!initialize()) return new Vector<>();
        // Create subscriber if it doesn't exist
        RawSubscriber sub;
        Vector<HashMap<String,Double>> data_list;
        if (!m_data_subscribers.containsKey(name)) {
            sub = m_base_table.getRawTopic("binary_data/" + this.m_struct_name + "/" + name)
                    .subscribe(this.m_struct_name,new byte[0]);
            m_data_subscribers.put(name, sub);
        } else {
            sub = m_data_subscribers.get(name);
        }

        // Get raw data
        TimestampedRaw timestamped_data = sub.getAtomic();
        byte[] raw_data = timestamped_data.value;
        double timestamp = timestamped_data.serverTime;
        if (raw_data.length > 0) {
            // Decode raw data into HashMap
            data_list = this.unpack(raw_data);
            for (var data : data_list) {
                data.put("timestamp", timestamp);
            }
        } else {
            data_list = new Vector<>();
        }

        // Return Data
        return data_list;
    }

    /**
     * Get the data structure size
     * @return Size of data structure
     */
    public int get_size() {
        return(m_struct_size);
    }

        /**
         * Get the field name given the index
         * 
         * @param index of data item
         * @return Name of field
         */
    public String get_name_by_index(final int index) {
        for (final String field : m_fields.keySet()) {
            if (m_fields.get(field) == index) {
                return(field);
            }
        }
        return("");
    }

    /**
         * Unpack a binary byte stream using this structure definition
         * 
         * @param data_stream The binary data
         * @return Returns a HashMap of field,value pairs
         * 
    */
    public HashMap<String, Double> decode_struct(final byte[] data_stream) {
        return(decode_struct(data_stream,0));
    }

    /**
     * Unpack an entire binary stream into a list of HashMaps
     * @param data_stream
     * @return List (vector) of hashmaps.  each hashmap represents a data structure being transferred
     */
    public Vector<HashMap<String, Double>> unpack(final byte[] data_stream) {
        Vector<HashMap<String, Double>> list;
        if (data_stream.length < 3) {
            //System.out.println("data received from Raspberry Pi is too short. Expect 3 or longer, but received " + data_stream.length + " bytes");
            return(new Vector<HashMap<String, Double>>());
        }
        else {
            list = new Vector<HashMap<String, Double>>();
            byte id = data_stream[0];
            int count = (data_stream[1] << 8) + data_stream[2];

            int offset = 3;
            for (int i=0; i< count; i++) {
                list.add(decode_struct(data_stream,offset));
                offset += get_size();  // Move pointer to next Struct
            }
            return(list);
        }
    }

    /**
     * Unpack a binary byte stream using this structure definition starting at a given offset
     *
     * @param data_stream The binary data
     * @param _offset The offset in the binary data stream to extract from
     * @return Returns a HashMap of field,value pairs, values are always doubles
     *
     */
    private HashMap<String, Double> decode_struct(final byte[] data_stream,final int _offset) {
        HashMap<String,Double> ret_map = new HashMap<String,Double>();
        // Data is big endian
        int offset = _offset;
        for (String field : m_fieldNames) {
            if (field.startsWith("_")) {
                //System.out.println("Unpack - skipping " + field);
                continue;
            }
            //System.out.println("unpack " + field);
            int data = 0;
            int encoding  = this.m_encodings.get(field).intValue();
            int precision = (encoding & 0x7f) >> 2;
            int size = (encoding & 0x3) + 1;

            boolean signed = (encoding & 0x80) != 0;
            // Extend sign on signed int. ints are 4 bytes in java.
            if (signed && size < 4) {
                if ((data_stream[offset] & 0x80) != 0) {  // negative number, big endian
                    data = 0xFFFFFFFF;  // sign extend, this bits will be shifted up as we grab bytes from the stream
                }
            }


            //System.out.println("unpack " + field + "; size=" + size);
            for (int i = 0; i < size; i++) {
                data = data << 8;
                data |= data_stream[offset + i] & 0xFF;
            }

            offset += size;

            double ddata = data * 1.0;
            if (precision > 0) {
                ddata = ddata/Math.pow(10,precision);
            }
            ret_map.put(field,ddata);
        }
        return(ret_map);
    }
}
