package frc.lib.util;

import java.util.TreeMap;

public class InterpolatableTreeMap<T> {
    private final TreeMap<Double, Interpolatable<T>> treeMap = new TreeMap<Double, Interpolatable<T>>();

    public InterpolatableTreeMap() {

    }
    
    /** Add to interpolatable map with key and data (interDouble or interPose2d). 
     * @param key Key to set value with
     * @param value Interpolatable value (Interpolatable.interDouble or interPose2d)
    */
    public void set(double key, Interpolatable<T> value) {
        treeMap.put(key, value);
    }

    /** Get interpolated entry with given key. 
     * @param key Key to get interpolated entry
    */
    public T get(double key) {
        if (treeMap.isEmpty()) return null;

        var value = treeMap.get(key);
        if (value != null) return value.get();

        // Returns the entry with the least key that is greater than or equal to the key.
        var topBound = treeMap.ceilingEntry(key);
        // Returns the entry with the greatest key that is less than or equal to the key.
        var bottomBound = treeMap.floorEntry(key);

        if (topBound == null) {
            return bottomBound.getValue().get();
        } else if (bottomBound == null) {
            return topBound.getValue().get();
        } else {
            return bottomBound.getValue().interpolate(topBound.getValue(),
                (key - bottomBound.getKey()) / (topBound.getKey() - bottomBound.getKey())
            );
        }
    }  
}