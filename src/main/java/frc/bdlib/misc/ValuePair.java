package frc.bdlib.misc;

public record ValuePair<K, V>(K left, V right) {
    public static <K, V> ValuePair<K, V> of(K obj1, V obj2) {
        return new ValuePair<>(obj1, obj2);
    }
}
