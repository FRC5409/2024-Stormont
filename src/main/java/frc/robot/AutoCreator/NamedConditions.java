package frc.robot.AutoCreator;

import java.util.HashMap;
import java.util.function.BooleanSupplier;

public class NamedConditions {

    private static HashMap<String, BooleanSupplier> conditions = new HashMap<>();

    public static void registerCondition(String conditionName, BooleanSupplier supplier) {
        conditions.put(conditionName, supplier);
    }

    public static BooleanSupplier getCondition(String conditionName) {
        return conditions.get(conditionName);
    }
}
