package arachne4.lib;

import java.lang.annotation.Annotation;
import java.lang.annotation.ElementType;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.Target;
import java.lang.reflect.Constructor;
import java.lang.reflect.InvocationTargetException;
import java.util.Collections;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;

import arachne4.lib.logging.ArachneLogger;
import arachne4.lib.subsystems.Subsystem;
import io.github.classgraph.AnnotationClassRef;
import io.github.classgraph.AnnotationInfo;
import io.github.classgraph.AnnotationParameterValue;
import io.github.classgraph.ClassGraph;
import io.github.classgraph.ClassInfo;
import io.github.classgraph.ScanResult;

public class Constants {
    private static final ArachneLogger log = ArachneLogger.getInstance();
    private static final Set<Class<?>> superclassesToIgnore = Set.of(Object.class);

    private static Map<Class<?>, Object> constants = Map.of();

    @SuppressWarnings("unchecked")
    public static <T> T get(Class<T> constantsClass) {
        return (T) constants.get(constantsClass);
    }

    static void setRobotClass(Class<? extends Subsystem> robotClass) {
        constants = createMap(robotClass);
    }

    private static Map<Class<?>, Object> createMap(Class<?> robotClass) {
        Map<Class<?>, Object> map = new HashMap<>();
        log.info("Loading constants files for robot: " + robotClass.getSimpleName());

        Class<? extends Annotation> annotation = RobotConstants.class;
        try (ScanResult scanResult = new ClassGraph()
                    .enableClassInfo()
                    .enableAnnotationInfo()
                    .scan()) {
            for (ClassInfo constantsClassInfo : scanResult.getClassesWithAnnotation(annotation)) {
                AnnotationInfo annotationInfo = constantsClassInfo.getAnnotationInfo(annotation);
                List<AnnotationParameterValue> annotationParamVals = annotationInfo.getParameterValues();
                Class<?> requiredRobotClass = ((AnnotationClassRef) annotationParamVals.get(0).getValue()).loadClass();

                if (requiredRobotClass.equals(robotClass)) {
                    loadConstantsToMap(map, constantsClassInfo.loadClass());
                }
            }
        }

        expandMapWithSuperclassKeys(map);

        for (var value : Set.copyOf(map.values())) {
            log.info(String.format(
                "Constants entries %s map to %s",
                map.entrySet().stream()
                    .filter(entry -> entry.getValue().equals(value))
                    .map(Map.Entry::getKey)
                    .map(Class::getSimpleName)
                    .sorted()
                    .toList(),
                value.getClass().getSimpleName()));
        }

        return map;
    }

    private static void expandMapWithSuperclassKeys(Map<Class<?>, Object> map) {
        Map<Class<?>, Class<?>> candidateMappings = new HashMap<>();
        Set<Class<?>> duplicatedSuperclasses = new HashSet<>();

        for (Class<?> key : map.keySet()) {
            for (Class<?> superclass : getSuperclassesAndInterfacesRecursively(key)) {
                if (superclass == key) {
                    continue;
                }
                else if (candidateMappings.containsKey(superclass)) {
                    log.error(String.format(
                        "Constants files %s and %s share the same superclass %s. Neither will be accessible via the superclass' key.",
                        candidateMappings.get(superclass).getSimpleName(),
                        key.getSimpleName(),
                        superclass.getSimpleName()));

                    candidateMappings.remove(superclass);
                    duplicatedSuperclasses.add(superclass);
                }
                else if (duplicatedSuperclasses.contains(superclass)) {
                    log.error(String.format(
                        "Constants file %s also shares the same superclass %s as previous constants files. None will be accessible via the superclass' key.",
                        key.getSimpleName(),
                        superclass.getSimpleName()));
                }
                else if (map.containsKey(superclass)) {
                    log.error(String.format(
                        "Constants file %s is a subclass of an existing constants file %s. %s will only be accessible via its exact class, and %s will only be accessible by classes not shared by %s's heirarchy.",
                        key.getSimpleName(),
                        superclass.getSimpleName(),
                        superclass.getSimpleName(),
                        key.getSimpleName(),
                        superclass.getSimpleName()));
                }
                else {
                    candidateMappings.put(superclass, key);
                }
            }
        }

        for (var entry : candidateMappings.entrySet()) {
            map.put(entry.getKey(), map.get(entry.getValue()));
        }
    }

    private static Set<Class<?>> getSuperclassesAndInterfacesRecursively(Class<?> clazz) {
        if (superclassesToIgnore.contains(clazz)) return Collections.emptySet();

        Set<Class<?>> results = new HashSet<>();
        results.add(clazz);

        if (clazz.getSuperclass() != null) results.addAll(getSuperclassesAndInterfacesRecursively(clazz.getSuperclass()));
        for (Class<?> interfaceClass : clazz.getInterfaces()) results.addAll(getSuperclassesAndInterfacesRecursively(interfaceClass));

        return results;
    }

    private static void loadConstantsToMap(Map<Class<?>, Object> map, Class<?> constantsClass) {
        try {
            Constructor<?> constructor = constantsClass.getDeclaredConstructor();
            constructor.setAccessible(true);

            map.put(constantsClass, constructor.newInstance());

            log.info("Successfully loaded constants file: " + constantsClass.getName());
        } catch (NoSuchMethodException e) {
            log.critical(String.format("Failed to instantiate constants file " + constantsClass.getName() + ". Constants files must have a no-argument constructor."));
        } catch (InstantiationException | IllegalAccessException | IllegalArgumentException | InvocationTargetException | SecurityException e) {
            log.critical(String.format("Failed to instantiate constants file " + constantsClass.getName() + " due to " + e.getClass().getSimpleName()));
        }
    }

    @Retention(RetentionPolicy.RUNTIME)
    @Target(ElementType.TYPE)
    public static @interface RobotConstants {
        Class<? extends Subsystem> value();
    }
}
