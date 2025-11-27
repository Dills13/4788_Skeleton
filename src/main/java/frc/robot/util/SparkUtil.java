package frc.robot.util;

import com.revrobotics.REVLibError;
import java.util.function.Supplier;

public class SparkUtil {
  public static void tryUntilOk(int maxAttempts, Supplier<REVLibError> command) {
    for (int i = 0; i < maxAttempts; i++) {
      var error = command.get();
      if (error == REVLibError.kOk) break;
    }
  }
}
