file(REMOVE_RECURSE
  "CMakeFiles/build"
  "build/build.elf"
  "build/obj/clocks.o"
  "build/obj/control_state.o"
  "build/obj/controller.o"
  "build/obj/controller_tests.o"
  "build/obj/delay.o"
  "build/obj/eic.o"
  "build/obj/foc.o"
  "build/obj/foc_math.o"
  "build/obj/foc_math_fpu.o"
  "build/obj/gpio.o"
  "build/obj/i2c.o"
  "build/obj/inc_encoder.o"
  "build/obj/main.o"
  "build/obj/mcp23017.o"
  "build/obj/motor_encoder.o"
  "build/obj/motor_unit.o"
  "build/obj/pwm.o"
  "build/obj/roller.o"
  "build/obj/sensor_state.o"
  "build/obj/sercom.o"
  "build/obj/simple_filter.o"
  "build/obj/spi.o"
  "build/obj/startup.o"
  "build/obj/state_recorder.o"
  "build/obj/stopwatch.o"
  "build/obj/system.o"
  "build/obj/tension_arm.o"
  "build/obj/timer.o"
  "build/obj/uart.o"
  "build/obj/uha_motor_driver.o"
)

# Per-language clean rules from dependency scanning.
foreach(lang )
  include(CMakeFiles/build.dir/cmake_clean_${lang}.cmake OPTIONAL)
endforeach()
