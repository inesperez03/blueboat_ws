#include "bindings.h"
#include <iostream>

int main() {
  set_raspberry_pi_version(Raspberry::Pi4);
  set_navigator_version(NavigatorVersion::Version1);   // luego prueba Version2
  init();
  std::cout << "Temp: " << read_temp() << std::endl;
  return 0;
}
