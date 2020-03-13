# HCSR04 - ATTiny85

Speed computation can be tested with the following code

```cpp
#include <sstream>
#include <iostream>
#include <iomanip>
#include <inttypes.h>

uint8_t distance;
uint8_t TCNT1 = 231;


std::string uint8_to_hex_string(const uint8_t *v, const size_t s) {
  std::stringstream ss;
  ss << std::hex << std::setfill('0');
  for (int i = 0; i < s; i++) {
    ss << std::hex << std::setw(2) << static_cast<int>(v[i]);
  }
  return ss.str();
}

int main(int argc, char const *argv[]) {
  uint32_t cm = TCNT1 * 72176;
  distance = uint8_t(cm >> 16);
  std::cout << "distance : 0x" << uint8_to_hex_string(&distance, 1) << std::endl;
  return 0;
}
```
