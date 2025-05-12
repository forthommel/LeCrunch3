#include "LeCrunch3/VICPClient.h"

int main() {
  VICPClient dso("pool05240024");
  dso.connectToDevice();
  dso.disconnectFromDevice();

  dso.sendDataToDevice("*IDN?", true);

  return 0;
}
