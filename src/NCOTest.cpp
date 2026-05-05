#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstdint>
#include "G2init.h"
#include "NCO.h"

using namespace std;

int main(int argc, char *argv[])
{
  uint8_t PRN = 9;
  int8_t bin = 0;
  int16_t codephase = 0;
  uint16_t width = 500;
  uint32_t idx = 0;
  float RefFreq = 40920000;
  float CodeFreqBasis = 1.023e6;
  float CarrreqBasis = 4.092e6;
  float SampleFreq = 16.368e6;

  if (argc >= 2)
    PRN = atoi(argv[1]);
  if (argc >= 3)
    codephase = atoi(argv[2]);
  if (argc >= 4)
    bin = atoi(argv[3]);
  printf("PRN: %d CP:%d bin: %d\n",
         PRN, codephase, bin);

G2INIT sv(PRN, codephase);

for (idx=0; idx<1023; idx++){
  printf("%x", sv.CACODE[idx]);
}
printf("\n");

NCO CODENCO(0, SampleFreq);
CODENCO.SetFrequency(CodeFreqBasis);
CODENCO.RakeSpacing(halfChip);
//CODENCO.RakeSpacing(Narrow);
CODENCO.LoadCACODE(sv.CACODE);
//CODENCO.LoadCACODE(sv.CODE);

for (idx=0; idx<128; idx++) {
  printf("%.16llX %2d %2d %2d %4d %1d\n",
   CODENCO.EPLreg&0xFFFFFFFF, CODENCO.Early, CODENCO.Prompt, CODENCO.Late,
   CODENCO.rotations, CODENCO.CACODE[CODENCO.rotations]);
  CODENCO.clk();
  if ((idx+1)%16 == 0) printf("\n");
}
}