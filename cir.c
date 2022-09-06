#include "cir.h"

cir_data_t get_CIR_data(){
  uint8_t sample[6];
  for (int index = 0; index < 500; index++){
        dwt_readaccdata(sample, sizeof(sample), index);
        int32_t real = 0;
        real =  sample[3] << 16;
        real += sample[2] << 8;
        real += sample[1];
        if (real & 0x020000)  // MSB of 18 bit value is 1
            real |= 0xfffc0000;
        int32_t img = 0;
        img =  sample[6] << 16;
        img += sample[5] << 8;
        img += sample[4];
        if (img & 0x020000)  // MSB of 18 bit value is 1
            img |= 0xfffc0000;
        printf("%d, %d, %f\n", real, img, sqrt(real*real + img*img));
      }
}