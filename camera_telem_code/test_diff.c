#include "difference.h"


int main(int argc, char *argv[]){
    stdev_calc test_me;
    stdev_calc_init(&test_me, 10);
    for(int i = 0; i < 100; i++){
        printf("%f\n", stdev_calc_march(&test_me, i));
    }
}