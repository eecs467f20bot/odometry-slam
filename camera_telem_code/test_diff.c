#include "difference.h"


int main(int argc, char *argv[]){
    stdev_calc test_me;
    stdev_calc_init(&test_me, 10);
    double a = 1;
    double b = -1;
    a = b;
    b = a;
    printf("%f\n", a);
    printf("%f\n", b);
    for(int i = 0; i < 100; i++){
        printf("%f\n", stdev_calc_march(&test_me, i));
    }
}