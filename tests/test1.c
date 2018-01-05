#include <stdio.h>
 
int main()
{

int a;
int b;
a=0x10;
b=0x60;

a=a &~(b);

    printf("a = %d \n", a);
    return 0;
}
