#include "types.h"
#include "stat.h"
#include "user.h"

char *str = "You can't change a character!";

int
main(int argc, char *argv[])
{
    str[1] = 'O';
    printf(1, str);
    return 0;
}