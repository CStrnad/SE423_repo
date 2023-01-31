#include <stdio.h>
#include <stdlib.h>

/* generate multiple lines of output */
int main()
{
	int rows;

	/* get and verify input */
	printf("How many rows (18 max)? ");
	scanf("%d",&rows);
	/* avoid out-of-range values */

    //Add an if-test to confirm the input is less that 18. 
    //Otherwise reset the value of variable rows to 18
    if (rows > 18) { 
        rows = 18; 
    }
    
	/* process the rows */
	printf("I will process %d rows\n",rows);

    //Use binary math to halve the value input. display this value.
    printf("%d is half of %d\n", (rows >> 1), rows);
    //Use binary math to double the value input. display this value.
    printf("%d is double of %d\n", (rows << 1), rows);

	return 0;
}